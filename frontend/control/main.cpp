// engine-sim-control: a small window for changing engine-sim's settings while it runs (GUI or
// headless). Talks to it over the control socket; everything applies live.

#include "control_client.h"
#include "launcher.h"

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlgpu3.h>
#include <SDL3/SDL.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

using nlohmann::json;
using clock_type = std::chrono::steady_clock;

namespace {

const ImVec4 Green(0.29f, 0.87f, 0.50f, 1.0f);
const ImVec4 Orange(1.00f, 0.67f, 0.24f, 1.0f);
const ImVec4 Red(1.00f, 0.39f, 0.35f, 1.0f);
const ImVec4 Muted(0.60f, 0.60f, 0.65f, 1.0f);

// Reads a field of the expected type, or the fallback: the game sends whatever its data holds
// (names can be translation objects, numbers can be missing), and the panel must not crash on it
template <typename T>
T field(const json &object, const char *key, T fallback) {
    if (!object.is_object()) return fallback;
    const auto it = object.find(key);
    if (it == object.end()) return fallback;
    if constexpr (std::is_same_v<T, json>) return *it;
    else if constexpr (std::is_same_v<T, std::string>) return it->is_string() ? it->template get<std::string>() : fallback;
    else if constexpr (std::is_same_v<T, bool>) return it->is_boolean() ? it->template get<bool>() : fallback;
    else return it->is_number() ? it->template get<T>() : fallback;
}

std::string field(const json &object, const char *key, const char *fallback) {
    return field<std::string>(object, key, std::string(fallback));
}

// ------------------------------------------------------------------ live settings editing

// Shows values from engine-sim's last status, except while the user is changing them: then the
// local edit wins until engine-sim has had a moment to catch up.
class SettingsEditor {
public:
    SettingsEditor(ControlClient *client) : m_client(client) {}

    void update(const json &settings) { m_settings = settings; }

    bool checkbox(const char *label, const char *path) {
        bool value = current(path, false);
        if (!ImGui::Checkbox(label, &value)) return false;
        commit(path, value, true);
        return true;
    }

    bool sliderFloat(const char *label, const char *path, float min, float max, const char *format, float displayScale = 1.0f) {
        float value = current(path, 0.0f) * displayScale;
        const bool changed = ImGui::SliderFloat(label, &value, min, max, format);
        finishSlider(path, json(value / displayScale), changed);
        return changed;
    }

    bool sliderInt(const char *label, const char *path, int min, int max, const char *format) {
        int value = static_cast<int>(std::lround(current(path, 0.0)));
        const bool changed = ImGui::SliderInt(label, &value, min, max, format);
        finishSlider(path, json(value), changed);
        return changed;
    }

    template <typename T>
    T current(const char *path, T fallback) {
        const auto pending = m_pending.find(path);
        if (pending != m_pending.end()) {
            const bool fresh = clock_type::now() - pending->second.changed < std::chrono::milliseconds(1500);
            if (fresh || pending->second.active) return pending->second.value.get<T>();
            m_pending.erase(pending);
        }

        const json::json_pointer pointer(path);
        if (m_settings.is_object() && m_settings.contains(pointer)) return m_settings[pointer].get<T>();
        return fallback;
    }

    void commit(const char *path, const json &value, bool now) {
        Pending &pending = m_pending[path];
        pending.value = value;
        pending.changed = clock_type::now();

        if (!now && clock_type::now() - pending.sent < std::chrono::milliseconds(100)) {
            pending.unsent = true;
            return;
        }

        json patch;
        patch[json::json_pointer(path)] = value;
        m_client->setSettings(patch);
        pending.sent = clock_type::now();
        pending.unsent = false;
    }

private:
    struct Pending {
        json value;
        clock_type::time_point changed;
        clock_type::time_point sent;
        bool unsent = false;
        bool active = false;
    };

    void finishSlider(const char *path, const json &value, bool changed) {
        if (changed) commit(path, value, false);

        const auto it = m_pending.find(path);
        if (it == m_pending.end()) return;

        it->second.active = ImGui::IsItemActive();
        // Releasing the slider sends the final value even if the last one was throttled
        if (ImGui::IsItemDeactivatedAfterEdit() || (!it->second.active && it->second.unsent)) {
            commit(path, it->second.value, true);
        }
    }

    ControlClient *m_client;
    json m_settings;
    std::map<std::string, Pending> m_pending;
};

// A live engine-sound slider. Shows engine-sim's current value except while (or just after)
// it's being dragged; sends at most 10 updates a second, plus the final value on release.
void synthSlider(ControlClient &client, const json &synth, const char *label, const char *key, float scale, const char *format, ImGuiSliderFlags flags = 0) {
    struct Pending { float value; clock_type::time_point changed, sent; };
    static std::map<std::string, Pending> pending;

    float value = static_cast<float>(field(synth, key, 0.0)) * scale;
    const auto it = pending.find(key);
    if (it != pending.end()) {
        if (clock_type::now() - it->second.changed < std::chrono::milliseconds(1500)) value = it->second.value;
        else pending.erase(it);
    }

    const float min = (flags & ImGuiSliderFlags_Logarithmic) ? 0.0001f * scale : 0.0f;
    const bool changed = ImGui::SliderFloat(label, &value, min, scale, format, flags);
    const bool released = ImGui::IsItemDeactivatedAfterEdit();
    if (!changed && !released) return;

    Pending &p = pending[key];
    p.value = value;
    p.changed = clock_type::now();
    if (released || clock_type::now() - p.sent > std::chrono::milliseconds(100)) {
        client.send({ { "cmd", "synth" }, { key, value / scale } });
        p.sent = clock_type::now();
    }
}

void help(const char *text) {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

// ------------------------------------------------------------------ engine file picker

std::mutex g_pickedMutex;
std::optional<std::string> g_picked;

void SDLCALL onFilePicked(void *, const char *const *files, int) {
    if (files == nullptr || files[0] == nullptr) return; // Cancelled or failed
    std::lock_guard<std::mutex> lock(g_pickedMutex);
    g_picked = files[0];
}

void openEnginePicker(SDL_Window *window, const std::string &currentScript) {
    static const SDL_DialogFileFilter filters[] = { { "Engine scripts", "mr" } };

    std::string start;
    if (!currentScript.empty()) start = std::filesystem::path(currentScript).parent_path().string();

    SDL_ShowOpenFileDialog(onFilePicked, nullptr, window, filters, 1, start.empty() ? nullptr : start.c_str(), false);
}

// ------------------------------------------------------------------ the panel

void startEngine(EngineLauncher &launcher, EngineLauncher::Mode mode, std::string *error) {
    error->clear();
    launcher.start(mode, error);
}

void drawPanel(SDL_Window *window, ControlClient &client, SettingsEditor &editor, EngineLauncher &launcher, const std::string &socketPath) {
    static std::string launchError;
    const ControlClient::Snapshot snap = client.snapshot();
    const bool starting = launcher.running();

    if (!snap.connected) {
        if (starting) {
            ImGui::TextColored(Orange, "Starting engine-sim...");
            return;
        }

        ImGui::TextColored(Orange, "engine-sim isn't running");
        ImGui::Spacing();
        if (ImGui::Button("Start in the background")) startEngine(launcher, EngineLauncher::Mode::Headless, &launchError);
        help("engine-sim-headless: no window, just the simulation. Stops when you close this panel.");
        ImGui::SameLine();
        if (ImGui::Button("Start with its window")) startEngine(launcher, EngineLauncher::Mode::Window, &launchError);
        help("The full engine-sim window with the engine view and gauges. Stops when you close this panel.");

        ImGui::Spacing();
        ImGui::TextColored(Muted, "Or start engine-sim yourself; this panel connects to it automatically.");

        const std::string &failure = launchError.empty() ? launcher.exitMessage() : launchError;
        if (!failure.empty()) {
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Text, Red);
            ImGui::TextWrapped("%s", failure.c_str());
            ImGui::PopStyleColor();
            ImGui::TextColored(Muted, "Full log: %s", EngineLauncher::logPath().c_str());
        }
        return;
    }

    // Sliders leave room for their labels
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.55f);

    const json &status = snap.status;
    editor.update(field(status, "settings", json::object()));

    const json engine = field(status, "engine", json());
    const json bridge = field(status, "bridge", json::object());
    const json performance = field(status, "performance", json::object());
    const bool linked = field(bridge, "linked", false);

    // --- Status
    ImGui::TextColored(Green, "engine-sim connected");
    ImGui::SameLine(0, ImGui::GetFontSize() * 1.5f);
    if (!field(bridge, "listening", false)) ImGui::TextColored(Red, "BeamNG bridge off");
    else if (field(bridge, "tuning", false)) ImGui::TextColored(Orange, "Profiling engine for BeamNG");
    else if (linked) ImGui::TextColored(Green, "Linked to BeamNG");
    else ImGui::TextColored(Muted, "Waiting for BeamNG");

    const char *stopLabel = "Stop engine-sim";
    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize(stopLabel).x - ImGui::GetStyle().FramePadding.x * 2);
    if (ImGui::SmallButton(stopLabel)) requestShutdown(socketPath);

    ImGui::Separator();

    // --- Engine
    if (ImGui::CollapsingHeader("Engine", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (engine.is_object()) {
            ImGui::Text("%s", field(engine, "name", "").c_str());

            const double rpm = field(engine, "rpm", 0.0);
            const double redline = std::max(field(engine, "redline_rpm", 1.0), 1.0);
            char rpmText[32];
            std::snprintf(rpmText, sizeof(rpmText), "%.0f rpm", rpm);
            ImGui::ProgressBar(static_cast<float>(std::min(rpm / redline, 1.0)), ImVec2(-FLT_MIN, 0), rpmText);

            const double load = field(performance, "load", 0.0);
            ImGui::TextColored(load > 0.9 ? Red : Muted, "%d Hz simulation, %.0f%% of a CPU core",
                field(engine, "simulation_frequency", 0), load * 100);
            if (load > 0.9) help("The simulation can barely keep up, which makes the sound crackle. Lower the simulation rate below.");

            // Folder and file name; the full path on hover
            const std::filesystem::path script = field(engine, "script", "");
            const std::string shortName = (script.parent_path().filename() / script.filename()).string();
            ImGui::TextColored(Muted, "%s", shortName.c_str());
            ImGui::SetItemTooltip("%s", script.c_str());
        }
        else {
            ImGui::TextColored(Red, "No engine loaded");
        }

        ImGui::BeginDisabled(snap.busy);
        if (ImGui::Button("Load engine...")) openEnginePicker(window, engine.is_object() ? field(engine, "script", "") : "");
        ImGui::SameLine();
        if (ImGui::Button("Default engine")) client.send({ { "cmd", "load" }, { "script", "" } });
        ImGui::EndDisabled();
        if (snap.busy) {
            ImGui::SameLine();
            ImGui::TextColored(Muted, "Loading...");
        }

        if (!snap.message.empty()) {
            ImGui::PushStyleColor(ImGuiCol_Text, snap.messageIsError ? Red : Muted);
            ImGui::TextWrapped("%s", snap.message.c_str());
            ImGui::PopStyleColor();
        }
    }

    // --- BeamNG
    if (ImGui::CollapsingHeader("BeamNG", ImGuiTreeNodeFlags_DefaultOpen)) {
        editor.checkbox("Follow BeamNG", "/bridge/connect");
        help("When off, the car keeps its stock engine and sound, and engine-sim plays on its own.");

        editor.sliderFloat("Volume", "/bridge/volume", 0, 200, "%.0f%%", 100.0f);

        editor.checkbox("Flat EQ", "/bridge/neutral_eq");
        help("The car's sound EQ is tuned for its own recordings. Flattening it lets engine-sim through unchanged.");
        ImGui::SameLine();
        editor.checkbox("Game's low cut", "/bridge/game_low_cut");
        help("BeamNG filters out everything below ~90-190 Hz on engine sound, which is where engine-sim's body is. Off lets the low end through (needs the updated plugin).");

        editor.checkbox("Intake", "/bridge/intake_sound");
        ImGui::SameLine();
        editor.checkbox("Exhaust", "/bridge/exhaust_sound");
        help("Which of the car's sound sources play engine-sim.");

        ImGui::SeparatorText("Performance");
        editor.checkbox("engine-sim drives the car", "/bridge/drive_car");
        help("Off: engine-sim only makes the sound and the car keeps its own engine. On: the car gets engine-sim's torque, so it accelerates like the loaded engine (fun, but an F1 engine in a truck bogs and stalls).");
        ImGui::BeginDisabled(!editor.current("/bridge/drive_car", false));
        int idleRpm = static_cast<int>(std::lround(editor.current("/bridge/beam_idle_rpm", 1100.0)));
        if (ImGui::SliderInt("Stock torque below", &idleRpm, 0, 3000, idleRpm == 0 ? "off" : "%d rpm")) {
            editor.commit("/bridge/beam_idle_rpm", idleRpm, false);
        }
        if (ImGui::IsItemDeactivatedAfterEdit()) editor.commit("/bridge/beam_idle_rpm", idleRpm, true);
        help("Below this rpm, or with the throttle closed, the car uses its own torque. Keeps idle steady.");
        ImGui::EndDisabled();

        editor.checkbox("Tune car to this engine", "/bridge/tune_beam_engine");
        help("Measures the engine (a short rev sweep while the car is quiet) and reshapes the car's own engine (torque curve, idle, redline, inertia) to match. This changes how the car drives too, even with the option above off.");
        ImGui::SameLine();
        ImGui::BeginDisabled(!editor.current("/bridge/tune_beam_engine", false));
        if (ImGui::Button("Retune")) client.send({ { "cmd", "retune" } });
        ImGui::EndDisabled();
    }

    // --- What BeamNG says about the player's car
    const json vehicle = field(status, "beamng_vehicle", json());
    if (vehicle.is_object() && ImGui::CollapsingHeader("BeamNG car", ImGuiTreeNodeFlags_DefaultOpen)) {
        const json car = field(vehicle, "vehicle", json::object());
        ImGui::Text("%s", field(car, "model", field(car, "directory", std::string("Unknown car"))).c_str());
        const std::string configuration = field(car, "configuration", "");
        if (!configuration.empty()) {
            ImGui::SameLine();
            ImGui::TextColored(Muted, "%s", configuration.c_str());
        }

        const json carEngine = field(vehicle, "engine", json());
        if (!carEngine.is_object()) {
            ImGui::TextColored(Muted, "No combustion engine (electric?)");
        }
        else {
            ImGui::Text("%s", field(carEngine, "name", field(carEngine, "part", std::string("Unknown engine"))).c_str());
            ImGui::SetItemTooltip("Part: %s", field(carEngine, "part", std::string("?")).c_str());

            // Peak torque from the jbeam table
            double peakTorque = 0, peakRpm = 0, maxRpmInTable = 0;
            std::vector<float> curve;
            const json torque = field(carEngine, "torque", json::array());
            for (const json &row : torque) {
                if (!row.is_array() || row.size() < 2 || !row[0].is_number() || !row[1].is_number()) continue;
                const double rpm = row[0].get<double>(), nm = row[1].get<double>();
                if (nm > peakTorque) { peakTorque = nm; peakRpm = rpm; }
                maxRpmInTable = std::max(maxRpmInTable, rpm);
                curve.push_back(static_cast<float>(nm));
            }

            const int cylinders = field(carEngine, "cylinders", 0);
            const std::string fuel = field(carEngine, "fuel", std::string("?"));
            const double idle = field(carEngine, "idle_rpm", 0.0);
            const double limit = field(carEngine, "rev_limiter_rpm", field(carEngine, "max_rpm", 0.0));
            ImGui::TextColored(Muted, "%s%d cylinders, %s, idle %.0f, limit %.0f rpm",
                cylinders > 0 ? "" : "? ", cylinders, fuel.c_str(), idle, limit);
            if (peakTorque > 0) ImGui::TextColored(Muted, "%.0f Nm at %.0f rpm (before boost)", peakTorque, peakRpm);

            if (vehicle.contains("turbo")) {
                const json turbo = vehicle["turbo"];
                ImGui::TextColored(Muted, "Turbo: %s, %.0f psi", field(turbo, "name", std::string("fitted")).c_str(), field(turbo, "wastegate_psi", 0.0));
            }
            if (vehicle.contains("supercharger")) {
                const json sc = vehicle["supercharger"];
                ImGui::TextColored(Muted, "Supercharger: %s (%s)", field(sc, "name", std::string("fitted")).c_str(), field(sc, "type", std::string("?")).c_str());
            }

            if (curve.size() >= 2) {
                char overlay[64];
                std::snprintf(overlay, sizeof(overlay), "torque, 0-%.0f rpm", maxRpmInTable);
                ImGui::PlotLines("##torque", curve.data(), static_cast<int>(curve.size()), 0, overlay, 0.0f, FLT_MAX,
                    ImVec2(-FLT_MIN, ImGui::GetFontSize() * 4));
            }
        }
    }

    // --- Sound
    if (ImGui::CollapsingHeader("Sound", ImGuiTreeNodeFlags_DefaultOpen)) {
        editor.sliderFloat("Headroom", "/audio/headroom_db", 0, 12, "%.1f dB");
        help("Lowers engine-sim's level so its peaks don't clip (harsh, buzzy distortion). 0 dB is upstream engine-sim, which clips; 6 dB is clean.");
    }

    // --- The synthesizer's mix, as on the engine-sim window's Z/X/C/V/B + scroll
    const json synth = field(status, "synth", json());
    if (synth.is_object() && ImGui::CollapsingHeader("Engine sound")) {
        synthSlider(client, synth, "Volume##synth", "volume", 100, "%.0f%%");
        help("engine-sim's own output level (Z). The BeamNG volume above is applied on top.");
        synthSlider(client, synth, "Convolution", "convolution", 100, "%.0f%%");
        help("How much of the exhaust's impulse response (its echo and resonance) is mixed in (X).");
        synthSlider(client, synth, "High freq. gain", "high_freq_gain", 1, "%.4f", ImGuiSliderFlags_Logarithmic);
        help("Adds the sharp, high-frequency part of the pressure pulses (C).");
        synthSlider(client, synth, "Low freq. noise", "low_freq_noise", 100, "%.0f%%");
        help("Rumbly air noise mixed into the exhaust (V).");
        synthSlider(client, synth, "High freq. noise", "high_freq_noise", 100, "%.0f%%");
        help("Random jitter on the input, a rough raspy texture (B).");

        if (ImGui::Button("Reset to this engine's sound")) client.send({ { "cmd", "synth" }, { "reset", true } });
        ImGui::TextColored(Muted, "Per engine; loading an engine starts from its own values.");
    }

    // --- Simulation
    if (ImGui::CollapsingHeader("Simulation")) {
        double frequency = editor.current("/simulation_frequency", 0.0);
        bool ownRate = frequency == 0;
        if (ImGui::Checkbox("Use the engine's own rate", &ownRate)) {
            editor.commit("/simulation_frequency", ownRate ? 0 : field(engine, "simulation_frequency", 10000), true);
        }
        if (!ownRate) editor.sliderInt("Rate", "/simulation_frequency", 2000, 40000, "%d Hz");
        help("Higher sounds cleaner but costs more CPU. If the load above gets near 100%, go lower.");
    }

    // --- Manual control, for headless without BeamNG
    if (ImGui::CollapsingHeader("Drive it yourself")) {
        if (linked) {
            ImGui::TextColored(Muted, "BeamNG is driving the engine.");
        }
        else if (engine.is_object()) {
            bool ignition = field(engine, "ignition", false);
            if (ImGui::Checkbox("Ignition", &ignition)) client.send({ { "cmd", "control" }, { "ignition", ignition } });

            ImGui::SameLine();
            ImGui::Button("Hold to crank");
            if (ImGui::IsItemActivated()) client.send({ { "cmd", "control" }, { "starter", true } });
            if (ImGui::IsItemDeactivated()) client.send({ { "cmd", "control" }, { "starter", false } });

            static float throttle = 0;
            if (!ImGui::IsAnyItemActive()) throttle = static_cast<float>(field(engine, "throttle", 0.0)) * 100;
            if (ImGui::SliderFloat("Throttle", &throttle, 0, 100, "%.0f%%")) {
                client.send({ { "cmd", "control" }, { "throttle", throttle / 100.0 } });
            }
            ImGui::TextColored(Muted, "In the engine-sim window its own keys take over again.");
        }
    }

    ImGui::PopItemWidth();
}

} // namespace

int main(int argc, char **argv) {
    // --start headless|window: start engine-sim unless it's already running
    std::optional<EngineLauncher::Mode> startMode;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--start" && i + 1 < argc) {
            const std::string_view mode = argv[++i];
            if (mode == "headless") startMode = EngineLauncher::Mode::Headless;
            else if (mode == "window" || mode == "gui") startMode = EngineLauncher::Mode::Window;
            else { std::fprintf(stderr, "--start takes headless or window\n"); return 2; }
        }
        else {
            std::printf("Usage: %s [--start headless|window]\n\n"
                        "Settings window for a running engine-sim. With --start, also starts engine-sim\n"
                        "(unless it's already running) and stops it again when the window closes.\n", argv[0]);
            return arg == "--help" || arg == "-h" ? 0 : 2;
        }
    }

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "engine-sim-control: SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    const float scale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());
    SDL_Window *window = SDL_CreateWindow("engine-sim control",
        static_cast<int>(520 * scale), static_cast<int>(660 * scale),
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (window == nullptr) {
        std::fprintf(stderr, "engine-sim-control: SDL_CreateWindow failed: %s\n", SDL_GetError());
        return 1;
    }

    // SDL picks Vulkan on Linux
    SDL_GPUDevice *gpu = SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_SPIRV, false, nullptr);
    if (gpu == nullptr || !SDL_ClaimWindowForGPUDevice(gpu, window)) {
        std::fprintf(stderr, "engine-sim-control: no GPU device: %s\n", SDL_GetError());
        return 1;
    }
    SDL_SetGPUSwapchainParameters(gpu, window, SDL_GPU_SWAPCHAINCOMPOSITION_SDR, SDL_GPU_PRESENTMODE_VSYNC);
    SDL_ShowWindow(window);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr; // Nothing worth persisting about one fixed window

    ImGui::StyleColorsDark();
    ImGuiStyle &style = ImGui::GetStyle();
    style.ScaleAllSizes(scale);
    style.FontScaleDpi = scale;
    style.FontSizeBase = 16.0f;

    ImGui_ImplSDL3_InitForSDLGPU(window);
    ImGui_ImplSDLGPU3_InitInfo init{};
    init.Device = gpu;
    init.ColorTargetFormat = SDL_GetGPUSwapchainTextureFormat(gpu, window);
    init.MSAASamples = SDL_GPU_SAMPLECOUNT_1;
    ImGui_ImplSDLGPU3_Init(&init);

    const std::string socketPath = ControlClient::defaultSocketPath();
    ControlClient client(socketPath);
    SettingsEditor editor(&client);

    EngineLauncher launcher;
    if (startMode && !std::filesystem::exists(socketPath)) {
        std::string error;
        if (!launcher.start(*startMode, &error)) std::fprintf(stderr, "engine-sim-control: %s\n", error.c_str());
    }

    bool done = false;
    while (!done) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT || event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) done = true;
        }
        if (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) {
            SDL_Delay(50);
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(g_pickedMutex);
            if (g_picked) {
                client.send({ { "cmd", "load" }, { "script", *g_picked } });
                g_picked.reset();
            }
        }

        ImGui_ImplSDLGPU3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::Begin("engine-sim control", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
        try {
            drawPanel(window, client, editor, launcher, socketPath);
        }
        catch (const std::exception &e) {
            // Unexpected data shouldn't take the panel down; show it and carry on
            ImGui::TextColored(Red, "Couldn't show this: %s", e.what());
        }
        ImGui::End();

        ImGui::Render();
        ImDrawData *drawData = ImGui::GetDrawData();

        SDL_GPUCommandBuffer *commands = SDL_AcquireGPUCommandBuffer(gpu);
        SDL_GPUTexture *swapchain = nullptr;
        SDL_WaitAndAcquireGPUSwapchainTexture(commands, window, &swapchain, nullptr, nullptr);
        if (swapchain != nullptr && drawData->DisplaySize.x > 0 && drawData->DisplaySize.y > 0) {
            ImGui_ImplSDLGPU3_PrepareDrawData(drawData, commands);

            SDL_GPUColorTargetInfo target{};
            target.texture = swapchain;
            target.clear_color = SDL_FColor{ 0.09f, 0.09f, 0.10f, 1.0f };
            target.load_op = SDL_GPU_LOADOP_CLEAR;
            target.store_op = SDL_GPU_STOREOP_STORE;
            SDL_GPURenderPass *pass = SDL_BeginGPURenderPass(commands, &target, 1, nullptr);
            ImGui_ImplSDLGPU3_RenderDrawData(drawData, commands, pass);
            SDL_EndGPURenderPass(pass);
        }
        SDL_SubmitGPUCommandBuffer(commands);
    }

    // Whatever this panel started, it stops
    SDL_HideWindow(window);
    launcher.stop(socketPath);

    SDL_WaitForGPUIdle(gpu);
    ImGui_ImplSDL3_Shutdown();
    ImGui_ImplSDLGPU3_Shutdown();
    ImGui::DestroyContext();
    SDL_ReleaseWindowFromGPUDevice(gpu, window);
    SDL_DestroyGPUDevice(gpu);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
