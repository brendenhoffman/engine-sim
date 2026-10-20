// engine-sim-control: a small window for changing engine-sim's settings while it runs (GUI or
// headless). Talks to it over the control socket; everything applies live.

#include "control_client.h"

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlgpu3.h>
#include <SDL3/SDL.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <mutex>
#include <optional>
#include <string>

using nlohmann::json;
using clock_type = std::chrono::steady_clock;

namespace {

const ImVec4 Green(0.29f, 0.87f, 0.50f, 1.0f);
const ImVec4 Orange(1.00f, 0.67f, 0.24f, 1.0f);
const ImVec4 Red(1.00f, 0.39f, 0.35f, 1.0f);
const ImVec4 Muted(0.60f, 0.60f, 0.65f, 1.0f);

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

void drawPanel(SDL_Window *window, ControlClient &client, SettingsEditor &editor) {
    const ControlClient::Snapshot snap = client.snapshot();

    if (!snap.connected) {
        ImGui::TextColored(Orange, "Waiting for engine-sim...");
        ImGui::Spacing();
        ImGui::TextWrapped("Start the engine-sim GUI or engine-sim-headless. This panel connects to it automatically.");
        return;
    }

    // Sliders leave room for their labels
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.55f);

    const json &status = snap.status;
    editor.update(status.value("settings", json::object()));

    const json engine = status.value("engine", json());
    const json bridge = status.value("bridge", json::object());
    const json performance = status.value("performance", json::object());
    const bool linked = bridge.value("linked", false);

    // --- Status
    ImGui::TextColored(Green, "engine-sim connected");
    ImGui::SameLine(0, ImGui::GetFontSize() * 1.5f);
    if (!bridge.value("listening", false)) ImGui::TextColored(Red, "BeamNG bridge off");
    else if (bridge.value("tuning", false)) ImGui::TextColored(Orange, "Profiling engine for BeamNG");
    else if (linked) ImGui::TextColored(Green, "Linked to BeamNG");
    else ImGui::TextColored(Muted, "Waiting for BeamNG");

    ImGui::Separator();

    // --- Engine
    if (ImGui::CollapsingHeader("Engine", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (engine.is_object()) {
            ImGui::Text("%s", engine.value("name", "").c_str());

            const double rpm = engine.value("rpm", 0.0);
            const double redline = std::max(engine.value("redline_rpm", 1.0), 1.0);
            char rpmText[32];
            std::snprintf(rpmText, sizeof(rpmText), "%.0f rpm", rpm);
            ImGui::ProgressBar(static_cast<float>(std::min(rpm / redline, 1.0)), ImVec2(-FLT_MIN, 0), rpmText);

            const double load = performance.value("load", 0.0);
            ImGui::TextColored(load > 0.9 ? Red : Muted, "%d Hz simulation, %.0f%% of a CPU core",
                engine.value("simulation_frequency", 0), load * 100);
            if (load > 0.9) help("The simulation can barely keep up, which makes the sound crackle. Lower the simulation rate below.");

            // Folder and file name; the full path on hover
            const std::filesystem::path script = engine.value("script", "");
            const std::string shortName = (script.parent_path().filename() / script.filename()).string();
            ImGui::TextColored(Muted, "%s", shortName.c_str());
            ImGui::SetItemTooltip("%s", script.c_str());
        }
        else {
            ImGui::TextColored(Red, "No engine loaded");
        }

        ImGui::BeginDisabled(snap.busy);
        if (ImGui::Button("Load engine...")) openEnginePicker(window, engine.is_object() ? engine.value("script", "") : "");
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

        int idleRpm = static_cast<int>(std::lround(editor.current("/bridge/beam_idle_rpm", 1100.0)));
        if (ImGui::SliderInt("Stock torque below", &idleRpm, 0, 3000, idleRpm == 0 ? "off" : "%d rpm")) {
            editor.commit("/bridge/beam_idle_rpm", idleRpm, false);
        }
        if (ImGui::IsItemDeactivatedAfterEdit()) editor.commit("/bridge/beam_idle_rpm", idleRpm, true);
        help("Below this rpm, or with the throttle closed, the car uses its own torque. Keeps idle steady.");

        editor.checkbox("Flat EQ", "/bridge/neutral_eq");
        help("The car's sound EQ is tuned for its own recordings. Flattening it lets engine-sim through unchanged.");

        editor.checkbox("Intake", "/bridge/intake_sound");
        ImGui::SameLine();
        editor.checkbox("Exhaust", "/bridge/exhaust_sound");
        help("Which of the car's sound sources play engine-sim.");

        editor.checkbox("Tune car to this engine", "/bridge/tune_beam_engine");
        help("Measures the engine (a short rev sweep while the car is quiet) and reshapes the car's torque curve, idle, redline and inertia to match.");
        ImGui::SameLine();
        ImGui::BeginDisabled(!editor.current("/bridge/tune_beam_engine", false));
        if (ImGui::Button("Retune")) client.send({ { "cmd", "retune" } });
        ImGui::EndDisabled();
    }

    // --- Sound
    if (ImGui::CollapsingHeader("Sound", ImGuiTreeNodeFlags_DefaultOpen)) {
        editor.sliderFloat("Headroom", "/audio/headroom_db", 0, 12, "%.1f dB");
        help("Lowers engine-sim's level so its peaks don't clip (harsh, buzzy distortion). 0 dB is upstream engine-sim, which clips; 6 dB is clean.");
    }

    // --- Simulation
    if (ImGui::CollapsingHeader("Simulation")) {
        double frequency = editor.current("/simulation_frequency", 0.0);
        bool ownRate = frequency == 0;
        if (ImGui::Checkbox("Use the engine's own rate", &ownRate)) {
            editor.commit("/simulation_frequency", ownRate ? 0 : engine.value("simulation_frequency", 10000), true);
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
            bool ignition = engine.value("ignition", false);
            if (ImGui::Checkbox("Ignition", &ignition)) client.send({ { "cmd", "control" }, { "ignition", ignition } });

            ImGui::SameLine();
            ImGui::Button("Hold to crank");
            if (ImGui::IsItemActivated()) client.send({ { "cmd", "control" }, { "starter", true } });
            if (ImGui::IsItemDeactivated()) client.send({ { "cmd", "control" }, { "starter", false } });

            static float throttle = 0;
            if (!ImGui::IsAnyItemActive()) throttle = static_cast<float>(engine.value("throttle", 0.0)) * 100;
            if (ImGui::SliderFloat("Throttle", &throttle, 0, 100, "%.0f%%")) {
                client.send({ { "cmd", "control" }, { "throttle", throttle / 100.0 } });
            }
            ImGui::TextColored(Muted, "In the engine-sim window its own keys take over again.");
        }
    }

    ImGui::PopItemWidth();
}

} // namespace

int main(int, char **) {
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

    ControlClient client(ControlClient::defaultSocketPath());
    SettingsEditor editor(&client);

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
        drawPanel(window, client, editor);
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
