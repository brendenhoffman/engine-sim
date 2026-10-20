// Replaces BeamNG's BinLinux/libBeamNGBlendLayerDSP.so, which BeamNG loads as an FMOD plugin at
// startup. The stock plugin is a pass-through DSP called "BeamNGBlendLayer"; this exports the
// same description so the game sees no difference.
//
// On top of that it hooks FMOD's createSound, playSound and setChannelGroup inside the game.
// When one of the mod's silent marker samples plays into an engine or exhaust event, a DSP that
// outputs engine-sim's audio (UDP 4571, "BESF" + float32 mono 44.1 kHz) goes at the head of
// that event's channel group, so the sound is positioned and mixed like the stock engine sound.
//
// Ported from BeamEngineSim (Windows). On Linux the FMOD functions are found by name with
// dlsym instead of by scanning the executable.

#include "fmod_abi.h"
#include "inline_hook.h"

#include <arpa/inet.h>
#include <dlfcn.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define EXPORT extern "C" __attribute__((visibility("default")))

namespace {

// ---------------------------------------------------------------- logging

FILE *s_log = nullptr;
std::mutex s_logMutex;

void logf(const char *fmt, ...) {
    std::lock_guard<std::mutex> lock(s_logMutex);
    if (s_log == nullptr) return;

    // Seconds since the plugin loaded, to line up with BeamNG's own log
    static const auto start = std::chrono::steady_clock::now();
    std::fprintf(s_log, "%8.3f ", std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());

    va_list args;
    va_start(args, fmt);
    std::vfprintf(s_log, fmt, args);
    va_end(args);
    std::fflush(s_log);
}

// ~/.local/state/engine-sim/beamng-plugin.log (or under $XDG_STATE_HOME)
void openLog() {
    namespace fs = std::filesystem;

    const char *state = std::getenv("XDG_STATE_HOME");
    const char *home = std::getenv("HOME");
    fs::path dir;
    if (state != nullptr && state[0] != '\0') dir = fs::path(state) / "engine-sim";
    else if (home != nullptr) dir = fs::path(home) / ".local/state/engine-sim";
    else return;

    std::error_code ec;
    fs::create_directories(dir, ec);
    s_log = std::fopen((dir / "beamng-plugin.log").c_str(), "w");
}

unsigned long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ---------------------------------------------------------------- audio from engine-sim

constexpr int DefaultAudioPort = 4571;

// ENGINE_SIM_AUDIO_PORT overrides the port (tests use it to stay clear of a running game)
int audioPort() {
    const char *value = std::getenv("ENGINE_SIM_AUDIO_PORT");
    const int port = value != nullptr ? std::atoi(value) : 0;
    return (port > 0 && port < 65536) ? port : DefaultAudioPort;
}

constexpr int RingSize = 1 << 16;
float s_ring[RingSize];
std::atomic<long long> s_written{0};
std::atomic<unsigned long long> s_audioTime{0};

// From engine-sim's "BESX" packets: keep the game's low-cut (high-pass) filter on engine sound
std::atomic<bool> s_keepLowCut{false};

void receiverThread() {
    const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    const int port = audioPort();
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (fd < 0 || bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        logf("bind %d failed: %s\n", port, std::strerror(errno));
        if (fd >= 0) close(fd);
        return;
    }

    char buffer[8192];
    for (;;) {
        const ssize_t n = recv(fd, buffer, sizeof(buffer), 0);
        if (n >= 12 && std::memcmp(buffer, "BESX", 4) == 0) {
            float values[2]; // version, keep low cut
            std::memcpy(values, buffer + 4, sizeof(values));
            s_keepLowCut = values[1] != 0;
            continue;
        }
        if (n < 4 || std::memcmp(buffer, "BESF", 4) != 0) continue;

        const int count = static_cast<int>((n - 4) / 4);
        const long long w = s_written.load();
        for (int i = 0; i < count; ++i) {
            std::memcpy(&s_ring[(w + i) & (RingSize - 1)], buffer + 4 + 4 * i, 4);
        }
        s_written.store(w + count);
        s_audioTime.store(nowMs());
    }
}

// ---------------------------------------------------------------- resampling

// Kaiser-windowed sinc, 44.1 kHz to the mixer rate
constexpr int SincTaps = 32, SincPhases = 1024;
float s_sinc[SincPhases + 1][SincTaps];

double besselI0(double x) {
    double sum = 1, term = 1;
    for (int k = 1; k < 32; ++k) {
        term *= (x / (2 * k)) * (x / (2 * k));
        sum += term;
    }
    return sum;
}

void buildSinc() {
    const double pi = 3.14159265358979323846, cutoff = 0.96, beta = 9.0;
    for (int p = 0; p <= SincPhases; ++p) {
        double sum = 0;
        for (int k = 0; k < SincTaps; ++k) {
            const double x = (k - SincTaps / 2 + 1) - static_cast<double>(p) / SincPhases;
            const double r = x / (SincTaps / 2);
            const double window = (std::fabs(r) <= 1) ? besselI0(beta * std::sqrt(1 - r * r)) / besselI0(beta) : 0;
            const double sinc = (x == 0) ? 1 : std::sin(pi * cutoff * x) / (pi * cutoff * x);
            s_sinc[p][k] = static_cast<float>(cutoff * sinc * window);
            sum += s_sinc[p][k];
        }
        for (int k = 0; k < SincTaps; ++k) s_sinc[p][k] = static_cast<float>(s_sinc[p][k] / sum);
    }
}

float sincSample(double position) {
    const long long i0 = static_cast<long long>(position);
    const double phase = (position - i0) * SincPhases;
    const int p = static_cast<int>(phase);
    const float t = static_cast<float>(phase - p);
    const float *a = s_sinc[p], *b = s_sinc[p + 1];

    float out = 0;
    for (int k = 0; k < SincTaps; ++k) {
        out += s_ring[(i0 - SincTaps / 2 + 1 + k) & (RingSize - 1)] * (a[k] + (b[k] - a[k]) * t);
    }
    return out;
}

// ---------------------------------------------------------------- the engine-sim DSP

struct Voice {
    double readPos;
    double lagAvg;
    int generation;
};

std::atomic<int> s_resyncs{0};
std::atomic<unsigned long long> s_lastStats{0};
// Each marker re-init is a new generation. Only the newest plays engine-sim; older DSPs pass
// their input through, since BeamNG may keep or recycle those groups.
std::atomic<int> s_generation{0};
std::atomic<int> s_oldVoiceCalls{0};

int esCreate(FmodDspState *state) {
    state->plugindata = new Voice{ -1, 0, s_generation.load() };
    return FMOD_OK;
}

int esRelease(FmodDspState *state) {
    delete static_cast<Voice *>(state->plugindata);
    return FMOD_OK;
}

int esProcess(FmodDspState *state, unsigned int length, const FmodDspBufferArray *in, FmodDspBufferArray *out, int inputsIdle, int op) {
    Voice *v = static_cast<Voice *>(state->plugindata);
    if (op != FMOD_DSP_PROCESS_PERFORM) {
        out->buffernumchannels[0] = in->buffernumchannels[0];
        out->bufferchannelmask[0] = in->bufferchannelmask[0];
        out->speakermode = in->speakermode;
        return FMOD_OK;
    }

    const int channels = out->buffernumchannels[0];
    float *dst = out->buffers[0];

    // Stale DSP, or engine-sim has gone quiet: pass the game's own sound through
    if (v->generation != s_generation.load()) s_oldVoiceCalls++;
    if (v->generation != s_generation.load() || nowMs() - s_audioTime.load() > 300) {
        if (inputsIdle) std::memset(dst, 0, sizeof(float) * length * channels);
        else std::memcpy(dst, in->buffers[0], sizeof(float) * length * channels);
        v->readPos = -1;
        return FMOD_OK;
    }

    int rate = 48000;
    if (state->functions && state->functions->getsamplerate) state->functions->getsamplerate(state, &rate);

    // Trail the writer by ~50 ms, steering the rate by at most 0.2% on a ~1 s lag average
    const double target = 2048;
    const long long written = s_written.load();
    const double lag = written - v->readPos;
    if (v->readPos < 0 || lag < length * 0.95 + SincTaps || lag > 16384) {
        if (v->readPos >= 0) s_resyncs++;
        v->readPos = written - target;
        v->lagAvg = target;
    }
    v->lagAvg += 0.02 * ((written - v->readPos) - v->lagAvg);
    const double step = 44100.0 / rate * (1.0 + 0.002 * std::fmax(-1.0, std::fmin(1.0, (v->lagAvg - target) / target)));

    float peak = 0;
    for (unsigned int i = 0; i < length; ++i) {
        const float sample = sincSample(v->readPos);
        for (int c = 0; c < channels; ++c) dst[i * channels + c] = sample;
        v->readPos += step;
        peak = std::fmax(peak, std::fabs(sample));
    }

    const unsigned long long now = nowMs();
    if (now - s_lastStats > 10000) {
        s_lastStats = now;
        logf("stats: rate %d ch %d len %u peak %.2f resyncs %d lag %.0f gen %d oldVoiceCalls %d\n",
            rate, channels, length, peak, s_resyncs.load(), v->lagAvg, s_generation.load(), s_oldVoiceCalls.exchange(0));
    }
    return FMOD_OK;
}

FmodDspDescription s_stubDescription;
FmodDspDescription s_esDescription;

// ---------------------------------------------------------------- FMOD hooks

CreateSoundFn s_createSound;
PlaySoundFn s_playSound;
SetChannelGroupFn s_setChannelGroup;
CreateDspFn s_createDsp;
AddDspFn s_addDsp;

GetNumDspsFn s_getNumDsps;
GetDspFn s_getDsp;
GetParentGroupFn s_getParentGroup;
GetGroupNameFn s_getGroupName;
DspGetInfoFn s_dspGetInfo;
DspGetChannelFormatFn s_dspGetChannelFormat;
DspGetBypassFn s_dspGetBypass;
DspGetNumParametersFn s_dspGetNumParameters;
DspGetParameterInfoFn s_dspGetParameterInfo;
DspGetParameterFloatFn s_dspGetParameterFloat;
DspGetParameterIntFn s_dspGetParameterInt;
DspGetParameterBoolFn s_dspGetParameterBool;

DspSetBypassFn s_dspSetBypass;
ChannelGetChannelGroupFn s_channelGetChannelGroup;

// Marker channels of the current generation. FMOD validates channel handles (a stale one just
// returns an error) but not group pointers, so event buses are always found through these.
std::vector<void *> s_markerPlayChannels;
bool s_lowCutDirty = false;   // New channels since the low cut was last applied
bool s_lowCutBypassed = false;

unsigned long long s_lastParameterDump = 0;
int s_parameterDumps = 0;

void logParameters(void *dsp) {
    if (!s_dspGetNumParameters || !s_dspGetParameterInfo) return;

    int count = 0;
    s_dspGetNumParameters(dsp, &count);
    std::string line;
    for (int i = 0; i < count && i < 24; ++i) {
        FmodDspParameterDesc *desc = nullptr;
        if (s_dspGetParameterInfo(dsp, i, &desc) != FMOD_OK || !desc) continue;

        char value[32] = "?";
        if (desc->type == 0 && s_dspGetParameterFloat) { float v; s_dspGetParameterFloat(dsp, i, &v, value, sizeof(value)); }
        else if (desc->type == 1 && s_dspGetParameterInt) { int v; s_dspGetParameterInt(dsp, i, &v, value, sizeof(value)); }
        else if (desc->type == 2 && s_dspGetParameterBool) { bool v; s_dspGetParameterBool(dsp, i, &v, value, sizeof(value)); }
        else continue;

        char name[17] = {}, label[17] = {};
        std::memcpy(name, desc->name, 16);
        std::memcpy(label, desc->label, 16);
        line += std::string(" ") + name + "=" + value + label;
    }
    logf("      params:%s\n", line.c_str());
}

// The sound event's own bus: the first "Master Bus" above a marker's group
void *findEventBus(void *group) {
    void *current = group;
    for (int depth = 0; current != nullptr && depth < 12; ++depth) {
        char name[64] = {};
        if (s_getGroupName && s_getGroupName(current, name, sizeof(name)) == FMOD_OK && std::strcmp(name, "Master Bus") == 0) {
            return current;
        }
        void *parent = nullptr;
        if (!s_getParentGroup || s_getParentGroup(current, &parent) != FMOD_OK) return nullptr;
        current = parent;
    }
    return nullptr;
}

// Event buses of the live marker channels (stale channels are skipped). Needs s_mutex.
std::vector<void *> liveEventBuses() {
    std::vector<void *> buses;
    if (!s_channelGetChannelGroup) return buses;

    for (void *channel : s_markerPlayChannels) {
        void *group = nullptr;
        if (s_channelGetChannelGroup(channel, &group) != FMOD_OK || !group) continue;
        void *bus = findEventBus(group);
        if (bus && std::find(buses.begin(), buses.end(), bus) == buses.end()) buses.push_back(bus);
    }
    return buses;
}

std::string dspName(void *dsp) {
    char name[32] = {};
    unsigned int version = 0;
    int channels = 0, width = 0, height = 0;
    s_dspGetInfo(dsp, name, &version, &channels, &width, &height);
    return name;
}

// The game's sound events high-pass engine sound at ~90-190 Hz, which takes the body out of
// engine-sim's output. While engine-sim is playing, bypass it (unless asked to keep it); put
// it back when engine-sim stops so stock sound is untouched. Game thread only. Needs s_mutex.
void applyLowCut() {
    if (!s_dspSetBypass || !s_getNumDsps || !s_getDsp || !s_dspGetInfo) return;

    const bool engineSimPlaying = nowMs() - s_audioTime.load() < 1000;
    const bool bypass = engineSimPlaying && !s_keepLowCut.load();
    if (bypass == s_lowCutBypassed && !s_lowCutDirty) return;

    int changed = 0;
    for (void *bus : liveEventBuses()) {
        int count = 0;
        s_getNumDsps(bus, &count);
        for (int i = 0; i < count; ++i) {
            void *dsp = nullptr;
            if (s_getDsp(bus, i, &dsp) != FMOD_OK || !dsp) continue;
            if (dspName(dsp) == "FMOD Highpass Simple" && s_dspSetBypass(dsp, bypass) == FMOD_OK) ++changed;
        }
    }

    if (bypass != s_lowCutBypassed || changed > 0) {
        logf("game low cut %s on %d filter(s)\n", bypass ? "bypassed" : "restored", changed);
    }
    s_lowCutBypassed = bypass;
    s_lowCutDirty = false;
}

// Logs the filters and EQs of the event buses every 5 s, a limited number of times.
// Game thread only (FMOD must not be queried from inside a DSP callback). Needs s_mutex.
void maybeDumpEventBuses() {
    const unsigned long long now = nowMs();
    if (s_parameterDumps >= 24 || now - s_lastParameterDump < 5000) return;

    const std::vector<void *> buses = liveEventBuses();
    if (buses.empty()) return;
    s_lastParameterDump = now;
    ++s_parameterDumps;

    for (void *bus : buses) {
        int count = 0;
        s_getNumDsps(bus, &count);
        logf("event bus %p filters:\n", bus);
        for (int i = 0; i < count; ++i) {
            void *dsp = nullptr;
            if (s_getDsp(bus, i, &dsp) != FMOD_OK || !dsp) continue;

            const std::string name = dspName(dsp);
            if (name.find("pass") == std::string::npos && name.find("Equalizer") == std::string::npos) continue;

            bool bypass = false;
            if (s_dspGetBypass) s_dspGetBypass(dsp, &bypass);
            logf("    [%d] %s%s\n", i, name.c_str(), bypass ? " (bypassed)" : "");
            logParameters(dsp);
        }
    }
}

// Logs every DSP of the group and its parents, in index order (index 0 is the head, the last
// to process before the group's output). Shows what runs after our DSP.
void logChain(void *group) {
    if (!s_getNumDsps || !s_getDsp || !s_dspGetInfo) return;

    void *current = group;
    for (int depth = 0; current != nullptr && depth < 12; ++depth) {
        char groupName[64] = "?";
        if (s_getGroupName) s_getGroupName(current, groupName, sizeof(groupName));

        int count = 0;
        s_getNumDsps(current, &count);
        logf("  group %p '%s' (%d DSPs)%s\n", current, groupName, count, depth == 0 ? " <- ours at the head" : "");

        for (int i = 0; i < count; ++i) {
            void *dsp = nullptr;
            if (s_getDsp(current, i, &dsp) != FMOD_OK || !dsp) continue;

            char name[32] = {};
            unsigned int version = 0;
            int channels = 0, width = 0, height = 0;
            s_dspGetInfo(dsp, name, &version, &channels, &width, &height);

            unsigned int mask = 0;
            int formatChannels = 0, speakerMode = 0;
            if (s_dspGetChannelFormat) s_dspGetChannelFormat(dsp, &mask, &formatChannels, &speakerMode);

            bool bypass = false;
            if (s_dspGetBypass) s_dspGetBypass(dsp, &bypass);

            logf("    [%d] %-24s channels %d format %d/%d%s\n", i, name, channels, formatChannels, speakerMode, bypass ? " (bypassed)" : "");
        }

        void *parent = nullptr;
        if (!s_getParentGroup || s_getParentGroup(current, &parent) != FMOD_OK) break;
        current = parent;
    }
}

std::mutex s_mutex;
// Entries are never removed; a freed group FMOD reuses keeps our old DSP, which generations
// turn into a pass-through. Hook ChannelGroup::release if leaks ever matter.
std::unordered_set<void *> s_markerSounds, s_markerChannels;
std::unordered_map<void *, int> s_bridgedGroups;
unsigned long long s_lastBridgeTick = 0;
void *s_system = nullptr;

bool isMarker(const char *name, unsigned int mode, void *exinfo) {
    static const char tag[] = "ENGINESIMMARKER";
    if (mode & (FMOD_OPENMEMORY | FMOD_OPENMEMORY_POINT)) {
        if (!exinfo || !name) return false;
        const unsigned int length = *reinterpret_cast<const unsigned int *>(static_cast<const char *>(exinfo) + 4); // FMOD_CREATESOUNDEXINFO.length
        const unsigned int scan = length < 4096 ? length : 4096;
        for (unsigned int i = 0; i + sizeof(tag) - 1 <= scan; ++i) {
            if (std::memcmp(name + i, tag, sizeof(tag) - 1) == 0) return true;
        }
        return false;
    }
    if ((mode & FMOD_OPENUSER) || !name) return false;
    return std::strstr(name, "engineSimMarker") != nullptr;
}

// Needs s_mutex
void bridgeGroup(void *group) {
    if (!group || !s_system) return;

    const auto it = s_bridgedGroups.find(group);
    if (it != s_bridgedGroups.end() && it->second == s_generation.load()) return;

    const unsigned long long now = nowMs();
    if (now - s_lastBridgeTick > 500) {
        // Markers were re-initialised: the previous channels belong to old sound events
        s_generation++;
        s_markerPlayChannels.clear();
    }
    s_lastBridgeTick = now;

    void *dsp = nullptr;
    const int created = s_createDsp(s_system, &s_esDescription, &dsp);
    const int added = created == FMOD_OK ? s_addDsp(group, FMOD_CHANNELCONTROL_DSP_HEAD, dsp) : -1;
    logf("bridge group %p gen %d: createDSP %d addDSP %d\n", group, s_generation.load(), created, added);
    if (added == FMOD_OK) s_bridgedGroups[group] = s_generation.load();

    static int chainsLogged = 0;
    if (added == FMOD_OK && chainsLogged++ < 4) logChain(group);
}

int hookCreateSound(void *system, const char *name, unsigned int mode, void *exinfo, void **sound) {
    const int result = s_createSound(system, name, mode, exinfo, sound);

    static std::atomic<int> logged{0};
    if (logged++ < 40) {
        if (mode & (FMOD_OPENMEMORY | FMOD_OPENMEMORY_POINT)) logf("createSound mode %08x (memory) -> %d\n", mode, result);
        else logf("createSound mode %08x %.120s -> %d\n", mode, (mode & FMOD_OPENUSER) || !name ? "(user)" : name, result);
    }

    if (result == FMOD_OK && sound && isMarker(name, mode, exinfo)) {
        std::lock_guard<std::mutex> lock(s_mutex);
        s_system = system;
        s_markerSounds.insert(*sound);
        logf("marker sound %p\n", *sound);
    }
    return result;
}

int hookPlaySound(void *system, void *sound, void *group, bool paused, void **channel) {
    const int result = s_playSound(system, sound, group, paused, channel);
    if (result != FMOD_OK) return result;

    std::lock_guard<std::mutex> lock(s_mutex);
    if (s_markerSounds.count(sound)) {
        static std::atomic<int> logged{0};
        if (logged++ < 20) logf("marker play: group %p channel %p\n", group, channel ? *channel : nullptr);
        if (group) bridgeGroup(group);
        else if (channel) s_markerChannels.insert(*channel);

        if (channel && *channel) {
            s_markerPlayChannels.push_back(*channel);
            s_lowCutDirty = true;
        }
    }

    // playSound runs constantly in game, which makes it a good place for these game-thread jobs
    applyLowCut();
    maybeDumpEventBuses();
    return result;
}

int hookSetChannelGroup(void *channel, void *group) {
    const int result = s_setChannelGroup(channel, group);
    if (result != FMOD_OK) return result;

    std::lock_guard<std::mutex> lock(s_mutex);
    if (s_markerChannels.erase(channel)) {
        logf("marker channel %p -> group %p\n", channel, group);
        bridgeGroup(group);
        s_markerPlayChannels.push_back(channel);
        s_lowCutDirty = true;
    }
    return result;
}

void installHooks() {
    void *createSound = dlsym(RTLD_DEFAULT, fmod_symbols::CreateSound);
    void *playSound = dlsym(RTLD_DEFAULT, fmod_symbols::PlaySound);
    void *setChannelGroup = dlsym(RTLD_DEFAULT, fmod_symbols::SetChannelGroup);
    s_createDsp = reinterpret_cast<CreateDspFn>(dlsym(RTLD_DEFAULT, fmod_symbols::CreateDsp));
    s_addDsp = reinterpret_cast<AddDspFn>(dlsym(RTLD_DEFAULT, fmod_symbols::AddDsp));

    s_getNumDsps = reinterpret_cast<GetNumDspsFn>(dlsym(RTLD_DEFAULT, fmod_symbols::GetNumDsps));
    s_getDsp = reinterpret_cast<GetDspFn>(dlsym(RTLD_DEFAULT, fmod_symbols::GetDsp));
    s_getParentGroup = reinterpret_cast<GetParentGroupFn>(dlsym(RTLD_DEFAULT, fmod_symbols::GetParentGroup));
    s_getGroupName = reinterpret_cast<GetGroupNameFn>(dlsym(RTLD_DEFAULT, fmod_symbols::GetGroupName));
    s_dspGetInfo = reinterpret_cast<DspGetInfoFn>(dlsym(RTLD_DEFAULT, fmod_symbols::DspGetInfo));
    s_dspGetChannelFormat = reinterpret_cast<DspGetChannelFormatFn>(dlsym(RTLD_DEFAULT, fmod_symbols::DspGetChannelFormat));
    s_dspGetBypass = reinterpret_cast<DspGetBypassFn>(dlsym(RTLD_DEFAULT, fmod_symbols::DspGetBypass));
    s_dspGetNumParameters = reinterpret_cast<DspGetNumParametersFn>(dlsym(RTLD_DEFAULT, fmod_symbols::DspGetNumParameters));
    s_dspGetParameterInfo = reinterpret_cast<DspGetParameterInfoFn>(dlsym(RTLD_DEFAULT, fmod_symbols::DspGetParameterInfo));
    s_dspGetParameterFloat = reinterpret_cast<DspGetParameterFloatFn>(dlsym(RTLD_DEFAULT, fmod_symbols::DspGetParameterFloat));
    s_dspGetParameterInt = reinterpret_cast<DspGetParameterIntFn>(dlsym(RTLD_DEFAULT, fmod_symbols::DspGetParameterInt));
    s_dspGetParameterBool = reinterpret_cast<DspGetParameterBoolFn>(dlsym(RTLD_DEFAULT, fmod_symbols::DspGetParameterBool));
    s_dspSetBypass = reinterpret_cast<DspSetBypassFn>(dlsym(RTLD_DEFAULT, fmod_symbols::DspSetBypass));
    s_channelGetChannelGroup = reinterpret_cast<ChannelGetChannelGroupFn>(dlsym(RTLD_DEFAULT, fmod_symbols::ChannelGetChannelGroup));

    logf("resolve: createSound %p playSound %p setChannelGroup %p createDSP %p addDSP %p\n",
        createSound, playSound, setChannelGroup, reinterpret_cast<void *>(s_createDsp), reinterpret_cast<void *>(s_addDsp));
    if (!createSound || !playSound || !setChannelGroup || !s_createDsp || !s_addDsp) {
        logf("FMOD functions not found, bridge disabled (game update?)\n");
        return;
    }

    struct { void *target; void *detour; void **original; const char *name; } hooks[] = {
        { createSound, reinterpret_cast<void *>(hookCreateSound), reinterpret_cast<void **>(&s_createSound), "createSound" },
        { playSound, reinterpret_cast<void *>(hookPlaySound), reinterpret_cast<void **>(&s_playSound), "playSound" },
        { setChannelGroup, reinterpret_cast<void *>(hookSetChannelGroup), reinterpret_cast<void **>(&s_setChannelGroup), "setChannelGroup" },
    };
    for (auto &hook : hooks) {
        std::string error;
        const bool ok = installInlineHook(hook.target, hook.detour, hook.original, &error);
        logf("hook %s: %s\n", hook.name, ok ? "ok" : error.c_str());
    }
}

void initialize() {
    openLog();
    logf("FMODGetDSPDescription (engine-sim BeamNG plugin)\n");

    s_stubDescription.pluginsdkversion = FMOD_PLUGIN_SDK_VERSION;
    std::strncpy(s_stubDescription.name, "BeamNGBlendLayer", sizeof(s_stubDescription.name) - 1);
    s_stubDescription.version = 0x00010000;
    s_stubDescription.numinputbuffers = 1;
    s_stubDescription.numoutputbuffers = 1;

    s_esDescription = s_stubDescription;
    std::memset(s_esDescription.name, 0, sizeof(s_esDescription.name));
    std::strncpy(s_esDescription.name, "EngineSimBridge", sizeof(s_esDescription.name) - 1);
    s_esDescription.create = esCreate;
    s_esDescription.release = esRelease;
    s_esDescription.process = esProcess;
    buildSinc();

    std::thread(receiverThread).detach();
    installHooks();
}

} // namespace

EXPORT FmodDspDescription *FMODGetDSPDescription() {
    static std::once_flag once;
    std::call_once(once, initialize);
    return &s_stubDescription;
}
