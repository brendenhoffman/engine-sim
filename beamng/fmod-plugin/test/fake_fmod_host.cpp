// Pretends to be BeamNG: exports FMOD C++ API functions under the same mangled names the game
// exports, loads the plugin, and checks that it hooks them, attaches its DSP to the marker's
// channel group, and plays audio received over UDP. Exit code 0 on success.
//
// Built with -O0 and exported dynamically so the functions look like the game's to the plugin:
// found by dlsym(RTLD_DEFAULT) and with ordinary prologues for the inline hook to move.

#include "../fmod_abi.h"

#include <arpa/inet.h>
#include <dlfcn.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

struct FMOD_CREATESOUNDEXINFO { int cbsize; unsigned int length; };
struct FMOD_DSP_DESCRIPTION;

namespace {
    int g_createSoundCalls = 0, g_playSoundCalls = 0, g_setChannelGroupCalls = 0;
    const FmodDspDescription *g_createdDescription = nullptr;
    void *g_addedToGroup = nullptr;
    int g_addedIndex = 0;
    int g_failures = 0;

    void check(bool ok, const char *what) {
        std::printf("%s %s\n", ok ? "  ok  " : "  FAIL", what);
        if (!ok) ++g_failures;
    }
}

namespace FMOD {
    class Sound;
    class ChannelGroup;
    class DSP;

    class ChannelControl {
    public:
        int addDSP(int index, DSP *dsp);
    };

    class Channel : public ChannelControl {
    public:
        int setChannelGroup(ChannelGroup *group);
    };

    class System {
    public:
        int createSound(const char *name, unsigned int mode, FMOD_CREATESOUNDEXINFO *exinfo, Sound **sound);
        int playSound(Sound *sound, ChannelGroup *group, bool paused, Channel **channel);
        int createDSP(const FMOD_DSP_DESCRIPTION *description, DSP **dsp);
    };

    // volatile locals give each function a plain prologue before any global access
    __attribute__((noinline)) int System::createSound(const char *name, unsigned int mode, FMOD_CREATESOUNDEXINFO *exinfo, Sound **sound) {
        volatile unsigned int m = mode; volatile const void *n = name; volatile const void *e = exinfo; (void)m; (void)n; (void)e;
        ++g_createSoundCalls;
        *sound = reinterpret_cast<Sound *>(0x5001);
        return FMOD_OK;
    }

    __attribute__((noinline)) int System::playSound(Sound *sound, ChannelGroup *group, bool paused, Channel **channel) {
        volatile const void *s = sound; volatile const void *g = group; volatile bool p = paused; (void)s; (void)g; (void)p;
        ++g_playSoundCalls;
        if (channel) *channel = reinterpret_cast<Channel *>(0xC001);
        return FMOD_OK;
    }

    __attribute__((noinline)) int System::createDSP(const FMOD_DSP_DESCRIPTION *description, DSP **dsp) {
        volatile const void *d = description; (void)d;
        g_createdDescription = reinterpret_cast<const FmodDspDescription *>(description);
        *dsp = reinterpret_cast<DSP *>(0xD001);
        return FMOD_OK;
    }

    __attribute__((noinline)) int ChannelControl::addDSP(int index, DSP *dsp) {
        volatile int i = index; volatile const void *d = dsp; (void)i; (void)d;
        g_addedToGroup = this;
        g_addedIndex = index;
        return FMOD_OK;
    }

    __attribute__((noinline)) int Channel::setChannelGroup(ChannelGroup *group) {
        volatile const void *g = group; (void)g;
        ++g_setChannelGroupCalls;
        return FMOD_OK;
    }
}

namespace {
    int sampleRate48k(FmodDspState *, int *rate) { *rate = 48000; return FMOD_OK; }

    void sendSine(double seconds, double frequency, float amplitude) {
        const int fd = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in to{};
        to.sin_family = AF_INET;
        const char *portOverride = std::getenv("ENGINE_SIM_AUDIO_PORT");
        to.sin_port = htons(portOverride != nullptr ? std::atoi(portOverride) : 4571);
        to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

        const int packet = 441; // 10 ms
        std::vector<char> buffer(4 + 4 * packet);
        std::memcpy(buffer.data(), "BESF", 4);
        long long n = 0;
        for (int p = 0; p < seconds * 100; ++p) {
            for (int i = 0; i < packet; ++i, ++n) {
                const float v = amplitude * static_cast<float>(std::sin(2 * M_PI * frequency * n / 44100.0));
                std::memcpy(buffer.data() + 4 + 4 * i, &v, 4);
            }
            sendto(fd, buffer.data(), buffer.size(), 0, reinterpret_cast<sockaddr *>(&to), sizeof(to));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        close(fd);
    }
}

int main(int argc, char **argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s path/to/libBeamNGBlendLayerDSP.so\n", argv[0]);
        return 2;
    }

    void *plugin = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    if (!plugin) {
        std::fprintf(stderr, "dlopen: %s\n", dlerror());
        return 1;
    }

    auto getDescription = reinterpret_cast<FmodDspDescription *(*)()>(dlsym(plugin, "FMODGetDSPDescription"));
    check(getDescription != nullptr, "exports FMODGetDSPDescription");
    if (!getDescription) return 1;

    const FmodDspDescription *stub = getDescription();
    check(std::strcmp(stub->name, "BeamNGBlendLayer") == 0, "stub description is named BeamNGBlendLayer");
    check(stub->pluginsdkversion == 110 && stub->process == nullptr, "stub matches the stock pass-through plugin");
    check(getDescription() == stub, "repeat calls return the same description");

    FMOD::System system;
    auto *group = reinterpret_cast<FMOD::ChannelGroup *>(0x6001);

    // An ordinary sound must pass straight through
    FMOD::Sound *other = nullptr;
    system.createSound("art/sound/blends/v8.wav", 0, nullptr, &other);
    FMOD::Channel *channel = nullptr;
    system.playSound(other, group, false, &channel);
    check(g_createSoundCalls == 1 && g_playSoundCalls == 1, "hooked functions still reach the originals");
    check(g_createdDescription == nullptr, "no DSP for ordinary sounds");

    // The marker sample gets the engine-sim DSP at the head of its group
    FMOD::Sound *marker = nullptr;
    system.createSound("/mods/engineSimBridge/art/sound/engineSim/engineSimMarkerEngine.wav", 0, nullptr, &marker);
    check(marker != nullptr, "marker createSound returns the sound");
    system.playSound(marker, group, false, &channel);
    check(g_createdDescription != nullptr && std::strcmp(g_createdDescription->name, "EngineSimBridge") == 0, "marker play creates the EngineSimBridge DSP");
    check(g_addedToGroup == group && g_addedIndex == FMOD_CHANNELCONTROL_DSP_HEAD, "DSP added at the head of the marker's group");
    if (g_createdDescription == nullptr) return 1;

    FmodDspStateFunctions functions{};
    functions.getsamplerate = sampleRate48k;
    FmodDspState state{};
    state.functions = &functions;
    check(g_createdDescription->create(&state) == FMOD_OK, "DSP create");

    const unsigned int length = 1024;
    const int channels = 2;
    std::vector<float> input(length * channels, 0.25f), output(length * channels, 0.0f);
    int inChannels = channels, outChannels = channels;
    unsigned int inMask = 0, outMask = 0;
    float *inBuffers[] = { input.data() };
    float *outBuffers[] = { output.data() };
    FmodDspBufferArray in{ 1, &inChannels, &inMask, inBuffers, 0 };
    FmodDspBufferArray out{ 1, &outChannels, &outMask, outBuffers, 0 };

    // The DSP is already waiting when the first, tiny packet of the session arrives (what happens
    // after a tuning sweep): it must not read before the start of the stream
    {
        const int fd = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in to{};
        to.sin_family = AF_INET;
        const char *portOverride = std::getenv("ENGINE_SIM_AUDIO_PORT");
        to.sin_port = htons(portOverride != nullptr ? std::atoi(portOverride) : 4571);
        to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        char packet[4 + 4 * 100];
        std::memcpy(packet, "BESF", 4);
        const float quiet = 0.1f;
        for (int i = 0; i < 100; ++i) std::memcpy(packet + 4 + 4 * i, &quiet, 4);
        sendto(fd, packet, sizeof(packet), 0, reinterpret_cast<sockaddr *>(&to), sizeof(to));
        close(fd);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    bool allFinite = true;
    for (int block = 0; block < 4; ++block) {
        g_createdDescription->process(&state, length, &in, &out, 0, FMOD_DSP_PROCESS_PERFORM);
        for (float v : output) allFinite = allFinite && std::isfinite(v) && std::fabs(v) <= 1.0f;
    }
    check(allFinite, "first tiny packet of a session gives clean output");
    check(output[0] == 0.25f, "passes the game's sound through until enough audio has arrived");

    // Feed audio like engine-sim does, then pull it through the DSP at a 48 kHz mixer rate
    std::thread sender(sendSine, 0.6, 440.0, 0.5f);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    g_createdDescription->process(&state, length, &in, &out, 0, FMOD_DSP_PROCESS_PERFORM);
    float peak = 0;
    for (float v : output) peak = std::fmax(peak, std::fabs(v));
    check(peak > 0.4f && peak < 0.6f, "plays engine-sim audio while it's arriving");
    check(output[0] == output[1], "same signal on every channel");

    // A second copy of the sound (BeamNG plays the stream from several emitters) renders the
    // same samples as the first in every mix block, so the copies can't drift out of phase
    {
        FmodDspState second{};
        second.functions = &functions;
        g_createdDescription->create(&second);
        std::vector<float> output2(length * channels, 0.0f);
        float *outBuffers2[] = { output2.data() };
        FmodDspBufferArray out2{ 1, &outChannels, &outMask, outBuffers2, 0 };

        bool same = true, moving = false;
        std::vector<float> previous;
        for (int block = 0; block < 6; ++block) {
            g_createdDescription->process(&state, length, &in, &out, 0, FMOD_DSP_PROCESS_PERFORM);
            g_createdDescription->process(&second, length, &in, &out2, 0, FMOD_DSP_PROCESS_PERFORM);
            same = same && output == output2;
            moving = moving || (!previous.empty() && previous != output);
            previous = output;
        }
        check(same, "two copies of the sound play identical samples in each mix block");
        check(moving, "and the shared stream moves on from block to block");
        g_createdDescription->release(&second);
    }

    sender.join();
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    g_createdDescription->process(&state, length, &in, &out, 0, FMOD_DSP_PROCESS_PERFORM);
    check(output[0] == 0.25f && output[length * channels - 1] == 0.25f, "passes the game's sound through once engine-sim goes quiet");

    g_createdDescription->release(&state);

    std::printf("%s\n", g_failures == 0 ? "all checks passed" : "FAILED");
    return g_failures == 0 ? 0 : 1;
}
