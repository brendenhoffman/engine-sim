#ifndef ENGINE_SIM_BEAMNG_FMOD_ABI_H
#define ENGINE_SIM_BEAMNG_FMOD_ABI_H

// The parts of the FMOD 2.x plugin and C++ APIs the plugin touches, declared by hand so the
// FMOD SDK isn't needed to build. Layout checked against BeamNG's stock
// libBeamNGBlendLayerDSP.so (plugin SDK 110, 216-byte description) and FMOD 2.03.

struct FmodDspState;

struct FmodDspStateFunctions {
    void *alloc;
    void *realloc;
    void *free;
    int (*getsamplerate)(FmodDspState *, int *);
};

struct FmodDspState {
    void *instance;
    void *plugindata;
    unsigned int channelmask;
    int source_speakermode;
    float *sidechaindata;
    int sidechainchannels;
    FmodDspStateFunctions *functions;
    int systemobject;
};

struct FmodDspBufferArray {
    int numbuffers;
    int *buffernumchannels;
    unsigned int *bufferchannelmask;
    float **buffers;
    int speakermode;
};

using FmodStateCallback = int (*)(FmodDspState *);
using FmodProcessCallback = int (*)(FmodDspState *, unsigned int length, const FmodDspBufferArray *in, FmodDspBufferArray *out, int inputsIdle, int op);

struct FmodDspDescription {
    unsigned int pluginsdkversion;
    char name[32];
    unsigned int version;
    int numinputbuffers;
    int numoutputbuffers;
    FmodStateCallback create, release, reset;
    void *read;
    FmodProcessCallback process;
    void *setposition;
    int numparameters;
    void **paramdesc;
    void *setters[4];
    void *getters[4];
    void *shouldiprocess, *userdata, *sys_register, *sys_deregister, *sys_mix;
};
static_assert(sizeof(FmodDspDescription) == 216, "FMOD_DSP_DESCRIPTION layout");

enum {
    FMOD_OK = 0,
    FMOD_DSP_PROCESS_PERFORM = 0,
    FMOD_OPENUSER = 0x400,
    FMOD_OPENMEMORY = 0x800,
    FMOD_OPENMEMORY_POINT = 0x10000000,
    FMOD_CHANNELCONTROL_DSP_HEAD = -1,
    FMOD_PLUGIN_SDK_VERSION = 110,
};

// FMOD C++ API member functions (Itanium ABI: `this` is the first argument). BeamNG links FMOD
// statically but exports these from BeamNG.drive.x64, so they're found by their mangled names.
using CreateSoundFn = int (*)(void *system, const char *name, unsigned int mode, void *exinfo, void **sound);
using PlaySoundFn = int (*)(void *system, void *sound, void *group, bool paused, void **channel);
using SetChannelGroupFn = int (*)(void *channel, void *group);
using CreateDspFn = int (*)(void *system, const FmodDspDescription *description, void **dsp);
using AddDspFn = int (*)(void *channelControl, int index, void *dsp);

// Read-only queries, used to log the processing chain around our DSP
using GetNumDspsFn = int (*)(void *channelControl, int *count);
using GetDspFn = int (*)(void *channelControl, int index, void **dsp);
using GetParentGroupFn = int (*)(void *group, void **parent);
using GetGroupNameFn = int (*)(void *group, char *name, int length);
using DspGetInfoFn = int (*)(void *dsp, char *name, unsigned int *version, int *channels, int *configWidth, int *configHeight);
using DspGetChannelFormatFn = int (*)(void *dsp, unsigned int *channelMask, int *channels, int *speakerMode);
using DspGetBypassFn = int (*)(void *dsp, bool *bypass);

namespace fmod_symbols {
    constexpr const char *CreateSound = "_ZN4FMOD6System11createSoundEPKcjP22FMOD_CREATESOUNDEXINFOPPNS_5SoundE";
    constexpr const char *PlaySound = "_ZN4FMOD6System9playSoundEPNS_5SoundEPNS_12ChannelGroupEbPPNS_7ChannelE";
    constexpr const char *SetChannelGroup = "_ZN4FMOD7Channel15setChannelGroupEPNS_12ChannelGroupE";
    constexpr const char *CreateDsp = "_ZN4FMOD6System9createDSPEPK20FMOD_DSP_DESCRIPTIONPPNS_3DSPE";
    constexpr const char *AddDsp = "_ZN4FMOD14ChannelControl6addDSPEiPNS_3DSPE";

    constexpr const char *GetNumDsps = "_ZN4FMOD14ChannelControl10getNumDSPsEPi";
    constexpr const char *GetDsp = "_ZN4FMOD14ChannelControl6getDSPEiPPNS_3DSPE";
    constexpr const char *GetParentGroup = "_ZN4FMOD12ChannelGroup14getParentGroupEPPS0_";
    constexpr const char *GetGroupName = "_ZN4FMOD12ChannelGroup7getNameEPci";
    constexpr const char *DspGetInfo = "_ZN4FMOD3DSP7getInfoEPcPjPiS3_S3_";
    constexpr const char *DspGetChannelFormat = "_ZN4FMOD3DSP16getChannelFormatEPjPiP16FMOD_SPEAKERMODE";
    constexpr const char *DspGetBypass = "_ZN4FMOD3DSP9getBypassEPb";
}

#endif /* ENGINE_SIM_BEAMNG_FMOD_ABI_H */
