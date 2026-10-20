#ifndef YDS_SDL_AUDIO_DEVICE_H
#define YDS_SDL_AUDIO_DEVICE_H

#include "yds_audio_device.h"

#include <SDL3/SDL_audio.h>

#include <cstddef>
#include <cstdint>

class ysSdlAudioSystem;

class ysSdlAudioDevice : public ysAudioDevice {
    friend ysSdlAudioSystem;

public:
    ysSdlAudioDevice();
    virtual ~ysSdlAudioDevice() override;

    virtual ysAudioBuffer *CreateBuffer(const ysAudioParameters *parameters, SampleOffset size) override;
    virtual ysAudioSource *CreateSource(const ysAudioParameters *parameters, SampleOffset size) override;
    virtual ysAudioSource *CreateSource(ysAudioBuffer *sourceBuffer) override;
    virtual void UpdateAudioSources() override;

private:
    bool Open();
    void FillBuffer(int16_t *audio, int frames);
    static void SDLCALL StreamCallback(void *userdata, SDL_AudioStream *stream, int additionalAmount, int totalAmount);

private:
    SDL_AudioStream *m_stream;
    SDL_AudioSpec m_outputFormat;
    size_t m_safeWriteFrames;
};

#endif /* YDS_SDL_AUDIO_DEVICE_H */
