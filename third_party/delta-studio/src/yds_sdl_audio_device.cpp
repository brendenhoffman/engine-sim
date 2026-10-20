#include "../include/yds_sdl_audio_device.h"

#include "../include/yds_sdl_audio_buffer.h"
#include "../include/yds_sdl_audio_source.h"

#include <algorithm>
#include <cstring>

ysSdlAudioDevice::ysSdlAudioDevice() : ysAudioDevice(API::Sdl) {
    m_stream = nullptr;
    m_outputFormat = { SDL_AUDIO_S16, 1, 44100 }; // TODO: multi-channel support
    m_safeWriteFrames = 0;
}

ysSdlAudioDevice::~ysSdlAudioDevice() {
    // Stops the callback before the sources it reads from go away
    if (m_stream != nullptr) SDL_DestroyAudioStream(m_stream);
}

bool ysSdlAudioDevice::Open() {
    m_stream = SDL_OpenAudioDeviceStream(
        SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &m_outputFormat, &ysSdlAudioDevice::StreamCallback, this);
    if (m_stream == nullptr) return false;

    // How far ahead of the read position writers must stay: one device buffer, converted from
    // the hardware rate to ours. The stream starts paused; UpdateAudioSources() resumes it.
    SDL_AudioSpec deviceFormat;
    int deviceFrames = 0;
    if (SDL_GetAudioDeviceFormat(SDL_GetAudioStreamDevice(m_stream), &deviceFormat, &deviceFrames)
        && deviceFormat.freq > 0)
    {
        m_safeWriteFrames = static_cast<size_t>(
            (static_cast<long long>(deviceFrames) * m_outputFormat.freq + deviceFormat.freq - 1) / deviceFormat.freq);
    }
    else {
        m_safeWriteFrames = 512;
    }

    return true;
}

ysAudioBuffer *ysSdlAudioDevice::CreateBuffer(const ysAudioParameters *parameters, SampleOffset size) {
    ysSdlAudioBuffer *newBuffer = m_audioBuffers.NewGeneric<ysSdlAudioBuffer>();
    newBuffer->Initialize(size, *parameters);

    return newBuffer;
}

ysAudioSource *ysSdlAudioDevice::CreateSource(const ysAudioParameters *parameters, SampleOffset size) {
    // Sources are mixed straight into the stream, so they must match its input format
    if (m_stream == nullptr ||
        parameters->m_sampleRate != m_outputFormat.freq ||
        parameters->m_channelCount != m_outputFormat.channels ||
        parameters->m_bitsPerSample != static_cast<int>(SDL_AUDIO_BITSIZE(m_outputFormat.format))) {
        return nullptr;
    }

    ysSdlAudioSource *newSource = m_audioSources.NewGeneric<ysSdlAudioSource>();

    newSource->m_bufferSize = size;
    newSource->m_audioParameters = *parameters;
    newSource->m_safeWriteFrames = m_safeWriteFrames;

    return newSource;
}

ysAudioSource *ysSdlAudioDevice::CreateSource(ysAudioBuffer *sourceBuffer) {
    ysAudioSource *newSource = CreateSource(sourceBuffer->GetAudioParameters(), sourceBuffer->GetSampleCount());
    if (newSource == nullptr) return nullptr;

    newSource->SetDataBuffer(sourceBuffer);

    return newSource;
}

void ysSdlAudioDevice::UpdateAudioSources() {
    if (m_stream == nullptr) return;

    if (m_audioSources.GetNumObjects() > 0) {
        SDL_ResumeAudioStreamDevice(m_stream);
    } else {
        SDL_PauseAudioStreamDevice(m_stream);
    }
}

void SDLCALL ysSdlAudioDevice::StreamCallback(void *userdata, SDL_AudioStream *stream, int additionalAmount, int totalAmount) {
    (void)totalAmount;

    auto *device = static_cast<ysSdlAudioDevice *>(userdata);
    int16_t chunk[1024];

    // Feed exactly what the device asked for, in stack-sized pieces
    int frames = additionalAmount / static_cast<int>(sizeof(int16_t));
    while (frames > 0) {
        const int count = std::min(frames, static_cast<int>(sizeof(chunk) / sizeof(chunk[0])));
        device->FillBuffer(chunk, count);
        SDL_PutAudioStreamData(stream, chunk, count * static_cast<int>(sizeof(int16_t)));
        frames -= count;
    }
}

void ysSdlAudioDevice::FillBuffer(int16_t *audio, int frames) {
    // Clear the buffer
    memset(audio, 0, frames * sizeof(int16_t));

    // Mix each source in
    const int sourceCount = m_audioSources.GetNumObjects();
    for (int i = 0; i < sourceCount; ++i) {
        auto *sdlBuffer = static_cast<ysSdlAudioSource *>(m_audioSources.Get(i));
        sdlBuffer->AddToBuffer(audio, frames);
    }
}
