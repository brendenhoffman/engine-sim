#include "../include/yds_sdl_audio_wave_file.h"

#include <SDL3/SDL.h>

#include <cstring>

ysSdlAudioWaveFile::ysSdlAudioWaveFile() : ysAudioFile(Platform::Sdl, AudioFormat::Wave) {
    m_dataLength = 0;
    m_data = nullptr;
}

ysSdlAudioWaveFile::~ysSdlAudioWaveFile() {
    DestroyInternalBuffer();
    CloseFile();
}

ysAudioFile::Error ysSdlAudioWaveFile::OpenFile(const char *fname) {
    ysAudioFile::Error error = ysAudioFile::OpenFile(fname);
    if (error != Error::None) return error;

    // Lazy approach: load everything since files are small
    SDL_AudioSpec fileSpec{};
    Uint8 *fileData = nullptr;
    Uint32 fileLength = 0;
    if (!SDL_LoadWAV(fname, &fileSpec, &fileData, &fileLength)) {
        return Error::CouldNotOpenFile;
    }

    // Callers read raw samples, so always hand back 16-bit mono at 44.1 kHz whatever the file is
    const SDL_AudioSpec targetSpec = { SDL_AUDIO_S16, 1, 44100 };
    int convertedLength = 0;
    const bool converted = SDL_ConvertAudioSamples(
        &fileSpec, fileData, static_cast<int>(fileLength), &targetSpec, &m_data, &convertedLength);
    SDL_free(fileData);
    if (!converted) {
        return Error::CouldNotOpenFile;
    }

    // Everything worked, commit data
    m_dataLength = static_cast<uint32_t>(convertedLength);
    m_audioParameters.m_channelCount = targetSpec.channels;
    m_audioParameters.m_sampleRate = targetSpec.freq;
    m_audioParameters.m_bitsPerSample = SDL_AUDIO_BITSIZE(targetSpec.format);
    m_sampleCount = m_audioParameters.GetSamplesFromSize(m_dataLength);
    m_fileOpen = true;

    return Error::None;
}

ysAudioFile::Error ysSdlAudioWaveFile::CloseFile() {
    Error error = ysAudioFile::CloseFile();
    if (error != Error::None) return error;

    if (m_data != nullptr) {
        SDL_free(m_data);
        m_data = nullptr;
        m_dataLength = 0;
    }

    return Error::None;
}

ysAudioFile::Error ysSdlAudioWaveFile::GenericRead(SampleOffset offset, SampleOffset size, void *buffer) {
    Error error = ysAudioFile::GenericRead(offset, size, buffer);
    if (error != Error::None) return error;

    // Work out where we need to read from
    const long fileOffset = m_audioParameters.GetSizeFromSamples(offset);
    const long fileSize = m_audioParameters.GetSizeFromSamples(size);

    // Check that there's space
    if (fileSize + fileOffset > m_dataLength) {
        return Error::FileReadError;
    }

    // Read it
    memcpy(buffer, m_data + fileOffset, fileSize);
    return Error::None;
}
