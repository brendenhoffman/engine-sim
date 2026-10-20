#include "../include/yds_sdl_audio_source.h"

#include "../include/yds_sdl_audio_buffer.h"

#include <algorithm>
#include <cmath>

ysSdlAudioSource::ysSdlAudioSource() : ysAudioSource(API::Sdl) {
    std::lock_guard<std::mutex> guard(m_mutex);
    m_readPosition = 0;
    m_safeWriteFrames = 0;
}

ysSdlAudioSource::~ysSdlAudioSource() {
    /* void */
}

ysError ysSdlAudioSource::LockEntireBuffer(void **buffer, SampleOffset *samples) {
    YDS_ERROR_DECLARE("LockEntireBuffer");

    YDS_NESTED_ERROR_CALL( ysAudioSource::LockEntireBuffer(buffer, samples) );
    m_mutex.lock();

    ysSdlAudioBuffer *sdlBuffer = static_cast<ysSdlAudioBuffer *>(m_dataBuffer);
    *buffer = sdlBuffer->GetBuffer();

    return YDS_ERROR_RETURN(ysError::None);
}

ysError ysSdlAudioSource::UnlockBuffer(void *buffer, SampleOffset samples) {
    YDS_ERROR_DECLARE("UnlockBuffer");

    YDS_NESTED_ERROR_CALL(ysAudioSource::UnlockBuffer(buffer, samples));

    m_mutex.unlock();
    return YDS_ERROR_RETURN(ysError::None);
}

ysError ysSdlAudioSource::LockBufferSegment(SampleOffset offset, SampleOffset samples, void **segment1, SampleOffset* size1, void **segment2, SampleOffset* size2) {
    YDS_ERROR_DECLARE("LockBufferSegment");

    YDS_NESTED_ERROR_CALL( ysAudioSource::LockBufferSegment(offset, samples, segment1, size1, segment2, size2) );
    m_mutex.lock();

    size_t byteOffset = m_audioParameters.GetSizeFromSamples(offset);

    ysSdlAudioBuffer *sdlBuffer = static_cast<ysSdlAudioBuffer *>(m_dataBuffer);
    *segment1 = (char*)sdlBuffer->GetBuffer() + byteOffset;
    *segment2 = sdlBuffer->GetBuffer();

    return YDS_ERROR_RETURN(ysError::None);
}

ysError ysSdlAudioSource::UnlockBufferSegments(void *segment1, SampleOffset segment1Size, void *segment2, SampleOffset segment2Size) {
    YDS_ERROR_DECLARE("UnlockBufferSegments");

    YDS_NESTED_ERROR_CALL( ysAudioSource::UnlockBufferSegments(segment1, segment1Size, segment2, segment2Size) );

    m_mutex.unlock();
    return YDS_ERROR_RETURN(ysError::None);
}

ysError ysSdlAudioSource::SetMode(Mode mode) {
    YDS_ERROR_DECLARE("SetMode");

    // Stop pauses playback in place (silence, read position frozen). Every other mode loops
    // the buffer, which is all engine-sim uses.
    std::lock_guard<std::mutex> guard(m_mutex);
    YDS_NESTED_ERROR_CALL( ysAudioSource::SetMode(mode) );

    return YDS_ERROR_RETURN(ysError::None);
}

ysError ysSdlAudioSource::SetDataBuffer(ysAudioBuffer *buffer) {
    YDS_ERROR_DECLARE("SetDataBuffer");

    YDS_NESTED_ERROR_CALL(ysAudioSource::SetDataBuffer(buffer));

    ysSdlAudioBuffer *sdlBuffer = static_cast<ysSdlAudioBuffer *>(buffer);

    void *target = nullptr;
    SampleOffset samples;
    YDS_NESTED_ERROR_CALL(LockEntireBuffer(&target, &samples));
    memcpy(target, sdlBuffer->GetBuffer(), sdlBuffer->GetBufferSize());

    YDS_NESTED_ERROR_CALL(UnlockBuffer(target, samples));

    return YDS_ERROR_RETURN(ysError::None);
}

ysError ysSdlAudioSource::SetVolume(float volume) {
    YDS_ERROR_DECLARE("SetVolume");

    std::lock_guard<std::mutex> guard(m_mutex);
    YDS_NESTED_ERROR_CALL( ysAudioSource::SetVolume(volume) );

    return YDS_ERROR_RETURN(ysError::None);
}

ysError ysSdlAudioSource::SetPan(float pan) {
    YDS_ERROR_DECLARE("SetPan");

    // TODO

    return YDS_ERROR_RETURN(ysError::None);
}

SampleOffset ysSdlAudioSource::GetCurrentPosition() {
    std::lock_guard<std::mutex> guard(m_mutex);
    return m_readPosition % m_bufferSize;
}

SampleOffset ysSdlAudioSource::GetCurrentWritePosition() {
    std::lock_guard<std::mutex> guard(m_mutex);
    return (m_readPosition + m_safeWriteFrames) % m_bufferSize;
}

void ysSdlAudioSource::Seek(SampleOffset offset) {
    // TODO
}

ysError ysSdlAudioSource::Destroy() {
    YDS_ERROR_DECLARE("Destroy");

    YDS_NESTED_ERROR_CALL(ysAudioSource::Destroy());

    return YDS_ERROR_RETURN(ysError::None);
}

void ysSdlAudioSource::AddToBuffer(int16_t *audio, int frames) {
    std::lock_guard<std::mutex> guard(m_mutex);
    if (m_bufferMode == Mode::Stop || m_dataBuffer == nullptr) return;

    const float volume = m_volume;
    auto mix = [volume](int16_t &out, int16_t in) {
        const float mixed = out + in * volume;
        out = static_cast<int16_t>(std::clamp(mixed, -32768.0f, 32767.0f));
    };

    ysSdlAudioBuffer *sdlBuffer = static_cast<ysSdlAudioBuffer *>(m_dataBuffer);
    auto *buffer = (int16_t *)sdlBuffer->GetBuffer();
    const size_t startOffset = m_readPosition % m_bufferSize;

    // Read the first segment
    const auto *segment1 = buffer + startOffset;
    const size_t length1 = std::min(size_t(frames), m_bufferSize - startOffset);
    for (size_t i = 0; i < length1; i++) {
        mix(audio[i], segment1[i]);
    }

    // Read the second segment
    const auto *segment2 = buffer;
    const size_t length2 = frames - length1;
    for (size_t i = 0; i < length2; i++) {
        mix(audio[i + length1], segment2[i]);
    }

    // Update position
    m_readPosition += frames;
}
