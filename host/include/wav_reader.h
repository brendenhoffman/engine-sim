#ifndef ATG_ENGINE_SIM_HOST_WAV_READER_H
#define ATG_ENGINE_SIM_HOST_WAV_READER_H

#include <cstdint>
#include <string>
#include <vector>

// Loads a RIFF/WAVE file as 16-bit mono at the requested sample rate.
// Handles integer PCM (8/16/24/32-bit), 32/64-bit float and WAVE_FORMAT_EXTENSIBLE.
// Multi-channel files are averaged down to mono; other rates are linearly resampled.
// Returns false and sets `error` on failure.
bool loadWavMono16(
    const std::string &path,
    int targetSampleRate,
    std::vector<int16_t> *samples,
    std::string *error);

#endif /* ATG_ENGINE_SIM_HOST_WAV_READER_H */
