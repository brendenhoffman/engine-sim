#include "wav_reader.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>

namespace {
    constexpr uint16_t FormatPcm = 1;
    constexpr uint16_t FormatFloat = 3;
    constexpr uint16_t FormatExtensible = 0xFFFE;

    uint16_t readU16(const uint8_t *p) { return uint16_t(p[0] | (p[1] << 8)); }
    uint32_t readU32(const uint8_t *p) { return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24); }

    // One sample, scaled to [-1, 1]
    double decodeSample(const uint8_t *p, uint16_t format, int bits) {
        if (format == FormatFloat) {
            if (bits == 32) { float f; std::memcpy(&f, p, 4); return f; }
            double d; std::memcpy(&d, p, 8); return d;
        }

        switch (bits) {
            case 8: return (p[0] - 128) / 128.0; // 8-bit WAV is unsigned
            case 16: return int16_t(readU16(p)) / 32768.0;
            case 24: return (int32_t(readU32(p) << 8) >> 8) / 8388608.0; // sign-extend 24 bits
            default: return int32_t(readU32(p)) / 2147483648.0;
        }
    }
}

bool loadWavMono16(
    const std::string &path,
    int targetSampleRate,
    std::vector<int16_t> *samples,
    std::string *error)
{
    auto fail = [&](const std::string &why) {
        *error = path + ": " + why;
        return false;
    };

    std::ifstream file(path, std::ios::binary);
    if (!file) return fail("could not open file");
    const std::vector<uint8_t> data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    if (data.size() < 12 || std::memcmp(data.data(), "RIFF", 4) != 0 || std::memcmp(data.data() + 8, "WAVE", 4) != 0) {
        return fail("not a RIFF/WAVE file");
    }

    uint16_t format = 0, channels = 0, bits = 0;
    uint32_t sampleRate = 0;
    const uint8_t *pcm = nullptr;
    size_t pcmSize = 0;

    // Walk the chunks; each is padded to an even length
    for (size_t offset = 12; offset + 8 <= data.size();) {
        const uint8_t *chunk = data.data() + offset;
        const size_t size = readU32(chunk + 4);
        const size_t available = std::min(size, data.size() - offset - 8);

        if (std::memcmp(chunk, "fmt ", 4) == 0 && available >= 16) {
            format = readU16(chunk + 8);
            channels = readU16(chunk + 10);
            sampleRate = readU32(chunk + 12);
            bits = readU16(chunk + 22);
            if (format == FormatExtensible && available >= 26) {
                format = readU16(chunk + 32); // first two bytes of the sub-format GUID
            }
        }
        else if (std::memcmp(chunk, "data", 4) == 0) {
            pcm = chunk + 8;
            pcmSize = available; // tolerate truncated files
        }

        offset += 8 + size + (size & 1);
    }

    if (pcm == nullptr || channels == 0 || sampleRate == 0) return fail("missing fmt or data chunk");

    const bool supported =
        (format == FormatPcm && (bits == 8 || bits == 16 || bits == 24 || bits == 32))
        || (format == FormatFloat && (bits == 32 || bits == 64));
    if (!supported) {
        return fail("unsupported encoding (format " + std::to_string(format) + ", " + std::to_string(bits) + "-bit)");
    }

    const size_t frameBytes = size_t(channels) * (bits / 8);
    const size_t frames = pcmSize / frameBytes;

    // Already in the target format (the usual case): copy samples exactly
    if (format == FormatPcm && bits == 16 && channels == 1 && int(sampleRate) == targetSampleRate) {
        samples->resize(frames);
        for (size_t i = 0; i < frames; ++i) (*samples)[i] = int16_t(readU16(pcm + 2 * i));
        return true;
    }

    std::vector<double> mono(frames);
    for (size_t i = 0; i < frames; ++i) {
        double sum = 0;
        for (int c = 0; c < channels; ++c) {
            sum += decodeSample(pcm + i * frameBytes + c * (bits / 8), format, bits);
        }
        mono[i] = sum / channels;
    }

    // Linear resample; impulse responses are short, so quality/speed isn't a concern here
    const double step = double(sampleRate) / targetSampleRate;
    const size_t outFrames = (frames == 0) ? 0 : size_t(std::floor((frames - 1) / step)) + 1;

    samples->resize(outFrames);
    for (size_t i = 0; i < outFrames; ++i) {
        const double position = i * step;
        const size_t i0 = size_t(position);
        const size_t i1 = std::min(i0 + 1, frames - 1);
        const double t = position - i0;
        const double v = mono[i0] + (mono[i1] - mono[i0]) * t;
        (*samples)[i] = int16_t(std::lround(std::clamp(v * 32767.0, -32768.0, 32767.0)));
    }

    return true;
}
