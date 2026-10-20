#include <gtest/gtest.h>

#include "wav_reader.h"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>

namespace {
    void put16(std::vector<uint8_t> &out, uint16_t v) { out.push_back(v & 0xFF); out.push_back(v >> 8); }
    void put32(std::vector<uint8_t> &out, uint32_t v) { for (int i = 0; i < 4; ++i) out.push_back((v >> (8 * i)) & 0xFF); }
    void tag(std::vector<uint8_t> &out, const char *t) { out.insert(out.end(), t, t + 4); }

    // Minimal PCM WAV; `data` is already interleaved and encoded
    std::string writeWav(const std::string &name, int channels, int rate, int bits, const std::vector<uint8_t> &data) {
        std::vector<uint8_t> file;
        tag(file, "RIFF"); put32(file, 36 + data.size()); tag(file, "WAVE");
        tag(file, "fmt "); put32(file, 16); put16(file, 1); put16(file, channels);
        put32(file, rate); put32(file, rate * channels * bits / 8); put16(file, channels * bits / 8); put16(file, bits);
        tag(file, "data"); put32(file, data.size());
        file.insert(file.end(), data.begin(), data.end());

        const std::string path = (std::filesystem::temp_directory_path() / name).string();
        std::ofstream(path, std::ios::binary).write(reinterpret_cast<const char *>(file.data()), file.size());
        return path;
    }
}

TEST(WavReaderTests, Mono16AtTargetRateIsExact) {
    std::vector<uint8_t> data;
    for (int16_t v : { int16_t(0), int16_t(1), int16_t(-1), int16_t(32767), int16_t(-32768) }) put16(data, uint16_t(v));
    const std::string path = writeWav("es-wav-mono16.wav", 1, 44100, 16, data);

    std::vector<int16_t> samples;
    std::string error;
    ASSERT_TRUE(loadWavMono16(path, 44100, &samples, &error)) << error;
    EXPECT_EQ(samples, (std::vector<int16_t>{ 0, 1, -1, 32767, -32768 }));

    std::filesystem::remove(path);
}

TEST(WavReaderTests, Stereo24IsDownmixed) {
    // Left at +half scale, right at -quarter scale: the average is +1/8
    std::vector<uint8_t> data;
    for (int i = 0; i < 4; ++i) {
        const int32_t left = 0x400000, right = -0x200000;
        for (int32_t v : { left, right }) {
            data.push_back(v & 0xFF); data.push_back((v >> 8) & 0xFF); data.push_back((v >> 16) & 0xFF);
        }
    }
    const std::string path = writeWav("es-wav-stereo24.wav", 2, 44100, 24, data);

    std::vector<int16_t> samples;
    std::string error;
    ASSERT_TRUE(loadWavMono16(path, 44100, &samples, &error)) << error;
    ASSERT_EQ(samples.size(), 4u);
    for (int16_t s : samples) EXPECT_NEAR(s, 32767 / 8, 2);

    std::filesystem::remove(path);
}

TEST(WavReaderTests, ResamplesToTargetRate) {
    std::vector<uint8_t> data;
    for (int i = 0; i < 480; ++i) put16(data, uint16_t(int16_t(1000)));
    const std::string path = writeWav("es-wav-48k.wav", 1, 48000, 16, data);

    std::vector<int16_t> samples;
    std::string error;
    ASSERT_TRUE(loadWavMono16(path, 44100, &samples, &error)) << error;
    EXPECT_NEAR(static_cast<double>(samples.size()), 441.0, 1.0);
    EXPECT_NEAR(samples[samples.size() / 2], 1000, 1);

    std::filesystem::remove(path);
}

TEST(WavReaderTests, RejectsNonWav) {
    const std::string path = (std::filesystem::temp_directory_path() / "es-wav-garbage.wav").string();
    std::ofstream(path) << "definitely not audio";

    std::vector<int16_t> samples;
    std::string error;
    EXPECT_FALSE(loadWavMono16(path, 44100, &samples, &error));
    EXPECT_FALSE(error.empty());

    std::filesystem::remove(path);
    EXPECT_FALSE(loadWavMono16("/nonexistent.wav", 44100, &samples, &error));
}

TEST(WavReaderTests, LoadsEveryShippedImpulseResponse) {
    const std::filesystem::path data = ENGINE_SIM_TEST_DATA_ROOT;
    int checked = 0;
    for (const auto &entry : std::filesystem::recursive_directory_iterator(data)) {
        if (entry.path().extension() != ".wav") continue;

        std::vector<int16_t> samples;
        std::string error;
        EXPECT_TRUE(loadWavMono16(entry.path().string(), 44100, &samples, &error)) << error;
        EXPECT_FALSE(samples.empty()) << entry.path();
        ++checked;
    }
    EXPECT_GT(checked, 50);
}
