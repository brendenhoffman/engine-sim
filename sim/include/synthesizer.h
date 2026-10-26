#ifndef ATG_ENGINE_SIM_ENGINE_SYNTHESIZER_H
#define ATG_ENGINE_SIM_ENGINE_SYNTHESIZER_H

#include "convolution_filter.h"
#include "leveling_filter.h"
#include "derivative_filter.h"
#include "low_pass_filter.h"
#include "jitter_filter.h"
#include "ring_buffer.h"
#include "butterworth_low_pass_filter.h"

#include <cinttypes>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <vector>

class Synthesizer {
    public:
        struct AudioParameters {
            float volume = 1.0f;
            float convolution = 1.0f;
            float dF_F_mix = 0.01f;
            float inputSampleNoise = 0.5f;
            float inputSampleNoiseFrequencyCutoff = 10000.0f;
            float airNoise = 1.0f;
            float airNoiseFrequencyCutoff = 2000.0f;
            float levelerTarget = 30000.0f;
            float levelerMaxGain = 1.9f;
            float levelerMinGain = 0.00001f;

            // How long the leveler remembers a loud peak (time constant, seconds). Upstream's
            // ~23 ms re-levels between firing pulses, so idle and full throttle end up about
            // equally loud; seconds keep revving louder than idle.
            float levelerRelease = 0.02266f;

            // Turbo and supercharger sounds, relative to the leveled engine
            float turboVolume = 1.0f;
        };

        // A turbo or supercharger's sounds, synthesized at the audio rate from what the
        // simulation says about it (VoiceInputs channels each, after the engine's channels)
        // Pitch (Hz), whine, hiss, flutter (signed flow), blow-off, wastegate, gear pitch (Hz), gear
        static constexpr int VoiceInputs = 8;

        struct Resonator {
            // Two-pole band-pass (RBJ, constant peak gain)
            float b0 = 0, a1 = 0, a2 = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
            void set(float frequency, float q, float sampleRate);
            float f(float x) {
                const float y = b0 * (x - x2) - a1 * y1 - a2 * y2;
                x2 = x1; x1 = x; y2 = y1; y1 = y;
                return y;
            }
        };

        struct Voice {
            double phase = 0, gearPhase = 0;
            float smoothedFrequency = 0, smoothedGear = 0;
            float drift = 0;            // Slow random pitch wander
            float slowFlow = 0;         // The compressor's flow, smoothed: surge is the swing about it
            float surge = 0;            // Envelope of that swing
            float hissLow = 0;          // One-pole low-pass state, to take the edge off the hiss
            int counter = 0;
            Resonator hiss, bov, bovLow, wastegate, chatter, whistle;
            float whistleGain = 1;      // Brings the narrow-band noise to about the tone's level
        };

        struct Parameters {
            int inputChannelCount = 1;
            int inputBufferSize = 1024;
            int audioBufferSize = 44100;
            float inputSampleRate = 10000;
            float audioSampleRate = 44100;
            int voiceCount = 0;
            // A channel for sound radiated by the engine block (combustion knock), mixed in
            // without the exhaust's filtering and impulse response; it follows the exhaust ones
            bool blockChannel = false;
            // Its ringing: 0 for a light metallic tick (about 1-3 kHz); a base frequency (Hz)
            // for a heavy iron block's low, slightly inharmonic modes (x1, 1.31, 2.64, 3.14)
            float knockResonance = 0;
            AudioParameters initialAudioParameters;
        };

        struct InputChannel {
            RingBuffer<float> data;
            float *transferBuffer = nullptr;
            double lastInputSample = 0.0f;
        };

        struct ProcessingFilters {
            ConvolutionFilter convolution;
            DerivativeFilter derivative;
            JitterFilter jitterFilter;
            ButterworthLowPassFilter<float> airNoiseLowPass;
            LowPassFilter inputDcFilter;
            ButterworthLowPassFilter<double> antialiasing;
        };

    public:
        Synthesizer();
        ~Synthesizer();

        void initialize(const Parameters &p);
        void initializeImpulseResponse(
            const int16_t *impulseResponse,
            unsigned int samples,
            float volume,
            int index);
        void startAudioRenderingThread();
        void endAudioRenderingThread();
        void destroy();

        int readAudioOutput(int samples, int16_t *buffer);

        void writeInput(const double *data);
        void endInputBlock();

        void waitProcessed();

        void audioRenderingThread();
        void renderAudio();

        double getLatency() const;

        int inputDelta(int s1, int s0) const;
        double inputDistance(double s1, double s0) const;

        void setInputSampleRate(double sampleRate);
        double getInputSampleRate() const { return m_inputSampleRate; }

        int16_t renderAudio(int inputOffset, const AudioParameters &params);

        double getLevelerGain();
        AudioParameters getAudioParameters();
        void setAudioParameters(const AudioParameters &params);

    //protected:
        ButterworthLowPassFilter<float> m_antialiasing;
        LevelingFilter m_levelingFilter;
        InputChannel *m_inputChannels;
        AudioParameters m_audioParameters;
        int m_inputChannelCount;
        int m_inputBufferSize;
        int m_inputSamplesRead;
        int m_latency;
        double m_inputWriteOffset;
        double m_lastInputSampleOffset;

        RingBuffer<int16_t> m_audioBuffer;
        int m_audioBufferSize;

        float m_inputSampleRate;
        float m_audioSampleRate;

        std::thread *m_thread;
        std::atomic<bool> m_run;
        bool m_processed;
        float m_levelerAttenuation = 1.0f;
        std::vector<int16_t> m_renderScratch;

        // Guards everything shared with the audio rendering thread: the input and output
        // buffers, m_processed/m_inputSamplesRead, the audio parameters and m_levelerAttenuation
        std::mutex m_lock0;
        std::condition_variable m_cv0;

        ProcessingFilters *m_filters;

        // Engine sound channels come first; the rest drive the voices
        int m_soundChannelCount = 0;
        int m_blockChannel = -1;
        float m_blockDc = 0;
        Resonator m_blockModes[4];
        float m_blockModeGain[4] = { 1, 1, 1, 0 };
        std::vector<Voice> m_voices;
        uint32_t m_noiseState = 0x12345678u;
        float noise() {
            m_noiseState ^= m_noiseState << 13;
            m_noiseState ^= m_noiseState >> 17;
            m_noiseState ^= m_noiseState << 5;
            return static_cast<float>(m_noiseState) * (2.0f / 4294967295.0f) - 1.0f;
        }
        float renderVoices(int inputSample);

    public:
        // Freezes the leveler while the engine is cranking or stopped, so the quiet between
        // cranking pulses isn't turned up into a scratchy hiss
        void setLevelerHold(bool hold) { m_levelerHold = hold; }
    private:
        std::atomic<bool> m_levelerHold{ false };
};

#endif /* ATG_ENGINE_SIM_ENGINE_SYNTHESIZER_H */
