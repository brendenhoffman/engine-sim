#include "synthesizer.h"

#include "utilities.h"

#include <cassert>
#include <cmath>
#include <algorithm>

#undef min
#undef max

Synthesizer::Synthesizer() {
    m_inputChannels = nullptr;
    m_inputChannelCount = 0;
    m_inputBufferSize = 0;
    m_inputWriteOffset = 0.0;
    m_inputSamplesRead = 0;

    m_audioBufferSize = 0;

    m_inputSampleRate = 0.0;
    m_audioSampleRate = 0.0;

    m_lastInputSampleOffset = 0.0;

    m_run = true;
    m_thread = nullptr;
    m_filters = nullptr;
}

Synthesizer::~Synthesizer() {
    assert(m_inputChannels == nullptr);
    assert(m_thread == nullptr);
    assert(m_filters == nullptr);
}

void Synthesizer::Resonator::set(float frequency, float q, float sampleRate) {
    const float w0 = 2.0f * 3.14159265f * std::min(frequency, 0.45f * sampleRate) / sampleRate;
    const float alpha = std::sin(w0) / (2 * q);
    const float a0 = 1 + alpha;
    b0 = alpha / a0;
    a1 = -2 * std::cos(w0) / a0;
    a2 = (1 - alpha) / a0;
}

void Synthesizer::initialize(const Parameters &p) {
    m_soundChannelCount = p.inputChannelCount;
    m_voices.assign(p.voiceCount, Voice());
    m_inputChannelCount = p.inputChannelCount + VoiceInputs * p.voiceCount;
    m_inputBufferSize = p.inputBufferSize;
    m_inputWriteOffset = p.inputBufferSize;
    m_audioBufferSize = p.audioBufferSize;
    m_inputSampleRate = p.inputSampleRate;
    m_audioSampleRate = p.audioSampleRate;
    m_audioParameters = p.initialAudioParameters;

    m_inputSamplesRead = 0;

    m_inputWriteOffset = 0;
    m_processed = true;

    m_audioBuffer.initialize(p.audioBufferSize);
    m_inputChannels = new InputChannel[m_inputChannelCount];
    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_inputChannels[i].transferBuffer = new float[p.inputBufferSize];
        m_inputChannels[i].data.initialize(p.inputBufferSize);
    }

    m_filters = new ProcessingFilters[m_inputChannelCount];
    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_filters[i].airNoiseLowPass.setCutoffFrequency(
            m_audioParameters.airNoiseFrequencyCutoff, m_audioSampleRate);

        m_filters[i].derivative.m_dt = 1 / m_audioSampleRate;

        m_filters[i].inputDcFilter.setCutoffFrequency(10.0);
        m_filters[i].inputDcFilter.m_dt = 1 / m_audioSampleRate;

        m_filters[i].jitterFilter.initialize(
            10,
            m_audioParameters.inputSampleNoiseFrequencyCutoff,
            m_audioSampleRate);

        m_filters[i].antialiasing.setCutoffFrequency(1900.0f, m_audioSampleRate);
    }

    m_levelingFilter.p_target = m_audioParameters.levelerTarget;
    m_levelingFilter.p_maxLevel = m_audioParameters.levelerMaxGain;
    m_levelingFilter.p_minLevel = m_audioParameters.levelerMinGain;
    m_antialiasing.setCutoffFrequency(m_audioSampleRate * 0.45f, m_audioSampleRate);

    for (int i = 0; i < m_audioBufferSize; ++i) {
        m_audioBuffer.write(0);
    }
}

void Synthesizer::initializeImpulseResponse(
    const int16_t *impulseResponse,
    unsigned int samples,
    float volume,
    int index)
{
    unsigned int clippedLength = 0;
    for (unsigned int i = 0; i < samples; ++i) {
        if (std::abs(impulseResponse[i]) > 100) {
            clippedLength = i + 1;
        }
    }

    const unsigned int sampleCount = std::min(10000U, clippedLength);
    m_filters[index].convolution.initialize(sampleCount);
    for (unsigned int i = 0; i < sampleCount; ++i) {
        m_filters[index].convolution.getImpulseResponse()[i] =
            volume * impulseResponse[i] / INT16_MAX;
    }
}

void Synthesizer::startAudioRenderingThread() {
    m_run = true;
    m_thread = new std::thread(&Synthesizer::audioRenderingThread, this);
}

void Synthesizer::endAudioRenderingThread() {
    if (m_thread != nullptr) {
        m_run = false;
        endInputBlock();

        m_thread->join();
        delete m_thread;

        m_thread = nullptr;
    }
}

void Synthesizer::destroy() {
    m_audioBuffer.destroy();

    for (int i = 0; i < m_inputChannelCount; ++i) {
        delete[] m_inputChannels[i].transferBuffer;
        m_inputChannels[i].data.destroy();
        m_filters[i].convolution.destroy();
    }

    delete[] m_inputChannels;
    delete[] m_filters;

    m_inputChannels = nullptr;
    m_filters = nullptr;

    m_inputChannelCount = 0;
}

int Synthesizer::readAudioOutput(int samples, int16_t *buffer) {
    std::lock_guard<std::mutex> lock(m_lock0);

    const int newDataLength = m_audioBuffer.size();
    if (newDataLength >= samples) {
        m_audioBuffer.readAndRemove(samples, buffer);
    }
    else {
        m_audioBuffer.readAndRemove(newDataLength, buffer);
        memset(
            buffer + newDataLength,
            0,
            sizeof(int16_t) * ((size_t)samples - newDataLength));
    }
    
    const int samplesConsumed = std::min(samples, newDataLength);

    return samplesConsumed;
}

void Synthesizer::waitProcessed() {
    {
        std::unique_lock<std::mutex> lk(m_lock0);
        m_cv0.wait(lk, [this] { return m_processed; });
    }
}

void Synthesizer::writeInput(const double *data) {
    std::lock_guard<std::mutex> lock(m_lock0);

    m_inputWriteOffset += (double)m_audioSampleRate / m_inputSampleRate;
    if (m_inputWriteOffset >= (double)m_inputBufferSize) {
        m_inputWriteOffset -= (double)m_inputBufferSize;
    }

    for (int i = 0; i < m_inputChannelCount; ++i) {
        RingBuffer<float> &buffer = m_inputChannels[i].data;
        const double lastInputSample = m_inputChannels[i].lastInputSample;
        const size_t baseIndex = buffer.writeIndex();
        const double distance =
            inputDistance(m_inputWriteOffset, m_lastInputSampleOffset);
        double s =
            inputDistance(baseIndex, m_lastInputSampleOffset);
        for (; s <= distance; s += 1.0) {
            if (s >= m_inputBufferSize) s -= m_inputBufferSize;

            const double f = s / distance;
            const double sample = lastInputSample * (1 - f) + data[i] * f;

            buffer.write(m_filters[i].antialiasing.fast_f(static_cast<float>(sample)));
        }

        m_inputChannels[i].lastInputSample = data[i];
    }

    m_lastInputSampleOffset = m_inputWriteOffset;
}

void Synthesizer::endInputBlock() {
    std::unique_lock<std::mutex> lk(m_lock0);

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_inputChannels[i].data.removeBeginning(m_inputSamplesRead);
    }

    if (m_inputChannelCount != 0) {
        m_latency = m_inputChannels[0].data.size();
    }
    
    m_inputSamplesRead = 0;
    m_processed = false;

    lk.unlock();
    m_cv0.notify_one();
}

void Synthesizer::audioRenderingThread() {
    while (m_run) {
        renderAudio();
    }
}

#undef max
void Synthesizer::renderAudio() {
    std::unique_lock<std::mutex> lk0(m_lock0);

    m_cv0.wait(lk0, [this] {
        const bool inputAvailable =
            m_inputChannels[0].data.size() > 0
            && m_audioBuffer.size() < 2000;
        return !m_run || (inputAvailable && !m_processed);
    });

    const int n = std::min(
        std::max(0, 2000 - (int)m_audioBuffer.size()),
        (int)m_inputChannels[0].data.size());

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_inputChannels[i].data.read(n, m_inputChannels[i].transferBuffer);
    }
    
    m_inputSamplesRead = n;
    m_processed = true;

    // Parameters can change from other threads; render this block with a consistent copy
    const AudioParameters params = m_audioParameters;

    lk0.unlock();

    for (int i = 0; i < m_inputChannelCount; ++i) {
        m_filters[i].airNoiseLowPass.setCutoffFrequency(
            static_cast<float>(params.airNoiseFrequencyCutoff), m_audioSampleRate);
        m_filters[i].jitterFilter.setJitterScale(params.inputSampleNoise);
    }
    m_levelingFilter.p_peakDecay = m_levelerHold
        ? 1.0f
        : static_cast<float>(std::exp(-1.0 / (std::max(params.levelerRelease, 0.001f) * m_audioSampleRate)));

    m_renderScratch.resize(n);
    for (int i = 0; i < n; ++i) {
        m_renderScratch[i] = renderAudio(i, params);
    }

    // The output buffer is drained by readAudioOutput() on another thread
    lk0.lock();
    for (int i = 0; i < n; ++i) {
        m_audioBuffer.write(m_renderScratch[i]);
    }
    m_levelerAttenuation = m_levelingFilter.getAttenuation();
    lk0.unlock();

    m_cv0.notify_one();
}

double Synthesizer::getLatency() const {
    return (double)m_latency / m_audioSampleRate;
}

int Synthesizer::inputDelta(int s1, int s0) const {
    return (s1 < s0)
        ? m_inputBufferSize - s0 + s1
        : s1 - s0;
}

double Synthesizer::inputDistance(double s1, double s0) const {
    return (s1 < s0)
        ? (double)m_inputBufferSize - s0 + s1
        : s1 - s0;
}

void Synthesizer::setInputSampleRate(double sampleRate) {
    if (sampleRate != m_inputSampleRate) {
        std::lock_guard<std::mutex> lock(m_lock0);
        m_inputSampleRate = sampleRate;
    }
}

int16_t Synthesizer::renderAudio(int inputSample, const AudioParameters &params) {
    const float airNoise = params.airNoise;
    const float dF_F_mix = params.dF_F_mix;
    const float convAmount = params.convolution;

    float signal = 0;
    for (int i = 0; i < m_soundChannelCount; ++i) {
        const float r_0 = 2.0 * ((double)rand() / RAND_MAX) - 1.0;

        const float jitteredSample =
            m_filters[i].jitterFilter.fast_f(m_inputChannels[i].transferBuffer[inputSample]);

        const float f_in = jitteredSample;
        const float f_dc = m_filters[i].inputDcFilter.fast_f(f_in);
        const float f = f_in - f_dc;
        const float f_p = m_filters[i].derivative.f(f_in);

        const float noise = 2.0 * ((double)rand() / RAND_MAX) - 1.0;
        const float r =
            m_filters->airNoiseLowPass.fast_f(noise);
        const float r_mixed =
            airNoise * r + (1 - airNoise);

        float v_in =
            f_p * dF_F_mix
            + f * r_mixed * (1 - dF_F_mix);
        if (std::fpclassify(v_in) == FP_SUBNORMAL) {
            v_in = 0;
        }

        const float v =
            convAmount * m_filters[i].convolution.f(v_in)
            + (1 - convAmount) * v_in;

        signal += v;
    }

    signal = m_antialiasing.fast_f(signal);

    m_levelingFilter.p_target = params.levelerTarget;
    float v_leveled = m_levelingFilter.f(signal) * params.volume;
    if (!m_voices.empty()) {
        // After the leveler: as loud on every engine, and not pumped by it
        v_leveled += params.volume * params.turboVolume * params.levelerTarget * renderVoices(inputSample);
    }
    int r_int = std::lround(v_leveled);
    if (r_int > INT16_MAX) {
        r_int = INT16_MAX;
    }
    else if (r_int < INT16_MIN) {
        r_int = INT16_MIN;
    }

    return static_cast<int16_t>(r_int);
}

float Synthesizer::renderVoices(int inputSample) {
    const float sr = m_audioSampleRate;
    const float twoPi = 6.28318530718f;
    float out = 0;
    for (size_t v = 0; v < m_voices.size(); ++v) {
        Voice &voice = m_voices[v];
        const float *in[VoiceInputs];
        for (int k = 0; k < VoiceInputs; ++k) {
            in[k] = m_inputChannels[m_soundChannelCount + VoiceInputs * v + k].transferBuffer;
        }
        const float frequency = std::max(0.0f, in[0][inputSample]);
        const float whine = std::max(0.0f, in[1][inputSample]);
        const float hiss = std::max(0.0f, in[2][inputSample]);
        const float flow = in[3][inputSample];
        const float bov = std::max(0.0f, in[4][inputSample]);
        const float wastegate = std::max(0.0f, in[5][inputSample]);
        const float gearFrequency = std::max(0.0f, in[6][inputSample]);
        const float gear = std::max(0.0f, in[7][inputSample]);

        voice.smoothedFrequency += 0.002f * (frequency - voice.smoothedFrequency);
        voice.smoothedGear += 0.002f * (gearFrequency - voice.smoothedGear);
        voice.drift += 0.0005f * (noise() - voice.drift);
        const float pitch = std::min(voice.smoothedFrequency, 0.45f * sr) * (1 + 0.004f * voice.drift);
        if (voice.counter-- <= 0) {
            voice.counter = 32;
            voice.hiss.set(std::clamp(0.5f * pitch, 600.0f, 4000.0f), 0.8f, sr);
            voice.bov.set(2400.0f, 0.7f, sr);
            voice.bovLow.set(700.0f, 0.9f, sr);
            voice.wastegate.set(1300.0f, 0.6f, sr);
            voice.chatter.set(500.0f, 1.0f, sr);
        }

        // Whine: the impeller's tone. Harmonics stop short of the top of the audio band (past
        // it they fold back down as harsh, warbling aliases), and very high whistles are
        // softened, as they are by the time they reach you.
        voice.phase += twoPi * pitch / sr;
        if (voice.phase > twoPi) voice.phase -= twoPi;
        const float ph = static_cast<float>(voice.phase);
        const float nyquist = 0.45f * sr;
        float tone = std::sin(ph);
        if (2 * pitch < nyquist) tone += 0.25f * std::sin(2 * ph);
        if (3 * pitch < nyquist) tone += 0.06f * std::sin(3 * ph);
        const float tilt = 1 / (1 + (pitch / 6000.0f) * (pitch / 6000.0f));
        out += 0.15f * whine * tilt * tone;

        // Gears driving a supercharger
        if (gear > 0) {
            const float gp = std::min(voice.smoothedGear, nyquist);
            voice.gearPhase += twoPi * gp / sr;
            if (voice.gearPhase > twoPi) voice.gearPhase -= twoPi;
            const float g = static_cast<float>(voice.gearPhase);
            const float gtilt = 1 / (1 + (gp / 6000.0f) * (gp / 6000.0f));
            out += 0.1f * gear * gtilt * (std::sin(g) + (2 * gp < nyquist ? 0.4f * std::sin(2 * g) : 0.0f));
        }

        // Air rushing through the compressor, its top end rolled off
        const float rush = voice.hiss.f(noise());
        voice.hissLow += 0.5f * (rush - voice.hissLow);
        out += 0.25f * hiss * voice.hissLow;

        // Surge: the compressor's flow swinging back and forth about its average. The swing
        // itself is the low thump, and it chatters while it lasts.
        voice.slowFlow += (twoPi * 6.0f / sr) * (flow - voice.slowFlow);
        const float swing = flow - voice.slowFlow;
        voice.surge += (twoPi * 20.0f / sr) * (std::abs(swing) - voice.surge);
        out += voice.surge * 3.0f * voice.chatter.f(noise()) + 0.8f * swing;

        // Blow-off valve and wastegate: venting gas
        const float n = noise();
        out += bov * (0.35f * voice.bov.f(n) + 0.15f * voice.bovLow.f(n));
        out += 0.3f * wastegate * voice.wastegate.f(noise());
    }
    return out;
}

double Synthesizer::getLevelerGain() {
    std::lock_guard<std::mutex> lock(m_lock0);
    return m_levelerAttenuation;
}

Synthesizer::AudioParameters Synthesizer::getAudioParameters() {
    std::lock_guard<std::mutex> lock(m_lock0);
    return m_audioParameters;
}

void Synthesizer::setAudioParameters(const AudioParameters &params) {
    std::lock_guard<std::mutex> lock(m_lock0);
    m_audioParameters = params;
}
