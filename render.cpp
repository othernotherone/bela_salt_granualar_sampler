/*
 * Granular sampler for Bela Salt + Salt+ -- v03 panel map (v02 DSP), 2026-09-23
 * Drop this file into a Bela C++ project as its only render.cpp.
 *
 *              SALT (12HP)                             SALT+ (10HP)             
 * ┌────────────────────────────────────┐  ┌────────────────────────────────────┐
 * │                                    │  │                                    │
 * │    (CV1)            (CV2)          │  │    (CV5)            (CV6)          │
 * │   POSITION           SIZE          │  │    SPRAY            SPREAD         │
 * │  recorded region    10-500ms       │  │  pos random       stereo width     │
 * │                                    │  │                                    │
 * │    (CV3)            (CV4)          │  │    (CV7)            (CV8)          │
 * │   DENSITY            PITCH         │  │    SHAPE            DRY/WET        │
 * │   0.5-60Hz         +/-2 oct        │  │  attack/decay          mix         │
 * │                                    │  │                                    │
 * │    [BTN1]           [BTN2]         │  │    [BTN3]           [BTN4]         │
 * │   REC toggle      FREEZE toggle    │  │    unused        CLEAR (5ms fade)  │
 * │    (LED1)           (LED2)         │  │    (LED3)           (LED4)         │
 * │ red=rec/yel=frz    flash=grain     │  │  input level      output level     │
 * │   off=idle           red           │  │    red/yellow       red/yellow     │
 * │                                    │  │                                    │
 * │  o T1  o T2  o CV1  o CV2  o AUDIO │  │  o CV5  o CV6   o T3     o T4      │
 * │   IN    IN     IN     IN      IN L │  │    IN     IN     IN       IN       │
 * │  REC  FREEZE   POS    SIZE   record│  │  SPRAY  SPREAD  unused   CLEAR     │
 * │ gate  toggle  (cv)   (cv)   source │  │   (cv)   (cv)           5ms fade   │
 * │                                    │  │                                    │
 * │  o T1  o T2  o CV3  o CV4  o AUDIO │  │  o CV7  o CV8   o T3     o T4      │
 * │  OUT   OUT    IN     IN     OUT L  │  │    IN     IN    OUT      OUT       │
 * │ unused unused DENS  PITCH    left  │  │  SHAPE  D/WET  unused   unused     │
 * │               (cv)   (cv)   audio  │  │   (cv)   (cv)                      │
 * │                                    │  │                                    │
 * │ [USB-B]  o CV1   o CV2    o AUDIO  │  │  o CV5   o CV6       [USB-A]       │
 * │ laptop    OUT     OUT     OUT R    │  │   OUT     OUT        MIDI host*    │
 * │          unused  unused   right    │  │  unused  unused                    │
 * │                          audio     │  │                                    │
 * │          o CV3   o CV4             │  │          o CV7   o CV8             │
 * │           OUT     OUT              │  │           OUT     OUT              │
 * │          unused  unused            │  │          unused  unused            │
 * │                                    │  │                                    │
 * │  BELA & REBELTECHNOLOGY            │  │                                    │
 * └────────────────────────────────────┘  └────────────────────────────────────┘
 *
 * Signal flow: Audio In L -> Record Buffer (10s) -> Grain Cloud (32 voices)
 *              Stereo dry input + stereo grains -> Dry/Wet -> Audio Out L/R
 *
 * Panel notes: buttons 2-4 share their corresponding trigger inputs.
 *              T1 gate is additional to the button-1 recording latch.
 *              Unused trigger outs are LOW; unused CV outs are 0 V.
 *              * USB MIDI is not used by this patch.
 *
 * CV/knob: 1 position, 2 size (10-500 ms), 3 density (0.5-60 Hz),
 *          4 pitch (+/-2 octaves), 5 spray, 6 stereo spread,
 *          7 envelope shape, 8 dry/wet.
 * Buttons: 1 record latch, 2 freeze, 3 unused, 4 clear (5 ms fade).
 * Triggers: T1 record gate; T2 freeze toggle; T3 unused; T4 clear.
 * Salt physically shares buttons 2-4 with trigger inputs 2-4.
 * Recording = (button-1 latch OR T1 gate), unless frozen or clearing.
 * Freeze pauses recording; unfreezing resumes an active latch/gate.
 * Clear resets the record latch and requires any T1 gate held during the
 * fade to go low before it can record again. Button events during clearing
 * are ignored.
 *
 * LED1: red recording, yellow frozen, otherwise off.
 * LED2: red grain flash. LED3/4: input/output level, yellow when hot.
 * Record source: left input; wet output: stereo; dry path: stereo.
 * Buffer: 10 seconds, 32 grains. Unused CV outputs are 0 V;
 * unused trigger outputs are low. Requires Salt hardware configuration,
 * 8 analog inputs, 16 digital channels, interleaved stereo audio.
 * Use short taps on button 1: Salt may reserve a long press for stop.
 *
 * All mutable DSP state belongs to render(). No background clearing,
 * memory allocation, or console output occurs in the audio callback.
 */

#include <Bela.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>

namespace {

constexpr float kBufferSeconds = 10.0f;
constexpr unsigned kMaxGrains = 32;
constexpr float kMinGrainMs = 10.0f;
constexpr float kMaxGrainMs = 500.0f;
constexpr float kMinDensityHz = 0.5f;
constexpr float kMaxDensityHz = 60.0f;
constexpr float kWetGain = 1.0f; // Lower for cleaner dense clouds, e.g. 0.25f.
constexpr float kPi = 3.14159265358979323846f;
constexpr unsigned kEnvelopeSteps = 256;
constexpr unsigned kMeterInterval = 512;
constexpr unsigned kFractionBits = 24;
constexpr uint32_t kFractionOne = 1u << kFractionBits;
constexpr uint32_t kFractionMask = kFractionOne - 1;
constexpr float kFractionScale = 1.0f / static_cast<float>(kFractionOne);

constexpr int kRecordButton = 6;
constexpr int kRecordGate = 15;
constexpr int kFreezeButton = 14;
constexpr int kUnusedButton = 1;
constexpr int kClearButton = 3;
constexpr int kLedPwm = 7;
constexpr int kLedPins[] = {2, 4, 8, 9};
constexpr int kTriggerOutPins[] = {0, 5, 12, 13};

enum LedColor { kOff, kRed, kYellow };

struct Grain {
    bool active;
    unsigned index;
    unsigned loopLength; // Valid physical region captured at grain creation.
    uint32_t fraction;
    uint32_t step;       // Q8.24 samples per audio sample; range 0.25-4.
    unsigned age;
    unsigned length;
    unsigned attackSamples;
    float attackScale;
    float decayScale;
    float panL;
    float panR;
};

struct SmoothedParam {
    float current;
    float target;
};

// Accept even a one-sample trigger immediately, then require a continuous
// low interval before rearming. This rejects ordinary switch bounce without
// imposing a minimum high-pulse width on Salt's shared button/trigger inputs.
struct PressDetector {
    bool armed;
    unsigned lowSamples;

    bool process(bool high, unsigned rearmSamples)
    {
        if (high) {
            lowSamples = 0;
            if (armed) {
                armed = false;
                return true;
            }
        } else if (!armed && ++lowSamples >= rearmSamples) {
            armed = true;
        }
        return false;
    }
};

float* gBuffer = nullptr;
unsigned gBufferLength = 0;
unsigned gWriteHead = 0;
unsigned gValidLength = 0;
Grain gGrains[kMaxGrains] = {};
SmoothedParam gParams[8] = {};
float gEnvelope[kEnvelopeSteps + 1] = {};
PressDetector gRecordPress = {true, 0};
PressDetector gFreezePress = {true, 0};
PressDetector gClearPress = {true, 0};

float gSampleRate = 44100.0f;
float gSmoothCoefficient = 0.002f;
float gDcPole = 0.995f;
float gDensityLogRatio = 0.0f;
unsigned gAnalogRatio = 2;
unsigned gMinGrainSamples = 441;
unsigned gRearmSamples = 353;
unsigned gClearFadeSamples = 221;
float gClearFadeScale = 1.0f / 221.0f;
unsigned gFlashSamples = 662;
bool gControlsInitialized = false;
bool gRecordLatched = false;
bool gGateBlocked = false;
bool gFrozen = false;
unsigned gClearRemaining = 0;
float gSpawnPhase = 0.0f;
unsigned gGrainFlash = 0;
uint32_t gRandomState = 123456789u;
float gDcX1L = 0.0f, gDcY1L = 0.0f;
float gDcX1R = 0.0f, gDcY1R = 0.0f;
float gInputPeak = 0.0f, gOutputPeak = 0.0f;
unsigned gMeterCounter = 0;
unsigned gPwmCounter = 0;
LedColor gInputLed = kOff, gOutputLed = kOff;

inline float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

inline float randomFloat()
{
    gRandomState = gRandomState * 1664525u + 1013904223u;
    return static_cast<float>(gRandomState >> 8) * (1.0f / 16777216.0f);
}

inline float envelopeRamp(float tablePosition)
{
    if (tablePosition >= static_cast<float>(kEnvelopeSteps))
        return 1.0f;
    const unsigned index = static_cast<unsigned>(tablePosition);
    const float fraction = tablePosition - static_cast<float>(index);
    return gEnvelope[index] + fraction * (gEnvelope[index + 1] - gEnvelope[index]);
}

inline float grainEnvelope(const Grain& grain)
{
    if (grain.age <= grain.attackSamples)
        return envelopeRamp(static_cast<float>(grain.age) * grain.attackScale);
    return envelopeRamp(static_cast<float>(grain.length - 1 - grain.age) * grain.decayScale);
}

inline float readGrain(const Grain& grain)
{
    unsigned next = grain.index + 1;
    if (next == grain.loopLength)
        next = 0;
    const float a = gBuffer[grain.index];
    const float b = gBuffer[next];
    return a + (b - a) * (static_cast<float>(grain.fraction) * kFractionScale);
}

inline void advanceGrain(Grain& grain)
{
    // The fraction never grows with the absolute buffer address. Integer
    // addition preserves sub-sample pitch precision throughout the buffer.
    const uint32_t position = grain.fraction + grain.step;
    grain.fraction = position & kFractionMask;
    grain.index += position >> kFractionBits;
    // A valid recording is at least 10 ms, much longer than the max 4-sample step.
    if (grain.index >= grain.loopLength)
        grain.index -= grain.loopLength;
    if (++grain.age >= grain.length)
        grain.active = false;
}

inline float dcBlock(float input, float& previousInput, float& previousOutput)
{
    float output = input - previousInput + gDcPole * previousOutput;
    // Stop a silent filter tail before it reaches costly subnormal values.
    if (fabsf(output) < 1.0e-20f)
        output = 0.0f;
    previousInput = input;
    previousOutput = output;
    return output;
}

inline float softClip(float value)
{
    if (value >= 1.5f) return 1.0f;
    if (value <= -1.5f) return -1.0f;
    return value - value * value * value * (1.0f / 6.75f);
}

void readControls(BelaContext* context, unsigned frame)
{
    for (unsigned channel = 0; channel < 8; ++channel) {
        const float value = analogRead(context, frame, channel);
        // This also handles an unexpected NaN by falling back to the midpoint.
        gParams[channel].target = (value == value) ? clampf(value, 0.0f, 1.0f) : 0.5f;
        if (!gControlsInitialized)
            gParams[channel].current = gParams[channel].target;
    }
    gControlsInitialized = true;
}

void startGrain()
{
    if (gValidLength < gMinGrainSamples)
        return;
    Grain* available = nullptr;
    for (unsigned i = 0; i < kMaxGrains; ++i) {
        if (!gGrains[i].active) {
            available = &gGrains[i];
            break;
        }
    }
    if (!available)
        return; // Drop a spawn instead of stealing an audible grain.

    Grain& grain = *available;
    float position = gParams[0].current
        + (randomFloat() - 0.5f) * gParams[4].current;
    if (position < 0.0f) position += 1.0f;
    if (position >= 1.0f) position -= 1.0f;

    // Double is used only on spawn, not in the sample loop, to split the
    // initial address without throwing away fractional position precision.
    const double offset = static_cast<double>(position) * gValidLength;
    const unsigned logicalIndex = static_cast<unsigned>(offset);
    grain.fraction = static_cast<uint32_t>((offset - logicalIndex) * kFractionOne);
    grain.loopLength = gValidLength;
    // The oldest valid sample is gWriteHead once full, including when frozen.
    const unsigned origin = (gValidLength == gBufferLength) ? gWriteHead : 0;
    grain.index = logicalIndex + origin;
    if (grain.index >= grain.loopLength)
        grain.index -= grain.loopLength;

    const float sizeMs = kMinGrainMs + gParams[1].current * (kMaxGrainMs - kMinGrainMs);
    grain.length = static_cast<unsigned>(sizeMs * (0.001f * gSampleRate) + 0.5f);
    if (grain.length < 3) grain.length = 3;
    grain.age = 0;
    const float attack = clampf(gParams[6].current, 0.001f, 0.999f);
    grain.attackSamples = static_cast<unsigned>(attack * (grain.length - 1) + 0.5f);
    if (grain.attackSamples < 1) grain.attackSamples = 1;
    if (grain.attackSamples > grain.length - 2) grain.attackSamples = grain.length - 2;
    grain.attackScale = static_cast<float>(kEnvelopeSteps) / grain.attackSamples;
    grain.decayScale = static_cast<float>(kEnvelopeSteps) / (grain.length - 1 - grain.attackSamples);

    const float rate = expf((gParams[3].current - 0.5f) * (4.0f * 0.69314718056f));
    grain.step = static_cast<uint32_t>(static_cast<double>(rate) * kFractionOne + 0.5);
    const float pan = clampf(0.5f + (randomFloat() - 0.5f) * gParams[5].current, 0.0f, 1.0f);
    const float angle = pan * (0.5f * kPi);
    grain.panL = cosf(angle);
    grain.panR = sinf(angle);
    grain.active = true;
    gGrainFlash = gFlashSamples;
}

void beginClear(bool gateHigh)
{
    if (gClearRemaining)
        return;
    gRecordLatched = false;
    gFrozen = false;
    gGateBlocked = gateHigh;
    gClearRemaining = gClearFadeSamples;
    gSpawnPhase = 0.0f;
    gGrainFlash = 0;
}

void finishClear()
{
    // Old memory is inaccessible: new grains may only read the valid prefix,
    // and every sample in that prefix is overwritten before it becomes valid.
    for (unsigned i = 0; i < kMaxGrains; ++i)
        gGrains[i].active = false;
    gWriteHead = 0;
    gValidLength = 0;
    gSpawnPhase = 0.0f;
    gDcX1L = gDcY1L = gDcX1R = gDcY1R = 0.0f;
}

inline void writeLed(BelaContext* context, unsigned frame, int pin, LedColor color)
{
    // Once calls are correct here because the cached state is applied to EVERY frame.
    pinModeOnce(context, frame, pin, color == kOff ? INPUT : OUTPUT);
    digitalWriteOnce(context, frame, pin, color == kYellow ? 1 : 0);
}

} // namespace

bool setup(BelaContext* context, void*)
{
    const bool audioOk = context->audioInChannels == 2 && context->audioOutChannels == 2
        && context->audioFrames > 0 && (context->flags & BELA_FLAG_INTERLEAVED)
        && context->audioSampleRate >= 8000.0f && context->audioSampleRate <= 192000.0f;
    const bool analogOk = context->analogInChannels == 8 && context->analogFrames > 0
        && (context->audioFrames == context->analogFrames
            || context->audioFrames == 2 * context->analogFrames);
    const bool digitalOk = context->digitalChannels == 16
        && context->digitalFrames == context->audioFrames;
    if (!audioOk || !analogOk || !digitalOk) {
        rt_printf("Salt granular: enable interleaved stereo audio, 8 analog inputs, and 16 digital channels.\n");
        rt_printf("Analog frames must equal audio frames or half of them; digital frames must equal audio frames.\n");
        return false;
    }

    gSampleRate = context->audioSampleRate;
    gBufferLength = static_cast<unsigned>(gSampleRate * kBufferSeconds);
    gBuffer = new(std::nothrow) float[gBufferLength];
    if (!gBuffer) {
        rt_printf("Salt granular: could not allocate the recording buffer.\n");
        return false;
    }
    memset(gBuffer, 0, gBufferLength * sizeof(float));
    gAnalogRatio = context->audioFrames / context->analogFrames; // Setup only.
    gMinGrainSamples = static_cast<unsigned>(gSampleRate * kMinGrainMs * 0.001f + 0.5f);
    gRearmSamples = static_cast<unsigned>(gSampleRate * 0.008f + 0.5f);
    gClearFadeSamples = static_cast<unsigned>(gSampleRate * 0.005f + 0.5f);
    gClearFadeScale = 1.0f / gClearFadeSamples;
    gFlashSamples = static_cast<unsigned>(gSampleRate * 0.015f + 0.5f);
    gSmoothCoefficient = 1.0f - powf(1.0f - 0.002f, 44100.0f / gSampleRate);
    gDcPole = powf(0.995f, 44100.0f / gSampleRate);
    gDensityLogRatio = logf(kMaxDensityHz / kMinDensityHz);

    for (unsigned i = 0; i <= kEnvelopeSteps; ++i)
        gEnvelope[i] = 0.5f * (1.0f - cosf(kPi * static_cast<float>(i) / kEnvelopeSteps));
    gEnvelope[0] = 0.0f;
    gEnvelope[kEnvelopeSteps] = 1.0f;
    for (unsigned i = 0; i < kMaxGrains; ++i)
        gGrains[i] = Grain{};
    for (unsigned i = 0; i < 8; ++i)
        gParams[i] = SmoothedParam{0.5f, 0.5f};
    gRecordPress = gFreezePress = gClearPress = PressDetector{true, 0};
    gWriteHead = gValidLength = gClearRemaining = gGrainFlash = 0;
    gRecordLatched = gGateBlocked = gFrozen = gControlsInitialized = false;
    gSpawnPhase = 0.0f;
    gRandomState = 123456789u;
    gDcX1L = gDcY1L = gDcX1R = gDcY1R = 0.0f;
    gInputPeak = gOutputPeak = 0.0f;
    gMeterCounter = gPwmCounter = 0;
    gInputLed = gOutputLed = kOff;

    const int inputs[] = {kRecordButton, kRecordGate, kFreezeButton, kUnusedButton, kClearButton};
    for (int pin : inputs) pinMode(context, 0, pin, INPUT);
    for (int pin : kTriggerOutPins) pinMode(context, 0, pin, OUTPUT);
    for (int pin : kLedPins) pinMode(context, 0, pin, INPUT);
    pinMode(context, 0, kLedPwm, OUTPUT);

    rt_printf("Salt granular v02: %.1f-second buffer, %u grains. BTN1 record, BTN2 freeze, BTN4 clear.\n",
        kBufferSeconds, kMaxGrains);
    return true;
}

void render(BelaContext* context, void*)
{
    readControls(context, 0);
    const float density = kMinDensityHz * expf(gDensityLogRatio * gParams[2].current);
    const float spawnIncrement = density / gSampleRate;
    unsigned analogFrame = 0;
    unsigned nextAnalogSample = gAnalogRatio;

    // Unused Salt CV outputs: bipolar zero is the normalized midpoint.
    for (unsigned i = 0; i < context->analogFrames * context->analogOutChannels; ++i)
        context->analogOut[i] = 0.5f;

    for (unsigned n = 0; n < context->audioFrames; ++n) {
        if (n == nextAnalogSample) {
            readControls(context, ++analogFrame);
            nextAnalogSample += gAnalogRatio;
        }
        for (unsigned channel = 0; channel < 8; ++channel) {
            SmoothedParam& param = gParams[channel];
            const float difference = param.target - param.current;
            param.current = fabsf(difference) < 1.0e-7f
                ? param.target : param.current + gSmoothCoefficient * difference;
        }

        const bool gateHigh = digitalRead(context, n, kRecordGate) != 0;
        const bool recordPressed = gRecordPress.process(digitalRead(context, n, kRecordButton) != 0, gRearmSamples);
        const bool freezePressed = gFreezePress.process(digitalRead(context, n, kFreezeButton) != 0, gRearmSamples);
        const bool clearPressed = gClearPress.process(digitalRead(context, n, kClearButton) != 0, gRearmSamples);
        if (!gateHigh) gGateBlocked = false;
        if (gClearRemaining && gateHigh) gGateBlocked = true;
        if (clearPressed) {
            beginClear(gateHigh);
        } else if (!gClearRemaining) {
            if (recordPressed) gRecordLatched = !gRecordLatched;
            if (freezePressed) gFrozen = !gFrozen;
        }
        const bool recording = !gFrozen && !gClearRemaining
            && (gRecordLatched || (gateHigh && !gGateBlocked));

        const float inputL = audioRead(context, n, 0);
        const float inputR = audioRead(context, n, 1);
        if (recording) {
            gBuffer[gWriteHead] = inputL;
            if (++gWriteHead == gBufferLength) gWriteHead = 0;
            if (gValidLength < gBufferLength) ++gValidLength;
        }
        if (!gClearRemaining && gValidLength >= gMinGrainSamples) {
            gSpawnPhase += spawnIncrement;
            if (gSpawnPhase >= 1.0f) {
                gSpawnPhase -= 1.0f;
                startGrain();
            }
        }

        float outL = 0.0f, outR = 0.0f;
        for (unsigned i = 0; i < kMaxGrains; ++i) {
            Grain& grain = gGrains[i];
            if (!grain.active) continue;
            const float sample = readGrain(grain) * grainEnvelope(grain);
            outL += sample * grain.panL;
            outR += sample * grain.panR;
            advanceGrain(grain);
        }
        outL = dcBlock(outL, gDcX1L, gDcY1L);
        outR = dcBlock(outR, gDcX1R, gDcY1R);
        const float clearGain = gClearRemaining ? gClearRemaining * gClearFadeScale : 1.0f;
        const float wet = gParams[7].current;
        const float wetGain = wet * clearGain * kWetGain;
        const float dryGain = 1.0f - wet;
        const float mixL = softClip(inputL * dryGain + outL * wetGain);
        const float mixR = softClip(inputR * dryGain + outR * wetGain);
        audioWrite(context, n, 0, mixL);
        audioWrite(context, n, 1, mixR);

        const float inputLevel = fabsf(inputL);
        const float outputLevel = fabsf(mixL) + fabsf(mixR);
        if (inputLevel > gInputPeak) gInputPeak = inputLevel;
        if (outputLevel > gOutputPeak) gOutputPeak = outputLevel;
        if (++gMeterCounter >= kMeterInterval) {
            gInputLed = gInputPeak > 0.5f ? kYellow : (gInputPeak > 0.01f ? kRed : kOff);
            gOutputLed = gOutputPeak > 1.0f ? kYellow : (gOutputPeak > 0.02f ? kRed : kOff);
            gInputPeak = gOutputPeak = 0.0f;
            gMeterCounter = 0;
        }
        const LedColor status = gFrozen ? kYellow : (recording ? kRed : kOff);
        writeLed(context, n, kLedPins[0], status);
        writeLed(context, n, kLedPins[1], gGrainFlash ? kRed : kOff);
        writeLed(context, n, kLedPins[2], gInputLed);
        writeLed(context, n, kLedPins[3], gOutputLed);
        // Common 50% LED drive, continuous across audio blocks.
        digitalWriteOnce(context, n, kLedPwm, (gPwmCounter++ & 31u) < 16u);
        for (int pin : kTriggerOutPins) digitalWriteOnce(context, n, pin, 0);

        if (gGrainFlash) --gGrainFlash;
        if (gClearRemaining && --gClearRemaining == 0)
            finishClear();
    }
}

void cleanup(BelaContext*, void*)
{
    delete[] gBuffer;
    gBuffer = nullptr;
}
