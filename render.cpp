/*
 * Granular Sampler for Bela Salt + Salt+
 *
 * Records audio into a circular buffer, then spawns grains that read from it
 * with independent position, size, pitch, and panning. All parameters are
 * CV-controllable via the Salt/Salt+ panel.
 *
 *         SALT (12HP)                          SALT+ (10HP)
 * ┌──────────────────────────────────┐  ┌──────────────────────────────────┐
 * │                                  │  │                                  │
 * │    (CV1)          (CV2)          │  │    (CV5)          (CV6)          │
 * │   POSITION        SIZE           │  │    SPRAY          SPREAD         │
 * │   buf scan       10-500ms        │  │   pos random     stereo width    │
 * │                                  │  │                                  │
 * │    (CV3)          (CV4)          │  │    (CV7)          (CV8)          │
 * │   DENSITY         PITCH          │  │    SHAPE          DRY/WET        │
 * │   0.5-60Hz       +/-2 oct        │  │   env skew       mix             │
 * │                                  │  │                                  │
 * │        [BTN1]          [BTN2]    │  │        [BTN3]          [BTN4]    │
 * │        (LED1)          (LED2)    │  │        (LED3)          (LED4)    │
 * │       REC tog        FREEZE tog  │  │       in level       out level   │
 * │    red=rec yel=frz   flash=grain │  │       red/yel         red/yel    │
 * │                                  │  │                                  │
 * │   o T1  o T2  o CV1  o CV2  o    │  │   o CV5  o CV6  o T3   o T4      │
 * │    IN    IN    IN     IN   AUD   │  │    IN     IN     IN     IN       │
 * │                              IN  │  │                                  │
 * │  REC    ---   POS    SIZE  rec   │  │  SPRAY  SPREAD  ---    ---       │
 * │  GATE        (cv)   (cv)   src   │  │  (cv)   (cv)                     │
 * │                                  │  │                                  │
 * │   o T1  o T2  o CV3  o CV4  o    │  │   o CV7  o CV8  o T3   o T4      │
 * │   OUT   OUT    IN     IN  AUDIO  │  │    IN     IN    OUT    OUT       │
 * │                                  │  │                                  │
 * │  ---    ---   DENS   PTCH  grain │  │  SHAPE  D/WET  ---    ---        │
 * │               (cv)   (cv)  out   │  │  (cv)   (cv)                     │
 * │                                  │  │                                  │
 * │  [USB-B]  o CV1  o CV2  o        │  │   o CV5  o CV6   [USB-A]         │
 * │  laptop    OUT    OUT  AUD       │  │    OUT    OUT    MIDI host       │
 * │                         OUT      │  │                                  │
 * │           o CV3  o CV4           │  │   o CV7  o CV8                   │
 * │            OUT    OUT            │  │    OUT    OUT                    │
 * │                                  │  │                                  │
 * │  BELA & REBELTECHNOLOGY          │  │                                  │
 * └──────────────────────────────────┘  └──────────────────────────────────┘
 *
 * Signal Flow:  Audio In L ──► Record Buffer (10s) ──► Grain Cloud (32 voices)
 *                          ──► Dry/Wet Mix ──► Stereo Out L/R
 *
 * LED Status:   LED1: RED=recording, YELLOW=frozen, OFF=idle
 *               LED2: RED flash on grain spawn
 *               LED3: input level (RED=signal, YELLOW=hot)
 *               LED4: output level (RED=signal, YELLOW=hot)
 *
 * Knobs set base values for each CV parameter.
 * Patching CV adds modulation on top. All params smoothed.
 */

#include <Bela.h>
#include <cmath>
#include <cstring>
#include <new>

// ─── Configuration ───────────────────────────────────────────────────────────

static constexpr float BUFFER_SECONDS    = 10.0f;
static constexpr int   MAX_GRAINS        = 32;  // reduced from 64 for CPU headroom
static constexpr float MIN_GRAIN_MS      = 10.0f;
static constexpr float MAX_GRAIN_MS      = 500.0f;
static constexpr float MIN_DENSITY_HZ    = 0.5f;
static constexpr float MAX_DENSITY_HZ    = 60.0f;  // reduced from 100 for safety
static constexpr float SMOOTHING_COEFF   = 0.002f;

// ─── Salt Pin Mapping ────────────────────────────────────────────────────────

// Trigger inputs
static constexpr int PIN_TRIG_IN_1  = 15;
static constexpr int PIN_BUTTON_2   = 14;
static constexpr int PIN_BUTTON_3   = 1;
static constexpr int PIN_BUTTON_4   = 3;

// Trigger outputs
static constexpr int PIN_TRIG_OUT_1 = 0;
static constexpr int PIN_TRIG_OUT_2 = 5;
static constexpr int PIN_TRIG_OUT_3 = 12;
static constexpr int PIN_TRIG_OUT_4 = 13;

// LEDs
static constexpr int PIN_LED_1 = 2;
static constexpr int PIN_LED_2 = 4;
static constexpr int PIN_LED_3 = 8;
static constexpr int PIN_LED_4 = 9;

// ─── Data Structures ─────────────────────────────────────────────────────────

struct Grain {
	bool     active;
	float    bufferPos;      // fractional read position in the record buffer
	unsigned int grainSize;  // total grain length in samples
	float    phase;          // 0–1 progress through grain lifetime
	float    phaseInc;       // 1.0 / grainSize
	float    playbackRate;   // pitch ratio (1.0 = unity)
	float    panL;           // equal-power pan gains
	float    panR;
	float    envAttack;      // fraction of grain that is attack (0–1)
};

struct SmoothedParam {
	float current;
	float target;
	void update() { current += SMOOTHING_COEFF * (target - current); }
};

// ─── Global State ────────────────────────────────────────────────────────────

static float*       gBuffer       = nullptr;
static unsigned int gBufferLen    = 0;
static unsigned int gWriteHead    = 0;
static unsigned int gWrittenTotal = 0;  // total samples ever written

static Grain        gGrains[MAX_GRAINS];
static SmoothedParam gParams[8];

static bool  gRecording  = false;
static bool  gFrozen     = false;
static bool  gClearRequested = false;  // deferred clear (avoid memset in render)

// Edge detection for buttons/triggers
static bool  gPrevTrigIn1  = false;
static bool  gPrevButton2  = false;
static bool  gPrevButton3  = false;
static bool  gPrevButton4  = false;

static float gSpawnAccum = 0.0f;
static float gSpawnInterval = 44100.0f;  // cached, recalculated per block
static int   gGrainFlash = 0;   // LED2 flash countdown in samples
static float gSampleRate = 44100.0f;

// DC blocker state for output
static float gDCx1L = 0.0f, gDCy1L = 0.0f;
static float gDCx1R = 0.0f, gDCy1R = 0.0f;

// LED update decimation
static int gLEDCounter = 0;
static constexpr int LED_UPDATE_INTERVAL = 512;

// Precomputed log ratio for density mapping
static float gDensityLogRatio = 1.0f;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static uint32_t gRandSeed = 123456789;

inline float randomFloat() {
	gRandSeed = gRandSeed * 1664525u + 1013904223u;
	return (float)(gRandSeed >> 8) / 16777216.0f;
}

inline float clampf(float x, float lo, float hi) {
	return x < lo ? lo : (x > hi ? hi : x);
}

// DC blocker: y[n] = x[n] - x[n-1] + 0.995 * y[n-1]
inline float dcBlock(float in, float& x1, float& y1) {
	float out = in - x1 + 0.995f * y1;
	x1 = in;
	y1 = out;
	return out;
}

// Fast soft clip: cubic approximation (much cheaper than tanhf)
inline float softClip(float x) {
	if (x > 1.5f) return 1.0f;
	if (x < -1.5f) return -1.0f;
	return x - (x * x * x) / 6.75f;
}

// Fast cosine approximation (Bhaskara I, good to ~0.1% error)
// Input: phase 0–1 (maps to 0–2pi internally)
inline float fastCos01(float phase01) {
	// Map to -pi..pi range
	float x = phase01 * 2.0f - 1.0f;  // -1..1
	float x2 = x * x;
	// Attempt parabolic approximation for cos over -1..1 mapping to -pi..pi
	// cos(pi*x) ≈ 1 - (2*x)^2 * (1 - x^2/3) ... simpler: use polynomial
	// Actually let's use: cos(pi*x) ≈ 1 - 4.93480*x^2 + 4.05845*x^4 - 1.33519*x^6
	// But that's still expensive. Simpler parabolic:
	// cos(pi*x) ≈ 1 - x^2*(pi^2/2) + x^4*(pi^4/24) ... let's just do:
	float x4 = x2 * x2;
	return 1.0f - 4.9348f * x2 + 4.0585f * x4 - 1.3352f * x2 * x4;
}

// Skewed raised-cosine envelope using fast cosine
inline float grainEnvelope(float phase, float attack) {
	float a = clampf(attack, 0.001f, 0.999f);
	if (phase < a) {
		float p = phase / a;  // 0..1
		return 0.5f * (1.0f - fastCos01(p));
	} else {
		float p = (phase - a) / (1.0f - a);  // 0..1
		return 0.5f * (1.0f + fastCos01(p));
	}
}

// Read from the record buffer with linear interpolation
inline float bufferRead(float pos) {
	int   idx0 = (int)pos;
	int   idx1 = idx0 + 1;
	float frac = pos - (float)idx0;
	// Wrap indices
	if (idx0 >= (int)gBufferLen) idx0 -= gBufferLen;
	if (idx0 < 0)               idx0 += gBufferLen;
	if (idx1 >= (int)gBufferLen) idx1 -= gBufferLen;
	return gBuffer[idx0] * (1.0f - frac) + gBuffer[idx1] * frac;
}

// Background task to clear buffer (avoids memset in render)
void clearBufferTask(void*) {
	memset(gBuffer, 0, gBufferLen * sizeof(float));
	gWriteHead    = 0;
	gWrittenTotal = 0;
	rt_printf("Buffer cleared\n");
}

static AuxiliaryTask gClearTask;

// ─── Bela Callbacks ──────────────────────────────────────────────────────────

bool setup(BelaContext* context, void* userData)
{
	gSampleRate = context->audioSampleRate;
	gBufferLen  = (unsigned int)(gSampleRate * BUFFER_SECONDS);
	gBuffer     = new(std::nothrow) float[gBufferLen];
	if (!gBuffer) {
		rt_printf("Error: could not allocate %u-sample record buffer\n", gBufferLen);
		return false;
	}
	memset(gBuffer, 0, gBufferLen * sizeof(float));

	// Init grains
	for (int i = 0; i < MAX_GRAINS; i++)
		gGrains[i].active = false;

	// Init smoothed params to midpoint
	for (int i = 0; i < 8; i++) {
		gParams[i].current = 0.5f;
		gParams[i].target  = 0.5f;
	}

	// Precompute density mapping constant
	gDensityLogRatio = logf(MAX_DENSITY_HZ / MIN_DENSITY_HZ);

	// Create background task for buffer clearing
	gClearTask = Bela_createAuxiliaryTask(clearBufferTask, 50, "clear-buffer");

	// Configure digital pins
	int inputs[]  = { PIN_TRIG_IN_1, PIN_BUTTON_2, PIN_BUTTON_3, PIN_BUTTON_4 };
	int outputs[] = { PIN_TRIG_OUT_1, PIN_TRIG_OUT_2, PIN_TRIG_OUT_3, PIN_TRIG_OUT_4,
	                  PIN_LED_1, PIN_LED_2, PIN_LED_3, PIN_LED_4 };

	for (int pin : inputs)
		pinMode(context, 0, pin, INPUT);
	for (int pin : outputs)
		pinMode(context, 0, pin, OUTPUT);

	rt_printf("Granular sampler ready. Buffer: %.1fs (%u samples), max %d grains\n",
	          BUFFER_SECONDS, gBufferLen, MAX_GRAINS);
	return true;
}

void render(BelaContext* context, void* userData)
{
	int analogRatio = context->audioFrames / context->analogFrames;

	// ── Per-block: recalculate spawn interval (avoid powf per sample) ────
	float densityHz = MIN_DENSITY_HZ *
		expf(gDensityLogRatio * gParams[2].current);
	gSpawnInterval = gSampleRate / densityHz;

	for (unsigned int n = 0; n < context->audioFrames; n++) {

		// ── Read CVs ─────────────────────────────────────────────────
		int af = n / analogRatio;
		for (int ch = 0; ch < 8; ch++)
			gParams[ch].target = analogRead(context, af, ch);

		for (int ch = 0; ch < 8; ch++)
			gParams[ch].update();

		// ── Read triggers & buttons (edge detection) ─────────────────
		bool trigIn1 = digitalRead(context, n, PIN_TRIG_IN_1);
		bool btn2    = digitalRead(context, n, PIN_BUTTON_2);
		bool btn3    = digitalRead(context, n, PIN_BUTTON_3);
		bool btn4    = digitalRead(context, n, PIN_BUTTON_4);

		// T1 input: record gate
		if (trigIn1 && !gPrevTrigIn1) {
			gRecording = true;
		} else if (!trigIn1 && gPrevTrigIn1) {
			gRecording = false;
		}
		gPrevTrigIn1 = trigIn1;

		// Button 2: toggle recording
		if (btn2 && !gPrevButton2)
			gRecording = !gRecording;
		gPrevButton2 = btn2;

		// Button 3: toggle freeze
		if (btn3 && !gPrevButton3) {
			gFrozen = !gFrozen;
			if (gFrozen)
				gRecording = false;
		}
		gPrevButton3 = btn3;

		// Button 4: clear buffer (deferred to background task)
		if (btn4 && !gPrevButton4) {
			gRecording = false;
			gFrozen    = false;
			// Kill all active grains immediately
			for (int i = 0; i < MAX_GRAINS; i++)
				gGrains[i].active = false;
			// Schedule the heavy memset on a background thread
			Bela_scheduleAuxiliaryTask(gClearTask);
		}
		gPrevButton4 = btn4;

		// ── Record to buffer ─────────────────────────────────────────
		float inputL = audioRead(context, n, 0);

		if (gRecording && !gFrozen) {
			gBuffer[gWriteHead] = inputL;
			gWriteHead = (gWriteHead + 1) % gBufferLen;
			if (gWrittenTotal < gBufferLen)
				gWrittenTotal++;
		}

		// ── Spawn grains ─────────────────────────────────────────────
		unsigned int usableLen = gFrozen ? gBufferLen : gWrittenTotal;
		if (usableLen > (unsigned int)(gSampleRate * MIN_GRAIN_MS * 0.001f)) {

			gSpawnAccum += 1.0f;
			if (gSpawnAccum >= gSpawnInterval) {
				gSpawnAccum -= gSpawnInterval;

				// Find a free grain slot
				int slot = -1;
				for (int i = 0; i < MAX_GRAINS; i++) {
					if (!gGrains[i].active) { slot = i; break; }
				}

				if (slot >= 0) {
					Grain& g = gGrains[slot];

					// Position: CV1 maps across usable buffer
					float basePos = gParams[0].current * (float)usableLen;

					// Spray: CV5 adds random offset
					float spray = (randomFloat() * 2.0f - 1.0f)
					              * gParams[4].current * (float)usableLen * 0.5f;

					float startPos = basePos + spray;
					// Wrap into valid range
					while (startPos < 0) startPos += usableLen;
					while (startPos >= (float)usableLen) startPos -= usableLen;

					// If not frozen, offset relative to oldest data in the buffer
					if (!gFrozen && gWrittenTotal >= gBufferLen) {
						startPos = startPos + (float)gWriteHead;
						if (startPos >= (float)gBufferLen)
							startPos -= (float)gBufferLen;
					}

					// Grain size: CV2 maps linearly 10ms–500ms
					float sizeMs = MIN_GRAIN_MS + gParams[1].current * (MAX_GRAIN_MS - MIN_GRAIN_MS);
					g.grainSize = (unsigned int)(sizeMs * 0.001f * gSampleRate);
					if (g.grainSize < 1) g.grainSize = 1;

					// Pitch: CV4, center at 0.5 = unity, ±2 octaves exponential
					// Use expf instead of powf(2,...): 2^x = e^(x*ln2)
					g.playbackRate = expf((gParams[3].current - 0.5f) * 4.0f * 0.6931472f);

					// Stereo spread: CV6
					float pan = 0.5f + (randomFloat() - 0.5f) * gParams[5].current;
					pan = clampf(pan, 0.0f, 1.0f);
					// Equal-power pan using fast approximation
					// sin/cos of 0..pi/2: use x*(pi/2) mapped parabolic
					float panAngle = pan * 1.5707963f;  // 0..pi/2
					// Fast sin/cos via polynomial (good enough for panning)
					float s = panAngle;
					float c = 1.5707963f - panAngle;
					s = s * (1.0f - s * s * 0.16667f);
					c = c * (1.0f - c * c * 0.16667f);
					g.panL = c;
					g.panR = s;

					// Envelope shape: CV7
					g.envAttack = gParams[6].current;

					g.bufferPos  = startPos;
					g.phase      = 0.0f;
					g.phaseInc   = 1.0f / (float)g.grainSize;
					g.active     = true;

					// Flash LED2
					gGrainFlash = (int)(gSampleRate * 0.015f);
				}
			}
		}

		// ── Process active grains ────────────────────────────────────
		float outL = 0.0f;
		float outR = 0.0f;

		for (int i = 0; i < MAX_GRAINS; i++) {
			Grain& g = gGrains[i];
			if (!g.active) continue;

			float amp = grainEnvelope(g.phase, g.envAttack);
			float sample = bufferRead(g.bufferPos);

			outL += sample * amp * g.panL;
			outR += sample * amp * g.panR;

			// Advance read position
			g.bufferPos += g.playbackRate;
			if (g.bufferPos >= (float)gBufferLen)
				g.bufferPos -= (float)gBufferLen;
			if (g.bufferPos < 0.0f)
				g.bufferPos += (float)gBufferLen;

			// Advance phase (envelope timing, independent of pitch)
			g.phase += g.phaseInc;
			if (g.phase >= 1.0f)
				g.active = false;
		}

		// ── DC block the grain output ────────────────────────────────
		outL = dcBlock(outL, gDCx1L, gDCy1L);
		outR = dcBlock(outR, gDCx1R, gDCy1R);

		// ── Dry/wet mix ──────────────────────────────────────────────
		float wet = gParams[7].current;
		float inputR = audioRead(context, n, 1);

		float mixL = inputL * (1.0f - wet) + outL * wet;
		float mixR = inputR * (1.0f - wet) + outR * wet;

		// Soft clip (fast cubic, no tanhf)
		mixL = softClip(mixL);
		mixR = softClip(mixR);

		audioWrite(context, n, 0, mixL);
		audioWrite(context, n, 1, mixR);

		// ── Update LEDs (heavily decimated) ──────────────────────────
		if (gGrainFlash > 0) gGrainFlash--;

		gLEDCounter++;
		if (gLEDCounter >= LED_UPDATE_INTERVAL) {
			gLEDCounter = 0;

			// LED1: recording / frozen / idle
			if (gFrozen) {
				pinModeOnce(context, n, PIN_LED_1, OUTPUT);
				digitalWriteOnce(context, n, PIN_LED_1, 1);  // YELLOW
			} else if (gRecording) {
				pinModeOnce(context, n, PIN_LED_1, OUTPUT);
				digitalWriteOnce(context, n, PIN_LED_1, 0);  // RED
			} else {
				pinModeOnce(context, n, PIN_LED_1, INPUT);    // OFF
			}

			// LED2: grain spawn flash
			if (gGrainFlash > 0) {
				pinModeOnce(context, n, PIN_LED_2, OUTPUT);
				digitalWriteOnce(context, n, PIN_LED_2, 0);  // RED flash
			} else {
				pinModeOnce(context, n, PIN_LED_2, INPUT);    // OFF
			}

			// LED3: input level
			if (fabsf(inputL) > 0.01f) {
				pinModeOnce(context, n, PIN_LED_3, OUTPUT);
				digitalWriteOnce(context, n, PIN_LED_3, fabsf(inputL) > 0.5f ? 1 : 0);
			} else {
				pinModeOnce(context, n, PIN_LED_3, INPUT);
			}

			// LED4: output level
			float outLevel = fabsf(mixL) + fabsf(mixR);
			if (outLevel > 0.02f) {
				pinModeOnce(context, n, PIN_LED_4, OUTPUT);
				digitalWriteOnce(context, n, PIN_LED_4, outLevel > 1.0f ? 1 : 0);
			} else {
				pinModeOnce(context, n, PIN_LED_4, INPUT);
			}
		}
	}
}

void cleanup(BelaContext* context, void* userData)
{
	delete[] gBuffer;
	gBuffer = nullptr;
}
