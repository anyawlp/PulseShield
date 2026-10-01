// PulseShield v6 -> Teensy 4.0 (C++), per-sample, fully causal. Validated on host against the Python reference.
// Pipeline: DC blocker -> 5 ms look-ahead delay | causal 5 ms RMS + 50 ms long-term RMS | onset detector
//           (rise > 6/12/24/30/36 dB within 5/10/20/40/80 ms AND >= 3x the 50 ms level before it) | instant clamp to
//           pre-spike loudness, rise to a cap (+3 dB over pre-event level, <= 0.08 RMS) | loudness may rise at most
//           30 dB per 220 ms (0.136 dB/ms, linear IN dB) during rise and recovery; every rise is then S-curve smoothed
//           (two cascaded 15 ms poles in dB); the gain drop is a 2 ms smooth S (two cascaded 1 ms averages).
// Hardware: INMP441 L/R pin -> GND (left channel). Tools > USB Type > Audio.
#include <math.h>
#include <stdint.h>
#ifndef PS_SR
#define PS_SR 44117.64706f          // Teensy AUDIO_SAMPLE_RATE_EXACT
#endif
#ifndef PS_DC
#define PS_DC 1                     // 1 = DC blocker on (hardware); 0 only for bit-exact host comparison
#endif

namespace ps {
const int L = 220, WENV = 220, LS = 44, PLANW = 176, WLT = 2205, PKN = 44, HN = 3530;  // 5 ms, 5 ms, 1 ms, 4 ms, 50 ms, 1 ms, 80 ms history
const int NT = 5;
const int SH[NT] = {220, 441, 882, 1764, 3528};                                         // 5/10/20/40/80 ms
const float ONSET_DB[NT] = {6.0f, 12.0f, 24.0f, 30.0f, 36.0f};                         // dB rise that counts as a spike
const float LOUD_FLOOR = 0.02f, GAIN_FLOOR = 0.01f, LT_RATIO = 3.0f, BASE_REF = 0.02f, ABS_CAP = 0.06f;
const float K = 1.1885023f;                                                             // +1.5 dB
const float S_TAU = 0.015f, SNAP_LN = 0.00058f;
float rise_f = 1.0f;                                                                    // per-sample gain multiplier (30 dB per 220 ms)
const int END_N = 88;                                                                   // 2 ms
const float R_DC = 0.9964f;                                                             // ~25 Hz high-pass

float delayBuf[L]; int dpos;
float sq[WENV]; int spos; double ssum;
float sql[WLT]; int sqlpos; double sqlsum;
float pk[PKN]; int ppos;
float hdb[HN], henv[HN], hlt[HN]; int hpos; int count;
float plan[L + 1]; int plpos;
float ma[LS]; int mapos; double masum;
float ma2[LS]; int mapos2; double masum2; float gl_, s1_, s2_, sa_;
float tg, g0, target_, base_, ep_peak, gain_; bool in_ep; int quiet;
float dcX, dcY;

void reset() {
  for (int i = 0; i < L; i++) delayBuf[i] = 0; for (int i = 0; i < WENV; i++) sq[i] = 0; for (int i = 0; i < WLT; i++) sql[i] = 0;
  for (int i = 0; i < PKN; i++) pk[i] = 0; for (int i = 0; i < HN; i++) hdb[i] = henv[i] = hlt[i] = 0;
  for (int i = 0; i < L + 1; i++) plan[i] = 1; for (int i = 0; i < LS; i++) ma[i] = ma2[i] = 1;
  dpos = spos = sqlpos = ppos = hpos = count = plpos = mapos = mapos2 = 0; ssum = sqlsum = 0; masum = masum2 = LS; gl_ = 1; s1_ = s2_ = 0; sa_ = 1.0f - expf(-1.0f / (S_TAU * PS_SR));
  tg = g0 = target_ = gain_ = 1; base_ = 0; rise_f = powf(10.0f, (30.0f / 0.22f) / 20.0f / PS_SR); ep_peak = 0; in_ep = false; quiet = 0; dcX = dcY = 0;
}

// x: one input sample in [-1,1]. returns one output sample. `gain_out` (optional) gets the applied gain.
inline float step(float raw, float* gain_out = 0) {
#if PS_DC
  float x = raw - dcX + R_DC * dcY; dcX = raw; dcY = x;
#else
  float x = raw;
#endif
  // causal RMS envelopes (running sums in double: no drift)
  float s = x * x;
  ssum += s - sq[spos]; sq[spos] = s; spos = (spos + 1) % WENV;
  float env = sqrtf(fmaxf((float)ssum, 0.0f) / WENV);
  hdb[hpos] = 20.0f * log10f(env + 1e-6f); henv[hpos] = env;
  sqlsum += s - sql[sqlpos]; sql[sqlpos] = s; sqlpos = (sqlpos + 1) % WLT;
  hlt[hpos] = sqrtf(fmaxf((float)sqlsum, 0.0f) / WLT);
  pk[ppos] = fabsf(x); ppos = (ppos + 1) % PKN;
  float pkmax = 0; for (int i = 0; i < PKN; i++) if (pk[i] > pkmax) pkmax = pk[i];
  float level = fmaxf(env, pkmax / 1.4142f);

  // onset detector
  bool flag = false; float g_need = 1.0f, base_hit = 1e9f;
  if (count >= WENV + SH[NT - 1]) {
    for (int j = 0; j < NT; j++) {
      int p = (hpos - SH[j] + HN) % HN;
      if (hdb[hpos] - hdb[p] > ONSET_DB[j] && env > LOUD_FLOOR && env > LT_RATIO * hlt[p]) {
        flag = true; g_need = fminf(g_need, fmaxf(GAIN_FLOOR, henv[p] / level)); base_hit = fminf(base_hit, henv[p]);
      }
    }
  }
  hpos = (hpos + 1) % HN; if (count < 1000000) count++;

  // loud-event tracking + planned gain
  if (flag) {
    if (!in_ep) { g0 = 1.0f; base_ = base_hit; ep_peak = 0; }
    in_ep = true; quiet = 0; base_ = fminf(base_, base_hit);
  } else if (in_ep) {
    quiet = (env < 0.3f * ep_peak) ? quiet + 1 : 0;
    if (quiet >= END_N) { in_ep = false; ep_peak = 0; }
  }
  if (in_ep) {
    ep_peak = fmaxf(ep_peak, env);
    float cap = fminf(fmaxf(base_, BASE_REF) * K, ABS_CAP);
    target_ = fminf(1.0f, fmaxf(GAIN_FLOOR, cap / fmaxf(ep_peak, 1e-9f)));
    if (flag) { g0 = fminf(g0, g_need); tg = fminf(fminf(tg, g0), target_); }
    tg = fminf(target_, tg * rise_f);
  } else tg = 1.0f;

  // look-ahead: gain for the audio emerging now = min of plan over the next PLANW samples; 1 ms ramp; rise-limited
  plan[plpos] = tg; plpos = (plpos + 1) % (L + 1);
  float gmin = 2.0f; for (int j = 0; j <= PLANW; j++) { float v = plan[(plpos + j) % (L + 1)]; if (v < gmin) gmin = v; }
  masum += gmin - ma[mapos]; ma[mapos] = gmin; mapos = (mapos + 1) % LS;
  float gma = (float)(masum / LS); if (gma > 1.0f - 1e-6f) gma = 1.0f;
  masum2 += gma - ma2[mapos2]; ma2[mapos2] = gma; mapos2 = (mapos2 + 1) % LS;     // second 1 ms average -> smooth S drop
  float gma2 = (float)(masum2 / LS); if (gma2 > 1.0f - 1e-6f) gma2 = 1.0f;
  gl_ = fminf(gma2, gl_ * rise_f);                                              // loudness rises <= 0.136 dB/ms
  if (gl_ >= 1.0f && s2_ == 0.0f) { gain_ = 1.0f; }                             // fast path: fully open
  else {
    float lgl = logf(gl_);
    if (lgl < s2_) { s1_ = s2_ = lgl; }                                         // drops followed at once
    else { s1_ += sa_ * (lgl - s1_); s2_ += sa_ * (s1_ - s2_); }                // every rise: S-curve (2 cascaded poles in dB)
    if (gl_ >= 1.0f && s2_ > -SNAP_LN) { s1_ = s2_ = 0.0f; }
    gain_ = expf(s2_);
  }

  float outx = delayBuf[dpos]; delayBuf[dpos] = x; dpos = (dpos + 1) % L;
  float y = outx * gain_; if (y > 1.0f) y = 1.0f; if (y < -1.0f) y = -1.0f;
  if (gain_out) *gain_out = gain_;
  return y;
}
}  // namespace ps

#ifdef PS_HOST   // ---- host test hooks (not compiled on Teensy) ----
extern "C" void ps_reset() { ps::reset(); }
extern "C" void ps_process(const int16_t* in, int16_t* out, float* g, int n) {
  for (int i = 0; i < n; i++) { float gg; float y = ps::step((float)in[i] / 32768.0f, &gg); out[i] = (int16_t)(y * 32767.0f); g[i] = gg; }
}
extern "C" double ps_ssum() { return ps::ssum; }
#else            // ---- Teensy 4.0 audio plumbing ----
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
AudioInputI2S i2s_mic; AudioRecordQueue queue_in; AudioPlayQueue queue_out; AudioOutputUSB usb_out;
AudioConnection patchCord1(i2s_mic, 0, queue_in, 0);
AudioConnection patchCord2(queue_out, 0, usb_out, 0);
AudioConnection patchCord3(queue_out, 0, usb_out, 1);
void setup() { AudioMemory(60); ps::reset(); queue_in.begin(); }
void loop() {
  if (queue_in.available() >= 1) {
    int16_t* in_buffer = queue_in.readBuffer();
    int16_t* out_buffer = queue_out.getBuffer();
    if (out_buffer) for (int i = 0; i < 128; i++) out_buffer[i] = (int16_t)(ps::step((float)in_buffer[i] / 32768.0f) * 32767.0f);
    queue_in.freeBuffer();
    if (out_buffer) queue_out.playBuffer();
  }
}
#endif
