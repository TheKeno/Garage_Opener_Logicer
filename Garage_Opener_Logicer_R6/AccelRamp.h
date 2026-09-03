/*
 * AccelRamp - the speed-to-step-size curve, factored out so more than one
 * encoder class can share one feel.
 *
 * Call stepForDetent() exactly once per detent. It measures the interval since
 * the previous detent, smooths it, and maps it onto a step size: 1 at or below
 * slowHz detents/second, maxStep at or above fastHz, eased in quadratically
 * between the two so the onset is gentle rather than a cliff.
 *
 * The math is identical to the copy inside AccelEncoder, so both knobs feel
 * the same. (AccelEncoder keeps its own copy deliberately - it predates this
 * header and is left untouched; switching it over here is behaviour-neutral if
 * you ever want one implementation.)
 */

#ifndef ACCEL_RAMP_H
#define ACCEL_RAMP_H

#include <Arduino.h>

class AccelRamp {
public:
  void configure(uint16_t slowHz, uint16_t fastHz, long maxStep) {
    if (slowHz < 1) slowHz = 1;
    if (fastHz <= slowHz) fastHz = slowHz + 1;
    _slowUs = 1000000UL / slowHz;
    _fastUs = 1000000UL / fastHz;
    _idleUs = _slowUs * 2;
    _maxStep = maxStep < 1 ? 1 : maxStep;
    _ivUs = _slowUs;
  }

  void setMaxStep(long maxStep) { _maxStep = maxStep < 1 ? 1 : maxStep; }
  long maxStep() const { return _maxStep; }
  void disable() { _maxStep = 1; }

  // Park the ramp so the next detent is worth 1 again.
  void reset(uint32_t nowUs) {
    _ivUs = _slowUs;
    _lastUs = nowUs - _idleUs;   // unsigned: a wrap here is harmless
  }

  long stepForDetent(uint32_t nowUs) {
    uint32_t dt = nowUs - _lastUs;         // unsigned: correct across rollover
    _lastUs = nowUs;
    if (dt >= _idleUs) {
      _ivUs = _slowUs;                     // a fresh spin always starts at 1
    } else {
      if (dt < _fastUs) dt = _fastUs;      // clamp: one freak-fast detent must
      _ivUs = (_ivUs + dt) >> 1;           // not slam the step to maxStep
    }
    return stepFor(_ivUs);
  }

  uint16_t rateHz() const {
    if (_ivUs == 0) return 0;
    uint32_t hz = 1000000UL / _ivUs;
    return hz > 65535UL ? 65535 : (uint16_t)hz;
  }

private:
  // Fixed point at 1/256 throughout: no float, and one division per detent.
  long stepFor(uint32_t intervalUs) const {
    if (_maxStep <= 1 || intervalUs >= _slowUs) return 1;
    if (intervalUs <= _fastUs) return _maxStep;
    uint32_t f = ((_slowUs - intervalUs) * 256UL) / (_slowUs - _fastUs);  // 0..256
    uint32_t eased = (f * f + 128) >> 8;
    // Rounded, not truncated: the smoothed interval only approaches _fastUs
    // asymptotically, so truncating would put maxStep permanently one count
    // out of reach.
    return 1 + (long)(((uint32_t)(_maxStep - 1) * eased + 128) >> 8);
  }

  long     _maxStep = 32;
  uint32_t _slowUs = 125000;   // 8 detents/s
  uint32_t _fastUs = 25000;    // 40 detents/s
  uint32_t _idleUs = 250000;
  uint32_t _lastUs = 0;
  uint32_t _ivUs = 125000;
};

#endif  // ACCEL_RAMP_H
