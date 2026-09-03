#include "AccelDial.h"

// A resting click has both switches in the same state: 0b00 (both closed) or
// 0b11 (both open). 0b01 and 0b10 are the transient states passed through
// mid-click, while one switch has moved and the other has not.
static inline bool isRest(uint8_t code) { return code == 0 || code == 3; }

AccelDial::AccelDial(uint8_t pinClk, uint8_t pinDt)
  : _pinClk(pinClk), _pinDt(pinDt) {}

void AccelDial::begin(bool usePullups) {
  uint8_t mode = usePullups ? INPUT_PULLUP : INPUT;
  pinMode(_pinClk, mode);
  pinMode(_pinDt, mode);
  _prev = (uint8_t)((digitalRead(_pinClk) ? 2 : 0) | (digitalRead(_pinDt) ? 1 : 0));
  _restFrom = isRest(_prev) ? _prev : 3;
  _havePending = false;
  _clickAcc = 0;
  _ramp.reset(micros());
}

void AccelDial::setValue(long v) {
  if (v < _lo) v = _lo;
  else if (v > _hi) v = _hi;
  _value = v;
}

void AccelDial::setRange(long lo, long hi) {
  if (hi < lo) { long t = lo; lo = hi; hi = t; }
  _lo = lo;
  _hi = hi;
  if (!_accelSetByUser) {
    long span = _hi - _lo + 1;      // inclusive: 0..1023 is a span of 1024
    long m = span / 32;
    _ramp.setMaxStep(m < 1 ? 1 : m);
  }
  setValue(_value);
}

void AccelDial::setAcceleration(uint16_t slowHz, uint16_t fastHz, long maxStep) {
  _ramp.configure(slowHz, fastHz, maxStep);
  _accelSetByUser = true;
}

void AccelDial::disableAcceleration() {
  _ramp.disable();
  _accelSetByUser = true;
}

bool AccelDial::update() {
  uint8_t cur = (uint8_t)((digitalRead(_pinClk) ? 2 : 0) | (digitalRead(_pinDt) ? 1 : 0));
  if (cur == _prev) return false;

  bool changed = false;
  uint8_t prev = _prev;
  _prev = cur;

  if (isRest(prev) && !isRest(cur)) {
    // Leaving a detent. Bit 1 is CLK/A, bit 0 is DT/B; whichever moved first
    // names the direction, per the datasheet: A first is clockwise.
    uint8_t moved = (uint8_t)(prev ^ cur);
    _pendingDir = (moved & 0x02) ? +1 : -1;
    _restFrom = prev;
    _havePending = true;

  } else if (!isRest(prev) && isRest(cur)) {
    if (_havePending) {
      _havePending = false;
      // Only the opposite rest state is a completed click. Falling back to the
      // one we left is a wobble on the detent, and counts for nothing.
      if (cur != _restFrom) {
        _clickAcc += _pendingDir;
        if (_clickAcc >= (int8_t)_cpd)       { _clickAcc = 0; changed = applyClick(+1); }
        else if (_clickAcc <= -(int8_t)_cpd) { _clickAcc = 0; changed = applyClick(-1); }
      }
    }

  } else if (isRest(prev) && isRest(cur)) {
    // Both switches changed between two reads, so the transient state was
    // never seen and the direction is genuinely unknowable. Dropped, not
    // guessed - see missedClicks().
    _missed++;
    _havePending = false;
  }
  // A transient-to-transient change cannot happen without passing through a
  // rest state, so it is bounce; ignored.

  return changed;
}

bool AccelDial::applyClick(int8_t dir) {
  if (_reversed) dir = -dir;
  _dir = dir;
  _detents += dir;

  // Run the ramp even in fine mode, so its interval bookkeeping stays current
  // and letting go of fine mode does not produce a surprise jump.
  long step = _ramp.stepForDetent(micros());
  if (_fine) step = 1;
  _lastStep = dir * step;

  long before = _value;
  long span = _hi - _lo + 1;
  long v = _value + _lastStep;
  if (_wrap && span > 0) {
    v = (v - _lo) % span;
    if (v < 0) v += span;
    v += _lo;
  } else {
    if (v < _lo) v = _lo;
    else if (v > _hi) v = _hi;
  }
  _value = v;
  return _value != before;
}
