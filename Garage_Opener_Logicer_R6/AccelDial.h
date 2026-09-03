/*
 * AccelDial - for the CLK/DT rotary encoder module documented in
 * 41015725_-_Rotary_Encoder.pdf, with the same acceleration feel as
 * AccelEncoder.
 *
 * That module's detents differ from the EC11-style encoder AccelEncoder was
 * written for, which is why this is a separate class. Per the datasheet, its
 * two switches are BOTH open or BOTH closed at every resting click, and one
 * click toggles both:
 *
 *     rest         transient        rest
 *   both open  ->  one switch  ->  both closed   = one click
 *    (1,1)         moved first       (0,0)
 *
 * Direction is whichever switch moves first - A/CLK first is clockwise, B/DT
 * first is counter-clockwise - and the click is only counted once the knob
 * arrives at the *opposite* rest state. That makes the decoder self-validating
 * in a way a step-accumulator is not: nudge the knob so it wobbles off a
 * detent and falls back, and nothing is counted, because it never reached the
 * other rest state.
 *
 * Wiring (5-pin module -> Arduino Uno/Nano):
 *   CLK -> D2        encoder pin A
 *   DT  -> D3        encoder pin B
 *   SW  -> unused by this class; drive it with your own button code
 *   +   -> 5V        powers the module's own 10k pull-ups
 *   GND -> GND
 *
 * Usage:
 *     AccelDial dial(2, 3);
 *     void setup() { dial.begin(); dial.setRange(0, 1023); }
 *     void loop()  { if (dial.update()) Serial.println(dial.value()); }
 *
 * Turn it slowly and the value moves by 1 per click. Spin it and the step
 * grows toward maxStep - about two flicks across a 0..1023 range - and a pause
 * drops the next click back to 1.
 *
 * update() only reads two pins and does integer math, so it is safe to call
 * from a pin-change ISR when loop() can block (an SD write, a delay()) for
 * longer than a click takes. Polling is otherwise fine, but see
 * missedClicks(): this decoder needs to observe the transient state, so if
 * polling is too slow to catch it the click is dropped rather than guessed at,
 * and that counter is how you find out.
 *
 *     attachInterrupt(digitalPinToInterrupt(2), tick, CHANGE);
 *     attachInterrupt(digitalPinToInterrupt(3), tick, CHANGE);
 *     void tick() { dial.update(); }
 *
 * In that arrangement value() is a multi-byte read an ISR can interrupt
 * halfway, so on an 8-bit core read it inside noInterrupts()/interrupts().
 */

#ifndef ACCEL_DIAL_H
#define ACCEL_DIAL_H

#include <Arduino.h>
#include "AccelRamp.h"

class AccelDial {
public:
  AccelDial(uint8_t pinClk, uint8_t pinDt);

  // Sets the pin modes and seeds the decoder from the resting click, so the
  // first update() cannot invent a step. The module carries its own 10k
  // pull-ups; the internal ones are enabled anyway so it still works with the
  // + rail unconnected. Pass false to leave the pins as plain inputs.
  void begin(bool usePullups = true);

  // Poll the pins. True only when value() actually changed - a click into the
  // end of the range, a wobble that falls back, or a dropped click all
  // return false.
  bool update();

  long value() const { return _value; }
  void setValue(long v);

  // Inclusive bounds. Unless setAcceleration() has been called, this also
  // picks maxStep as 1/32 of the range (so 0..1023 accelerates to 32 per
  // click, crossing the range in about two flicks).
  void setRange(long lo, long hi);
  long rangeLow() const  { return _lo; }
  long rangeHigh() const { return _hi; }

  // Off by default: the value stops at the bounds instead of rolling over.
  void setWrap(bool wrap) { _wrap = wrap; }

  // Swap which way counts up, if the knob feels backwards in the panel.
  void setReversed(bool reversed) { _reversed = reversed; }

  // Rest-state transitions per felt click. 1 for the module in the datasheet.
  // Set 2 if one click of your unit moves the value by 2 - some encoders pass
  // through two rest states per detent.
  void setClicksPerDetent(uint8_t clicks) { _cpd = clicks ? clicks : 1; }

  // At or below slowHz clicks/second a click moves the value by 1; at or above
  // fastHz it moves it by maxStep; between the two it eases in quadratically.
  void setAcceleration(uint16_t slowHz, uint16_t fastHz, long maxStep);
  void disableAcceleration();

  // Hook for an external button: while fine mode is on, every click is worth
  // exactly one count regardless of spin speed. This class never reads a
  // button itself - feed it from your own debounced input, e.g.
  //     dial.setFineMode(button.pressed());
  void setFineMode(bool fine) { _fine = fine; }
  bool fineMode() const { return _fine; }

  // The signed amount the last click applied, before clamping.
  long lastStep() const { return _lastStep; }

  // -1, +1, or 0 before anything has moved.
  int8_t direction() const { return _dir; }

  // Smoothed spin speed in clicks/second, the input to the accel curve.
  uint16_t rateHz() const { return _ramp.rateHz(); }

  // Raw click count since begin(), ignoring range, wrap and acceleration.
  long detents() const { return _detents; }

  // Clicks seen as a jump straight from one rest state to the other, with the
  // transient state never observed. Direction is unknowable in that case, so
  // the click is dropped. A number climbing here means update() is not being
  // called often enough - move it to a pin-change interrupt.
  unsigned long missedClicks() const { return _missed; }

private:
  bool applyClick(int8_t dir);

  uint8_t   _pinClk, _pinDt;
  uint8_t   _prev = 3;         // last code, (A << 1) | B
  uint8_t   _restFrom = 3;     // rest state the current move left
  int8_t    _pendingDir = 0;   // direction named by the first switch to move
  bool      _havePending = false;
  uint8_t   _cpd = 1;
  int8_t    _clickAcc = 0;

  long      _value = 0;
  long      _lo = 0, _hi = 1023;
  bool      _wrap = false;
  bool      _reversed = false;
  bool      _fine = false;
  bool      _accelSetByUser = false;

  AccelRamp _ramp;
  long      _lastStep = 0;
  long      _detents = 0;
  int8_t    _dir = 0;
  unsigned long _missed = 0;
};

#endif  // ACCEL_DIAL_H
