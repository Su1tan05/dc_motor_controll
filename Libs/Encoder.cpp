#include "Encoder.h"
#include <cstdio>

Encoder::Encoder(PinName channelA, PinName channelB, int reductionRatio,
                 int pulsesPerRev, Encoding encoding)
    : channelA_(channelA), channelB_(channelB) {

  m_pulses = 0;
  m_revolutions = 0;
  m_reductionRatio = reductionRatio;
  m_pulsesPerRev = pulsesPerRev;
  m_encoding = encoding;

  if (m_encoding == X2_ENCODING) {
    m_pulseRepeats = 2; //????????????????????????????
  } else if (m_encoding == X4_ENCODING) {
    m_pulseRepeats = 4;
  }

  // Workout what the current state is.
  int chanA = channelA_.read();
  int chanB = channelB_.read();

  // 2-bit state.
  m_currState = (chanA << 1) | (chanB);
  m_prevState = m_currState;

  // X2 encoding uses interrupts on only channel A.
  // X4 encoding uses interrupts on      channel A,
  // and on channel B.
  channelA_.rise(this, &Encoder::encode);
  channelA_.fall(this, &Encoder::encode);

  // If we're using X4 encoding, then attach interrupts to channel B too.
  if (encoding == X4_ENCODING) {
    channelB_.rise(this, &Encoder::encode);
    channelB_.fall(this, &Encoder::encode);
  }
}

void Encoder::reset(void) {

  m_pulses = 0;
  m_revolutions = 0;
}

int Encoder::getCurrentState(void) { return m_currState; }

int Encoder::getPulses(void) { return m_pulses; }

float Encoder::getCurrentAngle() {

  float angle = (m_pulses * 360.00) /
                (m_reductionRatio * m_pulsesPerRev * m_pulseRepeats);
  return angle;
}

void Encoder::setZeroAngleRad(float zero_angle) {
    m_pulses = (int) ((zero_angle * m_reductionRatio * m_pulsesPerRev * m_pulseRepeats) / (2 * 3.141592653));
}

float Encoder::getCurrentAngleRad() {
  float angle = (m_pulses * 2 * 3.141592653) /
                (m_reductionRatio * m_pulsesPerRev * m_pulseRepeats);
  return angle;
}

float Encoder::getCurrentSpeed() { return m_currSpeed; }

// +-------------+
// | X2 Encoding |
// +-------------+
//
// When observing states two patterns will appear:
//
// Counter clockwise rotation:
//
// 10 -> 01 -> 10 -> 01 -> ...
//
// Clockwise rotation:
//
// 11 -> 00 -> 11 -> 00 -> ...
//
// We consider counter clockwise rotation to be "forward" and
// counter clockwise to be "backward". Therefore pulse count will increase
// during counter clockwise rotation and decrease during clockwise rotation.
//
// +-------------+
// | X4 Encoding |
// +-------------+
//
// There are four possible states for a quadrature encoder which correspond to
// 2-bit gray code.
//
// A state change is only valid if of only one bit has changed.
// A state change is invalid if both bits have changed.
//
// Clockwise Rotation ->
//
//    00 01 11 10 00
//
// <- Counter Clockwise Rotation
//
// If we observe any valid state changes going from left to right, we have
// moved one pulse clockwise [we will consider this "backward" or "negative"].
//
// If we observe any valid state changes going from right to left we have
// moved one pulse counter clockwise [we will consider this "forward" or
// "positive"].
//
// We might enter an invalid state for a number of reasons which are hard to
// predict - if this is the case, it is generally safe to ignore it, update
// the state and carry on, with the error correcting itself shortly after.
void Encoder::encode(void) {

  int change = 0;
  int chanA = channelA_.read();
  int chanB = channelB_.read();

  //   m_prevPulses = m_pulses;
  // 2-bit state.
  m_currState = (chanA << 1) | (chanB);

  if (m_encoding == X2_ENCODING) {

    // 11->00->11->00 is counter clockwise rotation or "forward".
    if ((m_prevState == 0x3 && m_currState == 0x0) ||
        (m_prevState == 0x0 && m_currState == 0x3)) {

      m_pulses++;

    }
    // 10->01->10->01 is clockwise rotation or "backward".
    else if ((m_prevState == 0x2 && m_currState == 0x1) ||
             (m_prevState == 0x1 && m_currState == 0x2)) {

      m_pulses--;
    }

  } else if (m_encoding == X4_ENCODING) {

    // Entered a new valid state.
    if (((m_currState ^ m_prevState) != INVALID) &&
        (m_currState != m_prevState)) {
      // 2 bit state. Right hand bit of prev XOR left hand bit of current
      // gives 0 if clockwise rotation and 1 if counter clockwise rotation.
      change = (m_prevState & PREV_MASK) ^ ((m_currState & CURR_MASK) >> 1);

      if (change == 0) {
        change = -1;
      }

      m_pulses -= change;
    }
  }

  m_prevState = m_currState;
}

void Encoder::speedCalculation(void) {
  float curAngle = getCurrentAngle();
  m_dt = 0.01;
  m_currSpeed = (m_prevAngle - curAngle) / m_dt;
  m_prevAngle = curAngle;
}