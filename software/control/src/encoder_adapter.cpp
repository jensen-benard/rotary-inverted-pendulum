#include "encoder_adapter.hpp"
#include <Encoder.h>

EncoderAdapter::EncoderAdapter(Encoder* encoder, const double ANGLE_OFFSET,  const int PULSES_PER_DEGREE): ANGLE_OFFSET(ANGLE_OFFSET), PULSES_PER_DEGREE(PULSES_PER_DEGREE) {
    this->encoder = encoder;
}

double EncoderAdapter::getData() {
    long encoderPosition = encoder->read();
    float angle = encoderPosition / PULSES_PER_DEGREE + ANGLE_OFFSET;
    return angle;
}