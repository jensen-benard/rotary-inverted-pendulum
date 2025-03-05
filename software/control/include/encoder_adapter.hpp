#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "sensor.hpp"
#include <Encoder.h>

class EncoderAdapter : public Sensor {
    public:
        EncoderAdapter(Encoder* encoder, const float ANGLE_OFFSET, const int PULSES_PER_DEGREE);
        float getData() override;

    private:
        Encoder* encoder;
        const float ANGLE_OFFSET;
        const int PULSES_PER_DEGREE;
};

#endif
