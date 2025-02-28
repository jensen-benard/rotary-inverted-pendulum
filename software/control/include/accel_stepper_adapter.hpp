#ifndef ACCEL_STEPPER_ADAPTER_HPP
#define ACCEL_STEPPER_ADAPTER_HPP

#include "sensor.hpp"
#include "actuator.hpp"
#include <AccelStepper.h>

class AccelStepperAdapter : public Sensor, public Actuator {
    public:
        AccelStepperAdapter(AccelStepper* stepper, const int MICROSTEPS_PER_DEGREE);
        double getData() override;
        void actuate(float controlInput) override;
        void stop() override;

    private:
        AccelStepper* stepper;
        const int MICROSTEPS_PER_DEGREE;
        float inputSpeed;
        float lastUpdateTime;
};


#endif