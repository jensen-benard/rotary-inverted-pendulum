#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include "actuator.hpp"
#include "sensor.hpp"
#include "state_variable.hpp"
#include "control.hpp"
#include "transition.hpp"
#include "state.hpp"
#include "state_machine.hpp"
#include "input_variable.hpp"


class RotaryInvertedPendulumSystem {
    private:
        static RotaryInvertedPendulumSystem* instance;
        
    public:
        RotaryInvertedPendulumSystem(Actuator* stepperMotor, 
            Sensor* pendulumAngleSensor, Sensor* armAngleSensor,
            ControlMethod* swingUpControlMethod, ControlMethod* balanceControlMethod,
            StateVariable* pendulumAngle, StateVariable* armAngle,
            StateVariable* pendulumAngleRateOfChange, StateVariable* armAngleRateOfChange,
            InputVariable* referenceAngle);

        void run();

        void setStateMachine(StateMachine* stateMachine);

    private:
        Actuator* stepperMotor;
        Sensor* pendulumAngleSensor;
        Sensor* armAngleSensor;
        ControlMethod* swingUpControlMethod;
        ControlMethod* balanceControlMethod;
        StateVariable* pendulumAngle;
        StateVariable* armAngle;
        StateVariable* pendulumAngleRateOfChange;
        StateVariable* armAngleRateOfChange;
        InputVariable* referenceAngle;
    
        StateMachine* stateMachine;
    
    private: 
            
        void updateStateVariables();

    public:
        
        static void runSwingUpControl();
        
        static void resetReferenceAngle();

        static void runBalanceControl();

        static void stop();

        static bool swingUpCondition(float swingUpTriggerAngle);

        static bool balanceCondition(float balanceTriggerAngle);

        static bool emergencyStopCondition(float armAngleLimit);
};


#endif