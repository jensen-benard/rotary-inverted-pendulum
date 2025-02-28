#ifndef ACTUATOR_HPP
#define ACTUATOR_HPP

class Actuator{
    public:
        virtual void actuate(float controlInput)=0;
        virtual void stop()=0;
};


#endif