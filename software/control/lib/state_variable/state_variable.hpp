#ifndef STATE_VARIABLE_HPP
#define STATE_VARIABLE_HPP

class StateVariable {
    public: 
        StateVariable(float initialValue, float initialRateOfChange, float currentTime);
        void update(float newValue, float currentTime);

        float getValue();
        float getRateOfChange();

    private:
        float value;
        float rateOfChange;
        float previousTime;
};

#endif