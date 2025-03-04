#ifndef INPUT_VARIABLE_HPP
#define INPUT_VARIABLE_HPP


class InputVariable {
    public:
        virtual ~InputVariable();

        virtual void reset(float currentTime)=0;
        virtual void update(float currentTime)=0;
        virtual float getValue()=0;

};


class StaticInput: public InputVariable {
    public:
        StaticInput(float initialValue);

        void reset(float currentTime) override;
        void update(float currentTime) override;
        float getValue() override;
    
    private:
        float value;
};


class TimeVaryingInput: public InputVariable {
    public:
        TimeVaryingInput(float* values, int totalValues, float currentTime, const float HOLD_TIME);

        void reset(float currentTime) override;
        void update(float currentTime) override;
        float getValue() override;
    
    private:
        float* values;
        int totalValues;
        int currentIndex;
        float previousTime; 

        const float HOLD_TIME;
};



#endif