#ifndef STATE_HPP
#define STATE_HPP

class State {
    public:
        State(void (*onEnterEventFunction)(), void (*onExitEventFunction)(), void (*duringUpdateEventFunction)());
        void onEnter();
        void onExit();
        void update();
    
    private:
        void (*onEnterEventFunction)();
        void (*onExitEventFunction)();
        void (*duringUpdateEventFunction)();
};

#endif