#ifndef STATE_HPP
#define STATE_HPP

class State {
    public:
        State(void (*onEnterEventFunction)(), void (*onExitEventFunction)(), void (*duringUpdateEventFunction)(), char* name);
        void onEnter();
        void onExit();
        void update();

        char* getName();

    private:
        void (*onEnterEventFunction)();
        void (*onExitEventFunction)();
        void (*duringUpdateEventFunction)();
        char* name;
};

#endif