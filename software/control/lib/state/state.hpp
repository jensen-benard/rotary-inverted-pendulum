#ifndef STATE_HPP
#define STATE_HPP

class State {
    public:
        State(void (*onEnterEventFunction)(), void (*onExitEventFunction)(), void (*duringUpdateEventFunction)(), const char* name);
        void onEnter();
        void onExit();
        void update();

        const char* getName();
        void (*getOnEnterEventFunction())();
        void (*getOnExitEventFunction())();
        void (*getUpdateEventFunction())();

    private:
        void (*onEnterEventFunction)();
        void (*onExitEventFunction)();
        void (*duringUpdateEventFunction)();
        const char* name;
};

#endif