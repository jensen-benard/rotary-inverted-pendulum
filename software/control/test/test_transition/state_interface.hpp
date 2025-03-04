#ifndef STATE_INTERFACE_HPP
#define STATE_INTERFACE_HPP

class StateInterface {
    public:
        virtual void onExit();
        virtual void update();
        virtual const char* getName();
        virtual void (*getOnEnterEventFunction())();
        virtual void (*getOnExitEventFunction())();
        virtual void (*getUpdateEventFunction())();

        virtual ~StateInterface() = default;
};


#endif