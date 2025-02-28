#ifndef REFERENCE_HPP
#define REFERENCE_HPP

class Reference {
    public:
        Reference(double* trajectoryPtr, int trajectoryLength, int angleHoldTime);
        void update();
        double getCurrentAngle();
        void reset(float zeroAngle);
    
  
    private:
        double currentAngle;
        double* trajectory;
        int currentTrajectoryIndex;
        const int TRAJECTORY_LENGTH;
        double previousUpdateTime;
        const double ANGLE_HOLD_TIME;
        float zeroAngle;

        void updateTrajectoryIndex();
  };


#endif 
