#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "math_utility.hpp"

class ControlMethod {
    public:
      virtual ~ControlMethod();
      virtual double getOutput(double armAngle, double armAngularVelocity, double pendulumAngle, double pendulumAngularVelocity, double referenceAngle)=0;
  };
  
  class LQRControlMethod: public ControlMethod {
    public:
      LQRControlMethod(double thetaArmGain, double thetaArmDotGain, double thetaPendulumGain, double thetaPendulumDotGain, double trackingGain);
  
      double getOutput(double armAngle, double armAngularVelocity, double pendulumAngle, double pendulumAngularVelocity, double referenceAngle) override;
  
    private:
      double thetaArmGain;
      double thetaArmDotGain;
      double thetaPendulumGain;
      double thetaPendulumDotGain;
      double trackingGain;
  };
  
  
  class LyapunovControlMethod: public ControlMethod {
    public:
      LyapunovControlMethod(double proportionalGain);
  
      double getOutput(double armAngle, double armAngularVelocity, double pendulumAngle, double pendulumAngularVelocity, double referenceAngle) override;
  
    private:
      double proportionalGain;
      
  };


#endif
