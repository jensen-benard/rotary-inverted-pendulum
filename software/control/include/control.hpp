#ifndef CONTROL_HPP
#define CONTROL_HPP

class ControlMethod {
    public:
      virtual ~ControlMethod();
      virtual float getOutput(float armAngle, float armAngularVelocity, float pendulumAngle, float pendulumAngularVelocity, float referenceAngle)=0;
  };
  
  class LQRControlMethod: public ControlMethod {
    public:
      LQRControlMethod(float thetaArmGain, float thetaArmDotGain, float thetaPendulumGain, float thetaPendulumDotGain, float trackingGain);
  
      float getOutput(float armAngle, float armAngularVelocity, float pendulumAngle, float pendulumAngularVelocity, float referenceAngle) override;
  
    private:
      float thetaArmGain;
      float thetaArmDotGain;
      float thetaPendulumGain;
      float thetaPendulumDotGain;
      float trackingGain;
  };
  
  
  class LyapunovControlMethod: public ControlMethod {
    public:
      LyapunovControlMethod(float proportionalGain);
  
      float getOutput(float armAngle, float armAngularVelocity, float pendulumAngle, float pendulumAngularVelocity, float referenceAngle) override;
  
    private:
      float proportionalGain;
      
  };


#endif

