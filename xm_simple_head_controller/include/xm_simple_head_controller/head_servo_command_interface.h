#ifndef HEAD_SERVO_COMMAND_INTERFACE_H
#define HEAD_SERVO_COMMAND_INTERFACE_H

#include<string>
#include<xm_simple_head_controller/head_servo_state_interface.h>
#include<hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface {


class HeadServoHandle :public HeadServoStateHandle
{
public:
      HeadServoHandle() : HeadServoStateHandle() ,yaw_cmd_(0),pitch_cmd_(0){}
      HeadServoHandle(const HeadServoStateHandle& hs, double* yaw_cmd, double* pitch_cmd)
          :HeadServoStateHandle(hs),yaw_cmd_(yaw_cmd),pitch_cmd_(pitch_cmd)
      {
          if(!yaw_cmd)
          {
              throw HardwareInterfaceException("Cannota create handle ' " + hs.getName()+ " '.head_yaw command  data pointer is null");
          }
          if(!pitch_cmd)
          {
              throw HardwareInterfaceException("Cannota create handle ' " + hs.getName() + " '.head_pitch command data pointer is null");
          }
      }

      void setYawcmd(double cmd) {assert(yaw_cmd_); *yaw_cmd_ = cmd;}
      void setPitchcmd(double cmd) {assert(pitch_cmd_); *pitch_cmd_ =cmd;}

      double getYawcmd() const {assert(yaw_cmd_); return *yaw_cmd_;}
      double getPichcmd() const {assert(pitch_cmd_);return *pitch_cmd_;}
private:


    double* yaw_cmd_;
    double* pitch_cmd_;
};

class HeadServoInterface :public HardwareResourceManager<HeadServoHandle,ClaimResources>{};

}

#endif // HEAD_SERVO_COMMAND_INTERFACE_H
