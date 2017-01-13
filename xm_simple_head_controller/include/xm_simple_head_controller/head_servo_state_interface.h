#ifndef HEAD_SERVO_INTERFACE_H
#define HEAD_SERVO_INTERFACE_H


#include<hardware_interface/internal/hardware_resource_manager.h>
#include<string>


namespace  hardware_interface {

class HeadServoStateHandle{

public:
    HeadServoStateHandle(): name_(),head_yaw_(0),head_pitch_(0){}

    HeadServoStateHandle(const std::string& name,const double* head_yaw,const double* head_pitch)
        : name_(name), head_yaw_(head_yaw),head_pitch_(head_pitch)
    {
        if(!head_pitch)
        {
            throw HardwareInterfaceException("Cannota create handle ' " + name + " '.head_pitch data pointer is null");
        }
        if(!head_yaw)
        {
            throw HardwareInterfaceException("Cannota create handle ' " + name + " '.head_yaw data pointer is null");
        }

    }

    std::string getName() const { return name_;}
    double getHeadYaw() const { assert(head_yaw_); return *head_yaw_;}
    double getHeadPitch() const {assert(head_pitch_); return *head_pitch_;}

    const double* getHeadYawPtr() const {return head_yaw_;}
    const double* getHeadPitchPtr() const {return head_pitch_;}


private:

    std::string name_;
    const double* head_yaw_;
    const double* head_pitch_;
};

class HeadServoStateInterface : public HardwareResourceManager<HeadServoStateHandle>{};

}
#endif // HEAD_SERVO_INTERFACE_H
