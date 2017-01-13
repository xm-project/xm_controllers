#ifndef XM_SIMPLE_HEAD_CONTROLLER_H
#define XM_SIMPLE_HEAD_CONTROLLER_H

#include<ros/ros.h>
#include<controller_interface/controller.h>
#include<pluginlib/class_list_macros.h>
#include<tf/tfMessage.h>
#include<realtime_tools/realtime_buffer.h>
#include<realtime_tools/realtime_publisher.h>
#include<xm_simple_head_controller/head_servo_command_interface.h>
#include<xm_simple_head_controller/head_servo_state_interface.h>
#include<geometry_msgs/Point.h>
#include<string>
#include<xm_msgs/xm_Look.h>

namespace xm_simple_head_controller {

class XmSimpleHeadController : public controller_interface::Controller<hardware_interface::HeadServoInterface>
{

public:
    XmSimpleHeadController();

    bool init(hardware_interface::HeadServoInterface * hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

    void update(const ros::Time &time, const ros::Duration &period);

    void starting(const ros::Time& Time);

    void stopping(const ros::Time& Time);


private:
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_,last_command_get_;
    ros::Subscriber head_to_subscriber_;
    ros::ServiceServer server;
    geometry_msgs::Point cmd_;
    std::string name_, frame_id_;
    double cmd_time_out_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_pub2_;

    double head_yaw_,head_pitch_;
    hardware_interface::HeadServoHandle handle_;

    bool cmdCallback(xm_msgs::xm_LookRequest &req, xm_msgs::xm_LookResponse &res);

};


PLUGINLIB_EXPORT_CLASS(xm_simple_head_controller::XmSimpleHeadController, controller_interface::ControllerBase)


}



#endif // XM_SIMPLE_HEAD_CONTROLLER_H
