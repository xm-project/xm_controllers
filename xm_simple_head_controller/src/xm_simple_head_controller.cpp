#include<xm_simple_head_controller/xm_simple_head_controller.h>
#include<boost/assign.hpp>
#include<tf/transform_datatypes.h>
#include<tf/tfMessage.h>
#include<cmath>
#define PI  3.1415926
namespace xm_simple_head_controller {

XmSimpleHeadController::XmSimpleHeadController()
    :  frame_id_("camera_link") ,name_("xm_head")
{
    cmd_.x = 0.0;
    cmd_.y = 0.0;
    cmd_.z = 0.0;
}

bool XmSimpleHeadController::init(hardware_interface::HeadServoInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
    head_yaw_=0.0;
    head_pitch_=0.0;
    double publish_rate = 20.0;
    controller_nh.getParam("publish_rate",publish_rate);
    publish_period_ =ros::Duration(1.0 / publish_rate);
    last_command_get_ = ros::Time::now();

    controller_nh.getParam("frame_id",frame_id_);
    controller_nh.getParam("name",name_);

    handle_ = hw->getHandle("xm_head");

    tf_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh,"/tf",100));
    tf_pub_->msg_.transforms.resize(1);
    tf_pub_->msg_.transforms[0].child_frame_id = "camera_link";
    tf_pub_->msg_.transforms[0].header.frame_id = "base_link";
    server = controller_nh.advertiseService("look",&XmSimpleHeadController::cmdCallback,this);
    return true;
}

bool XmSimpleHeadController::cmdCallback(xm_msgs::xm_LookRequest &req, xm_msgs::xm_LookResponse &res)
{
    if(isRunning())
    {
        cmd_ = req.pos;
        last_command_get_ = ros::Time::now();
        res.result =true;
        ros::Duration(1.0).sleep();
        return true;
    }
    else{
        res.result = false;
        return false;
        ROS_ERROR_NAMED(name_,"Can't accept new command. Controller is not running. ");
    }
}

void XmSimpleHeadController::update(const ros::Time &time, const ros::Duration &period)
{
    if(last_state_publish_time_ + publish_period_ <time)
    {
        last_state_publish_time_+= publish_period_;
	//ROS_INFO("current theta  yaw: %d  pitch: %d ",handle_.getHeadYaw(),handle_.getHeadPitch());
         geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw (0,handle_.getHeadPitch() ,handle_.getHeadYaw());
        if(tf_pub_->trylock())
        {
            geometry_msgs::TransformStamped& head_frame = tf_pub_->msg_.transforms[0];
            head_frame.header.stamp = time;
            head_frame.transform.translation.x = 0.13*sin(handle_.getHeadPitch()) -0.105;
            head_frame.transform.translation.y = 0.0;
            head_frame.transform.translation.z = 1.11+0.13*cos(handle_.getHeadPitch());
            head_frame.transform.rotation = orientation;
            tf_pub_->unlockAndPublish();
        }
    }
    handle_.setYawcmd(cmd_.z);
    handle_.setPitchcmd(cmd_.y);

}

void XmSimpleHeadController::starting(const ros::Time &Time)
{
    handle_.setYawcmd(0.0);
    handle_.setPitchcmd(0.0);
    last_state_publish_time_ =Time;
    state_=RUNNING;
}

void XmSimpleHeadController::stopping(const ros::Time &Time)
{
    handle_.setYawcmd(0.0);
    handle_.setPitchcmd(0.0);
}
}
