#include <agv_common_library/module_manager.h>

namespace module_manager
{
ModuleManager::ModuleManager(ros::NodeHandle nh, std::string name): nh_(nh), name_(name)
{
    moduleStatusPub_ = nh_.advertise<std_stamped_msgs::StringStamped>(name + "/module_status", 1);

    runPauseReq_ = nh_.subscribe(name_ + "/run_pause_req", 10, &ModuleManager::runPauseReqCb,this);
    resetError_ = nh_.subscribe(name_ + "/reset_error", 10, &ModuleManager::resetErrorCb,this);
    requestStartMission_ = nh_.subscribe("/request_start_mission", 10, &ModuleManager::requestStartMissionCb,this);

    reset_action_req_ = false;
    reset_error_request_ = false;
    resume_req_ = false;
    pause_req_ = false;
    action_running_ = false;
    module_state_ = "";
    module_status_ = status_.WAITING;
}

void ModuleManager::runPauseReqCb(const std_stamped_msgs::StringStamped::ConstPtr &msg)
{
    if (msg->data == "RUN"){
        ROS_INFO("Resume request");
        resume_req_ = true;
        pause_req_ = false;
    }
    else if (msg->data == "PAUSE"){
        ROS_INFO("Pause request");
        resume_req_ = false;
        pause_req_ = true;
    }
}

void ModuleManager::resetErrorCb(const std_stamped_msgs::EmptyStamped::ConstPtr &msg)
{
    reset_error_request_ = true;
}

void ModuleManager::requestStartMissionCb(const std_stamped_msgs::StringStamped::ConstPtr &msg)
{
    if (msg->data == "STOP"){
        ROS_INFO("STOP MISSION");
        reset_action_req_ = true;
    }
}

void ModuleManager::resetFlag()
{
    pause_req_ = false;
    resume_req_ = false;
    reset_action_req_ = false;
    reset_error_request_ = false;
    error_code_ = "";
}

void ModuleManager::startROS()
{
    ros::spinOnce();
}
}