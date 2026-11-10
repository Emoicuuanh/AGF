#ifndef MODULE_MANAGER_H
#define MODULE_MANAGER_H

#include <ros/ros.h>
#include <std_stamped_msgs/StringStamped.h>
#include <std_stamped_msgs/EmptyStamped.h>

namespace module_manager
{
class ModuleStatus
{
    public:
        int WAITING = 0;
        int RUNNING = 1;
        int PAUSED = 2;
        int ERROR = 3;
};

class ModuleManager
{
    public:

        ModuleManager(ros::NodeHandle nh, std::string name);

        void requestStartMissionCb(const std_stamped_msgs::StringStamped::ConstPtr &msg);

        void runPauseReqCb(const std_stamped_msgs::StringStamped::ConstPtr &msg);

        void resetErrorCb(const std_stamped_msgs::EmptyStamped::ConstPtr &msg);

        void resetFlag();

        void startROS();

        std::string name_;
        std::string error_code_;
        std::string module_state_;

        bool reset_action_req_;
        bool reset_error_request_;
        bool resume_req_;
        bool pause_req_;
        bool action_running_;

        int module_status_;

    private:
        ros::NodeHandle nh_;

        ros::Publisher moduleStatusPub_;

        ros::Subscriber runPauseReq_;
        ros::Subscriber resetError_;
        ros::Subscriber requestStartMission_;
        ModuleStatus status_;
    };
};
#endif