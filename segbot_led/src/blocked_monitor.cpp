#include <ctime>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <time.h>

/*******************************************************
*                    ROS Headers                       *
********************************************************/
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <tf/tf.h>

/*******************************************************
*                   segbot_led Headers                 *
********************************************************/
#include "bwi_msgs/LEDActionResult.h"
#include "bwi_msgs/LEDAnimations.h"
#include "bwi_msgs/LEDClear.h"
#include "bwi_msgs/LEDControlAction.h"

/*******************************************************
*                   Service Headers                    *
********************************************************/
#include "bwi_msgs/QuestionDialog.h"
#include "bwi_services/SpeakMessage.h"
#include "actionlib_msgs/GoalStatus.h"

#include <move_base/move_base.h>
#include <move_base_msgs/MoveBaseLogging.h>
#include <std_srvs/Empty.h>

/*******************************************************
*                 Global Variables                     *
********************************************************/
ros::Publisher signal_pub;

ros::Subscriber global_path;
ros::Subscriber robot_pose;
ros::Subscriber status;
ros::Subscriber robot_goal;
ros::Subscriber end_goal;

nav_msgs::Path current_path;
geometry_msgs::Pose current_pose;
geometry_msgs::Pose end_pose;


actionlib_msgs::GoalStatusArray r_goal;

bool heard_path = false;
bool heard_pose = false;
bool heard_goal = false;
bool block_detected = false;
double time_for_blocked = 0;
ros::Time startTime;
/*******************************************************
*                 Callback Functions                   *
********************************************************/
void path_cb(const nav_msgs::Path::ConstPtr& msg)
{
    current_path = *msg;
    heard_path = true;
}

// status code 1 moving in progress
// status code 4 is failed to get a plan
// status code 3 is code succeded
void status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr& msg_goal)
{
    r_goal = *msg_goal;
    heard_goal = true;
}

void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{


    //clock_t startTime = clock();
   // startTime = ros::Time::now();
   // if((ros::Time::now() - startTime) > ros::Duration(2.0)){
      //  ROS_INFO_STREAM("updated values within pose cb");
        geometry_msgs::PoseWithCovarianceStamped new_pose = *msg;
        current_pose = new_pose.pose.pose;
        heard_pose = true;
        //time_for_blocked = 0;
        //return;
    //}
    //clock_t endTime = clock();
    //clock_t clockTicksTaken = endTime - startTime;
    //time_for_blocked += clockTicksTaken / (double) CLOCKS_PER_SEC;
}

void end_goal_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
    geometry_msgs::Pose new_pose = *msg;
    end_pose = new_pose;
    heard_pose = true;
}

double getDistance(){
    return sqrt(pow((end_pose.position.x - current_pose.position.x),2) + pow((end_pose.position.y - current_pose.position.y),2));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blocked_monitor");
    ros::NodeHandle n;

    ros::Rate loop_rate(30);
    ros::Rate inner_rate(100);
    ros::Rate recovered_check(2);

    srand(time(NULL));
    time_t now = time(0);
    int old_count = 0;
    std::ofstream log_file;
    std::string log_filename = ros::package::getPath("led_study") + "/data/" + "blocked_state.csv";

    // Sets up service clients
    ros::ServiceClient speak_message_client = n.serviceClient<bwi_services::SpeakMessage>("/speak_message_service/speak_message");
    bwi_services::SpeakMessage speak_srv;

    ros::ServiceClient init_count_client = n.serviceClient<std_srvs::Empty>("move_base/init_replan_count");
    std_srvs::Empty init_count_srv;

    ros::ServiceClient get_count_client = n.serviceClient<move_base_msgs::MoveBaseLogging>("/move_base/log_replan_count");
    move_base_msgs::MoveBaseLogging get_count_srv;

    ros::ServiceClient gui_client = n.serviceClient<bwi_msgs::QuestionDialog>("question_dialog");
    bwi_msgs::QuestionDialog gui_srv;

    // Sets up subscribers
    global_path = n.subscribe("/move_base/GlobalPlanner/plan", 1, path_cb);
    robot_pose = n.subscribe("/amcl_pose", 1, pose_cb);
    robot_goal = n.subscribe("/move_base/status", 1, status_cb);
    end_goal = n.subscribe("/led_study/blocked_goal", 1, end_goal_cb);

    // Sets up action get_count_client
    actionlib::SimpleActionClient<bwi_msgs::LEDControlAction> ac("led_control_server", true);
    ac.waitForServer();
    int prevReplanCount = 0;
    double distanceToGoal;
    bwi_msgs::LEDControlGoal goal;
    double check = 0;
    distanceToGoal = getDistance();
    // Waits for current path and pose to update
    init_count_client.call(init_count_srv);
    startTime = ros::Time::now();
    while(!heard_path || !heard_pose)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    while(ros::ok())
    {
        // Updates current path and pose
        ros::spinOnce();
        get_count_client.call(get_count_srv);

        //ROS_INFO_STREAM(r_goal.status_list[0].status);
        if(heard_goal == true)
        {
            int randLED = rand()%2;
            int randSpeech = rand()%2;
            //TODO check if getting closer to goal by euclidian distance
            //Check constant if correct for variability
            //ROS_INFO_STREAM("distanceToGoal " << distanceToGoal);
            //ROS_INFO_STREAM("getDistance " << getDistance());
            ROS_INFO_STREAM("out of loop diff " << (distanceToGoal-getDistance()));
            ROS_INFO_STREAM("out of loop replan count " << get_count_srv.response.replan_count);
            ROS_INFO_STREAM("prev of loop replan count " << prevReplanCount);
            //test with replan count if reduces false positives and if improves preformance
            check = distanceToGoal-getDistance();
            old_count = prevReplanCount;
            ROS_INFO_STREAM("old cou mnt " << old_count);
            ROS_INFO_STREAM("--------------------------pose size: " << current_path.poses.size());
            while((current_path.poses.size() < 5 && (distanceToGoal > .1) || r_goal.status_list[0].status == 4) )//
            {
                //check = -check;
                //ROS_INFO_STREAM("distanceToGoal " << distanceToGoal);
                //ROS_INFO_STREAM("getDistance " << getDistance());
                //ROS_INFO_STREAM("difference in loop " << (check));
				distanceToGoal = getDistance();

                get_count_client.call(get_count_srv);

                if(randLED == 1)
                {
                    ROS_INFO_STREAM("want to use leds blocked loop");
                    if (!block_detected)
                    {
                        //init_count_client.call(init_count_srv);
                        //old_count = get_count_srv.response.replan_count;
                        ROS_INFO_STREAM("blocked using leds");
                        now = time(0);
                        tm *gmtm = gmtime(&now);
                        log_file.open(log_filename, std::ios_base::app | std::ios_base::out);
                        // state,led,speech,date,time
                        log_file << "start," << randLED << "," << randSpeech << "," << (1900 + gmtm->tm_year) << "-" << (1 + gmtm->tm_mon) << "-" << gmtm->tm_mday << "," << (1 + gmtm->tm_hour) << ":" << (1 + gmtm->tm_min) << ":" << (1 + gmtm->tm_sec) << std::endl;
                        log_file.close();
                        gui_srv.request.type = 0;
                        gui_srv.request.message = "My path is Blocked, please clear a path for me";
                        gui_client.call(gui_srv);
                        //hangs here when hardware fails
                        if(randSpeech)
                        {
                            speak_srv.request.message = "My path is Blocked, please clear a path for me";
                            speak_message_client.call(speak_srv);
                        }
                    }

                    goal.type.led_animations = bwi_msgs::LEDAnimations::BLOCKED;
                    goal.timeout = ros::Duration(8);
                    ac.sendGoal(goal);


                }
                else
                {
                    ROS_INFO_STREAM("dont want to use leds blocked loop");
                    if (!block_detected)
                    {
                        //init_count_client.call(init_count_srv);
                        //old_count = get_count_srv.response.replan_count;
                        ROS_INFO_STREAM("blocked not using leds");
                        now = time(0);
                        tm *gmtm = gmtime(&now);
                        ROS_INFO_STREAM("old count " << old_count);
                        log_file.open(log_filename, std::ios_base::app | std::ios_base::out);
                        // state,led,speech,date,time
                        log_file << "start," << randLED << "," << randSpeech << "," << (1900 + gmtm->tm_year) << "-" << (1 + gmtm->tm_mon) << "-" << gmtm->tm_mday << "," << (1 + gmtm->tm_hour) << ":" << (1 + gmtm->tm_min) << ":" << (1 + gmtm->tm_sec) << std::endl;
                        log_file.close();
                    }
                }
                block_detected = true;
                prevReplanCount = get_count_srv.response.replan_count;
                inner_rate.sleep();
                ros::spinOnce();
            }

            recovered_check.sleep();

            if (block_detected)
            {
                // End log
                ac.cancelAllGoals();

                gui_srv.request.type = 0;
                gui_srv.request.message = "";
                gui_client.call(gui_srv);
                block_detected = false;

                get_count_client.call(get_count_srv);
                ROS_INFO_STREAM("diff " << get_count_srv.response.recovery_count - old_count);
                get_count_client.call(get_count_srv);
                now = time(0);
                tm *gmtm = gmtime(&now);
                log_file.open(log_filename, std::ios_base::app | std::ios_base::out);
                // state,led,speech,date,time
                log_file << "end," << randLED << "," << randSpeech << "," << (1900 + gmtm->tm_year) << "-" << (1 + gmtm->tm_mon) << "-" << gmtm->tm_mday << "," << (1 + gmtm->tm_hour) << ":" << (1 + gmtm->tm_min) << ":" << (1 + gmtm->tm_sec) << "," << (get_count_srv.response.replan_count  - old_count) << "," << get_count_srv.response.recovery_count << std::endl;
                log_file.close();
                //init_count_client.call(init_count_srv);
            }
            heard_goal = false;
        }
        //init_count_client.call(init_count_srv);
        distanceToGoal = getDistance();
        prevReplanCount = get_count_srv.response.replan_count;
        //prevReplanCount = 0;
        loop_rate.sleep();
    }
    return 0;
}

