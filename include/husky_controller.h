#ifndef SIMULATION_INTERFACE_H
#define SIMULATION_INTERFACE_H

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Eigen>
#include <tf/tf.h>
#include <math.h>
#include <fstream>
#include <memory>
#include <iostream>


#define TOTAL_DOF 4 
const std::string JOINT_NAME[TOTAL_DOF] = {"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"};



    class controller_interface
    {
    public:
      controller_interface(ros::NodeHandle nh_,double hz_);
      ~controller_interface();
    /**
    * @brief joint value callback function
    * @detail get joint value from vrep simulator
    * @param msg : sensor_msgs::JointState
    */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    /**
    * @brief simulation callback function
    * @detail get simulation status from vrep simulator(0:sim not running, 1: sim paused, 2:sim stopped, 3:sim running)
    * @param msg : std_msgs::Int32
    */
    void sim_status_cb(const std_msgs::Int32ConstPtr& msg);
    /**
    * @brief vrep simulation time callback function
    * @detail To get vrep simulation time. time format is float32
    */
    void sim_time_cb(const std_msgs::Float32ConstPtr& msg);
    /**
    * @brief vrep simulation start function
    * @detail publish start topic message to vrep simulator
    */
    void vrepStart();
    /**
    * @brief vrep simulation stop function
    * @detail publish stop topic message to vrep simulator
    */
    void vrepStop();
    /**
    * @brief vrep simulation step trigger function
    * @detail update vrep simulation by trigger. when user publish joint command one step updated in vrep simulation
    */
    void vrepStepTrigger();
    /**
    * @brief vrep simulation sync mode function
    * @detail To match vrep simulation and controller step by step.
    */
    void vrepEnableSyncMode();
    /**
    * @brief vrep simulation sync mode function
    * @detail To match vrep simulation and controller step by step.
    */
    void sim_step_done_cb(const std_msgs::BoolConstPtr& msg);

    /**
    * @brief This is our joint control framework for husky mobile robot
    * @param read_vrep : Read sensor from V-REP. Do spinonce to update callback function.
    * @param write_vrep : Changing our joint value type to V-REP form. Usually Triggering is done by this function.
    * @param wait : For syncmode... this function is nessesary to sync controller and V-REP simulation environment.
    */
    void set_exec_time(float t);
    void read_vrep();
    void write_vrep(Eigen::VectorXd desired_velocity);
    void wait();
    void Getstate(Eigen::VectorXd& pose, Eigen::VectorXd& velocity);
    void SetInitialState();

    private:
    /**
    * @brief controller to vrep simulator publisher message lists
    * @param pub_velocity(geometry_msgs::Twist) : controller joint value --> vrep simulator robot joint value
    * @param vrep_sim_start_pub_(std_msgs::Bool) : vrep simulation start command
    * @param vrep_sim_stop_pub_(std_msgs::Bool) : vrep simulation stop command
    * @param vrep_sim_step_trigger_pub_(std_msgs::Bool) : vrep simulation update one step command
    * @param vrep_sim_enable_syncmode_pub_(std_msgs::Bool) : vrep simulation enable syncmode command
    */
    ros::Publisher velocity_pub_;
    ros::Publisher vrep_sim_start_pub_;
    ros::Publisher vrep_sim_stop_pub_;
    ros::Publisher vrep_sim_step_trigger_pub_;
    ros::Publisher vrep_sim_enable_syncmode_pub_;

    /**
    * @brief controller to vrep simulator subscriber message lists
    * @param sub_state_(nav_mgs::Odometry) : get vrep simulator robot current state value
    * @param vrep_sim_step_done_sub_(std_msgs::Bool) : get simulation trigger step done signal
    * @param vrep_sim_time_sub_(std_msgs::Float32) : get vrep simulator time tick
    * @param vrep_sim_status_sub_(std_msgs::Int32) : get vrep simulation status(0:sim not running, 1: sim paused, 2:sim stopped, 3:sim running)
    */
    ros::Subscriber odom_subscriber_;
    ros::Subscriber vrep_sim_step_done_sub_;
    ros::Subscriber vrep_sim_time_sub_;
    ros::Subscriber vrep_sim_status_sub_;

    /**
    * @brief Other parameters for using ros interface and control    
    */

    bool sim_step_done_;
    float sim_time_; // from v-rep simulation time
    int tick;
    int tick_init;
    ros::Rate rate_;
    Eigen::VectorXd pose_;  // x, y, theta
    Eigen::VectorXd velocity_;  // x_dot, y_dot, theta_dot
    geometry_msgs::Twist cmd_vel_;  // v, w
    bool is_first_run;
    double start_time;
    double current_time;
    double final_time;    
    int vrep_sim_status;
    float exec_time_;
    float hz_;

    constexpr static int NUM_FILES{10};
    std::ofstream files_[NUM_FILES];
    const std::string file_names_[NUM_FILES]
    {"BULLET_CYLINDER", "ODE_CYLINDER","VOLTEX_CYLINDER", "NEWTON_CYLINDER",
     "BULLET_SPHERE", "ODE_SPHERE", "VOLTEX_SPHERE", "NEWTON_SPHERE"};

    void initFile();
    void record(int file_number, double duration);
    };

#endif
