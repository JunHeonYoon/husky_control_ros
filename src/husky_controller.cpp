#include "husky_controller.h"


controller_interface::controller_interface(ros::NodeHandle nh_,double hz_):
    rate_(hz_)
  {    
    is_first_run = true;
    tick = 0;
    sim_step_done_ = false;
    sim_time_ = 0.0f;
    pose_.resize(3);
    pose_.setZero();
    velocity_.resize(2);
    velocity_.setZero();
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.linear.z = 0;
    cmd_vel_.angular.x = 0;
    cmd_vel_.angular.y = 0;
    cmd_vel_.angular.z = 0;
    
    vrep_sim_start_pub_ = nh_.advertise<std_msgs::Bool>("/startSimulation", 5);
    vrep_sim_stop_pub_ = nh_.advertise<std_msgs::Bool>("/stopSimulation", 5);
    vrep_sim_step_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/triggerNextStep", 100);
    vrep_sim_enable_syncmode_pub_ = nh_.advertise<std_msgs::Bool>("/enableSyncMode", 5);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 4);

    vrep_sim_step_done_sub_ = nh_.subscribe("/simulationStepDone", 100, &controller_interface::sim_step_done_cb, this);
    odom_subscriber_ = nh_.subscribe("/husky_velocity_controller/odom", 1, &controller_interface::odomCallback, this);
    vrep_sim_time_sub_ = nh_.subscribe("/simulationTime",100,&controller_interface::sim_time_cb,this);
    vrep_sim_status_sub_ = nh_.subscribe("/simulationState",100,&controller_interface::sim_status_cb,this);
  }
  controller_interface::~controller_interface()
  {
  }


  void controller_interface::set_exec_time(float t)
  {
    exec_time_ = t;
  }

  void controller_interface::odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    pose_(0) = msg->pose.pose.position.x;
    pose_(1) = msg->pose.pose.position.y;
    pose_(2) = tf::getYaw(msg->pose.pose.orientation);

    velocity_(0) = msg->twist.twist.linear.x;
    velocity_(1) = msg->twist.twist.angular.z;
  }

  void controller_interface::Getstate(Eigen::VectorXd& pose, Eigen::VectorXd& velocity)
  {
    pose(0) = pose_(0);
    pose(1) = pose_(1);
    pose(2) = pose_(2);

    velocity(0) = velocity_(0);
    velocity(1) = velocity_(1);
  }

  void controller_interface::SetInitialState()
  {
    pose_.Zero(3);
    velocity_.Zero(2);
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.linear.z = 0;
    cmd_vel_.angular.x = 0;
    cmd_vel_.angular.y = 0;
    cmd_vel_.angular.z = 0;
  }

  void controller_interface::sim_status_cb(const std_msgs::Int32ConstPtr& msg)
  {
    vrep_sim_status = msg->data;
  }

  void controller_interface::read_vrep()
  {
    ros::spinOnce();
  }
  
  void controller_interface::write_vrep(Eigen::VectorXd desired_velocity)
  {
    cmd_vel_.linear.x = desired_velocity(0);
    cmd_vel_.angular.z = desired_velocity(1);

    velocity_pub_.publish(cmd_vel_);
    vrepStepTrigger();
  }
  void controller_interface::wait()
  {
    while(ros::ok() && !sim_step_done_)
    {
      ros::spinOnce();
    }
    sim_step_done_ = false;
    rate_.sleep();
  }

  void controller_interface::sim_time_cb(const std_msgs::Float32ConstPtr& msg)
  {
    sim_time_ = msg->data;
    tick = (sim_time_*100)/(SIM_DT*100);    
  }

  void controller_interface::sim_step_done_cb(const std_msgs::BoolConstPtr &msg)
  {
    sim_step_done_ = msg->data;
  }

  void controller_interface::vrepStart()
  {
    ROS_INFO("Starting V-REP Simulation");
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_start_pub_.publish(msg);
  }

  void controller_interface::vrepStop()
  {
    ROS_INFO("Stopping V-REP Simulation");
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_stop_pub_.publish(msg);
  }

  void controller_interface::vrepStepTrigger()
  {
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_step_trigger_pub_.publish(msg);
  }

  void controller_interface::vrepEnableSyncMode()
  {
    ROS_INFO("Sync Mode On");
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_enable_syncmode_pub_.publish(msg);
  }

