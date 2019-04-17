#ifndef ROBOTIQ_2F_HW_USB____JOINT_STATE_PUBLISHER_H
#define ROBOTIQ_2F_HW_USB____JOINT_STATE_PUBLISHER_H

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>

class HectorJointStatePublisher{
public:
  HectorJointStatePublisher(ros::NodeHandle nh)
  {
    joint_name_vector.push_back("left_outer_knuckle_joint");

    out_joint_state_.name.resize(joint_name_vector.size());
    out_joint_state_.position.resize(joint_name_vector.size());


    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states",5);

    for (size_t i = 0; i < joint_name_vector.size(); ++i){
      //dyn_joint_state_subs_[i] = nh.subscribe("/" + joint_name_vector[i] + "/state", 1, &HectorJointStatePublisher::dynamixelJointStateCallback, this);
      //ros::Subscriber sub = nh.subscribe("/" + joint_name_vector[i] + "/state", 1, &HectorJointStatePublisher::dynamixelJointStateCallback, this);
      //dyn_joint_state_subs_.push_back(nh.subscribe("/" + joint_name_vector[i] + "/state", 1, &HectorJointStatePublisher::dynamixelJointStateCallback, this));
      out_joint_state_.name[i] = joint_name_vector[i];
      out_joint_state_.position[i] = 0.0;
      map_name_to_index_[joint_name_vector[i]] = i;
    }
  }

  ~HectorJointStatePublisher()
  {

  }
/*
  void dynamixelJointStateCallback(const dynamixel_msgs::JointState dyn_joint_state)
  {
    out_joint_state_.position[ map_name_to_index_[dyn_joint_state.name] ] = dyn_joint_state.current_pos;
  }
*/
  /*
  void timerPublishJointStates(const ros::TimerEvent& e)
  {
    out_joint_state_.header.stamp = ros::Time::now();
    joint_state_pub_.publish(out_joint_state_);
  }
*/
  void updateJointStates(double pos)
  {
    out_joint_state_.position[0] = pos;

    out_joint_state_.header.stamp = ros::Time::now();
    joint_state_pub_.publish(out_joint_state_);
  }
protected:
  std::vector<std::string> joint_name_vector;
  ros::Publisher joint_state_pub_;
  std::map<std::string, int> map_name_to_index_;

  sensor_msgs::JointState out_joint_state_;
};
#endif
