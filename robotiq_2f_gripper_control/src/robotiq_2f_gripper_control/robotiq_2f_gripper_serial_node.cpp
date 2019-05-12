

#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include "robotiq_2f_gripper_control/joint_state_publisher.hpp"


#include "robotiq_2f_gripper_control/robotiq_2f_gripper_serial_client.h"
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include "robotiq_2f_gripper_control/robotiq_2f_hw_usb.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
/*
  Note that this code currently works only to control ONE 2F gripper
  attached to ONE network interface. If you want to add more grippers
  to the same network, you'll need to edit the source file.
*/

// Note that you will likely need to run the following on your binary:
// sudo setcap cap_net_raw+ep <filename>


void changeCallback(robotiq_2f_gripper_control::Robotiq2FGripperSerialClient& client,
                    const robotiq_2f_gripper_control::Robotiq2FGripperSerialClient::GripperOutput::ConstPtr& msg)
{
  client.writeOutputs(*msg);
}




int main(int argc, char** argv)
{
  //using robotiq_ethercat::EtherCatManager;
  //using ROBOTIQ2FUSB;
  using robotiq_2f_gripper_control::Robotiq2FGripperSerialClient;

  typedef Robotiq2FGripperSerialClient::GripperOutput GripperOutput;
  typedef Robotiq2FGripperSerialClient::GripperInput GripperInput;

  ros::init(argc, argv, "robotiq_2f_gripper_node");
  
  ros::NodeHandle nh ("~");

  // Parameter names
  std::string port;
  int slave_no;
  bool activate;  

  nh.param<std::string>("port", port, "/dev/ttyUSB0");
  nh.param<int>("slave_number", slave_no, 9);
  nh.param<bool>("activate", activate, true);

  // Start Serial manager
  robotiq_2f_hardware::ROBOTIQ2FUSB manager;

  manager.setPort(port);
  manager.setServerID(slave_no);


  if(!manager.init())
  {
    ROS_FATAL_NAMED("robotiq_2f_hardware", "Could not initialize USB interface");
    return -1;
  }


  // register client 
  Robotiq2FGripperSerialClient client(manager, slave_no);

  // TODO delete
  GripperOutput test;
	
  test = client.readOutputs();
  ROS_WARN("rACT %d", test.rACT);
  ROS_WARN("rGTO %d", test.rGTO);
  ROS_WARN("rATR %d", test.rATR);
  ROS_WARN("rPR %d", test.rPR);
  ROS_WARN("rSP %d", test.rSP);
  ROS_WARN("rFR %d", test.rFR);

 GripperInput test_i;
	
  test_i = client.readInputs();
  ROS_WARN("gACT %d", test_i.gACT);
  ROS_WARN("gGTO %d", test_i.gGTO);
  ROS_WARN("gSTA %d", test_i.gSTA);
  ROS_WARN("gOBJ %d", test_i.gOBJ);
  ROS_WARN("gFLT %d", test_i.gFLT);
  ROS_WARN("gPR %d", test_i.gPR);
  ROS_WARN("gPO %d", test_i.gPO);
  ROS_WARN("gCU %d", test_i.gCU);

  // conditionally activate the gripper
  if (activate)
  {
    // Check to see if resetting is required? Or always reset?
ROS_WARN("ACTIVATE");
    GripperOutput out;
    out.rACT = 0x0;
    //out.rGTO = 0x1;
    //out.rPR = test_i.gPR;

    client.writeOutputs(out);
   out.rACT = 0x1;
   //out.rGTO = 0x1;
client.writeOutputs(out);
  }
/*
 test = client.readOutputs();
  ROS_WARN("rACT %d", test.rACT);
  ROS_WARN("rGTO %d", test.rGTO);
  ROS_WARN("rATR %d", test.rATR);
  ROS_WARN("rPR %d", test.rPR);
  ROS_WARN("rSP %d", test.rSP);
  ROS_WARN("rFR %d", test.rFR);
*/

  HectorJointStatePublisher jp(nh);
  double j_pos;
  double max_joint_pos = 0.792;
  // The max position returned from the encoder is about 230/255 for 2f-140 with default fingertip pad and closed to rPR=255
  //double fake_max_joint_pos = max_joint_pos*255/(228-3);

  ros::Subscriber sub = 
        nh.subscribe<GripperOutput>("output", 1,
                                    boost::bind(changeCallback, boost::ref(client), _1));
  ros::Publisher pub = nh.advertise<GripperInput>("input", 100);

  ros::Rate rate(20); // 10 Hz

  while (ros::ok()) 
  {
    Robotiq2FGripperSerialClient::GripperInput input = client.readInputs();
    pub.publish(input);
    
    // TODO add functions because same numbers are in robotiq_2f_gripper_action_server.cpp
    if (input.gPO <= 14) { 
 	j_pos = 0;
    }
    else if (input.gPO <= 77) {  
	j_pos = ((double)input.gPO - 245.66409)/-1661.8310875;
        j_pos = (max_joint_pos*(j_pos-0.14))/-0.14;
    }
    else { 
	j_pos = ((double)input.gPO - 226.84558)/-1479.31;
        j_pos = (max_joint_pos*(j_pos-0.14))/-0.14;
    }


    // TODO validate when gPO = 3 (fully opened)
    //j_pos = (double)((input.gPO - 3)/225.0f)*max_joint_pos;
    if(j_pos > max_joint_pos) {
        j_pos = max_joint_pos;
    }
	else if (j_pos < 0) {
j_pos = 0;
}

    jp.updateJointStates(j_pos);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
