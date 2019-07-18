

#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include "robotiq_2f_gripper_control/joint_state_publisher.hpp"
#include <math.h>  

#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>

//#include "robotiq_2f_gripper_control/robotiq_2f_gripper_serial_client.h"
//#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
//#include "robotiq_2f_gripper_control/robotiq_2f_hw_usb.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
/*
  Note that this code currently works only to control ONE 2F gripper
  attached to ONE network interface. If you want to add more grippers
  to the same network, you'll need to edit the source file.
*/

// Note that you will likely need to run the following on your binary:
// sudo setcap cap_net_raw+ep <filename>

typedef robotiq_2f_gripper_control::Robotiq2FGripper_robot_output GripperOutput;
typedef robotiq_2f_gripper_control::Robotiq2FGripper_robot_input GripperInput;


GripperInput global_input;


void changeCallback(const GripperOutput::ConstPtr& msg)
{
	global_input.gACT = (*msg).rACT;
        global_input.gPR = (*msg).rPR;
        global_input.gPO = global_input.gPR;
        global_input.gOBJ = 0x02;
        global_input.gGTO = 0x01;
}




int main(int argc, char** argv)
{
  //using robotiq_ethercat::EtherCatManager;
  //using ROBOTIQ2FUSB;
  //using robotiq_2f_gripper_control::Robotiq2FGripperSerialClient;

  //typedef Robotiq2FGripperSerialClient::GripperOutput GripperOutput;
  //typedef Robotiq2FGripperSerialClient::GripperInput GripperInput;

  ros::init(argc, argv, "robotiq_2f_gripper_simulation_node");
  
  ros::NodeHandle nh ("~");



  HectorJointStatePublisher jp(nh);
  double j_pos;
  double max_joint_pos = 0.786522656;
  double max_gap = 0.140;
  double theta_offset = acos(0.06691/0.1);
  // The max position returned from the encoder is about 230/255 for 2f-140 with default fingertip pad and closed to rPR=255
  //double fake_max_joint_pos = max_joint_pos*255/(228-3);



  global_input.gACT = 0x01;
  global_input.gGTO = 0x01;
  global_input.gSTA = 0x03;
  global_input.gOBJ = 0x00;
  global_input.gFLT = 0x00;
  global_input.gPR  = 0x00;
  global_input.gPO  = 0x00;
  global_input.gCU  = 0x00;




  ros::Subscriber sub = 
        nh.subscribe<GripperOutput>("output", 1, changeCallback);
  ros::Publisher pub = nh.advertise<GripperInput>("input", 100);

  ros::Rate rate(20); // 10 Hz







  while (ros::ok()) 
  {
    GripperInput input = global_input;

	if (input.gOBJ > 0x00) {
            global_input.gOBJ = 0x00;
        }


    pub.publish(input);
    
    // TODO add functions because same numbers are in robotiq_2f_gripper_action_server.cpp
    if (input.gPO <= 14) { 
 	j_pos = 0;
    }
    else if (input.gPO <= 77) {  
	j_pos = ((double)input.gPO - 245.66409)/-1661.8310875; // gives current gap
        j_pos = acos((j_pos/2 + 0.00735 - 0.0127)/0.1) - theta_offset;
    }
    else { 
	j_pos = ((double)input.gPO - 226.84558)/-1479.31;
        j_pos = acos((j_pos/2 + 0.00735 - 0.0127)/0.1) - theta_offset;  // TODO add method in another file
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
