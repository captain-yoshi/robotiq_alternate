//
// Created by bob on 10/07/19.
//

#ifndef ROBOTIQ_FT_SENSOR_ROBOTIQ_FTS300_HARDWARE_INTERFACE_H
#define ROBOTIQ_FT_SENSOR_ROBOTIQ_FTS300_HARDWARE_INTERFACE_H



#include <ros/ros.h>
#include <hardware_interface/force_torque_sensor_interface.h>









/*
#include <ros/ros.h>
#include <hardware_interface/force_torque_sensor_interface.h>

class RobotiqForceTorqueSensorInterface : public hardware_interface::ForceTorqueSensorInterface {
public:


    RobotiqForceTorqueSensorInterface(std::string fts_tcp_link)
    {

        registerHandle(hardware_interface::ForceTorqueSensorHandle(INTERFACE_NAME, fts_tcp_link, ft_tcp_.begin(), ft_tcp_.begin() + 3));


    }

    void update(RTShared &packet)
    {

    }


private:
    std::array<double, 6> ft_tcp_;  // force and torque at tcp point
    const std::string INTERFACE_NAME = "RobotiqForceTorqueSensorHW";


};
*/
















#endif //ROBOTIQ_FT_SENSOR_ROBOTIQ_FTS300_HARDWARE_INTERFACE_H
