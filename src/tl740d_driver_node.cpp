/*
 * ros driver node for TL740D imu
 * guolindong@gmail.com
 * 2018.01.23
 */

#include <math.h>
#include <iostream>

#include <ros/ros.h>
#include <serial/serial.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include "tf/LinearMath/Quaternion.h"

#include "tl740d.h"


struct SAcc     stcAcc;
struct SGyro    stcGyro;
struct SAngle   stcAngle;

unsigned char chrTemp[5000];
unsigned char ucRxCnt = 0;
unsigned int usRxLength = 0;

void DoParse() {
  static int temp_sign;
  static int temp_tenths;
  static int temp_hunderedths;
  static int temp_ones;
  static int temp_tens;
  static int temp_hundereds;
  static int temp_thousandths;
  int count;

  std::cout << std::fixed << std::setprecision(8);
  switch(chrTemp[3]) {
    case 0x84: {
      //angle
      count=0;
      for(int i=4;i<=10;i+=3)
      {
        temp_sign = static_cast<int>(chrTemp[i]>>4);
        temp_hundereds = static_cast<int>(chrTemp[i] & 0x0F);
        temp_tens = static_cast<int>(chrTemp[i+1]>>4);
        temp_ones = static_cast<int>(chrTemp[i+1] & 0x0F);
        temp_tenths = static_cast<int>(chrTemp[i+2]>>4);
        temp_hunderedths = static_cast<int>(chrTemp[i+2] & 0x0F);
        if(temp_sign==0)
        {
          stcAngle.Angle[count] = static_cast<double>((temp_hundereds*100+temp_tens*10+temp_ones*1+temp_tenths*0.1+temp_hunderedths*0.01)/180*M_PI);
        }
        else
        {
          stcAngle.Angle[count] = (-1)*static_cast<double>((temp_hundereds*100+temp_tens*10+temp_ones*1+temp_tenths*0.1+temp_hunderedths*0.01)/180*M_PI);
        }
        count++;
      }
      //right_handed_coordinate_system
      //stcAngle.Angle[1]=(-1)*stcAngle.Angle[1];
      //std::cout << "Temp: " << "Angle: " << stcAngle.Angle[0] << ", " << stcAngle.Angle[1] << ", " << stcAngle.Angle[2] << std::endl;
      
      //acc
      count=0;
      for(int i=13;i<=19;i+=3)
      {
        
        temp_sign = static_cast<int>(chrTemp[i]>>4);
        temp_tens = static_cast<int>(chrTemp[i] & 0x0F);
        temp_ones = static_cast<int>(chrTemp[i+1]>>4);
        temp_tenths = static_cast<int>(chrTemp[i+1] & 0x0F);
        temp_hunderedths = static_cast<int>(chrTemp[i+2]>>4);
        temp_thousandths = static_cast<int>(chrTemp[i+2] & 0x0F);
        if(temp_sign==0)
        {
          stcAcc.a[count] = static_cast<double>((temp_tens*10+temp_ones*1+temp_tenths*0.1+temp_hunderedths*0.01+temp_thousandths*0.001)*9.80665);
        }
        else
        {
          stcAcc.a[count] = (-1)*static_cast<double>((temp_tens*10+temp_ones*1+temp_tenths*0.1+temp_hunderedths*0.01+temp_thousandths*0.001)*9.80665);
        }
        count++; 
      }
      //std::cout << "Temp: " << "Acc: " << stcAcc.a[0] << ", " << stcAcc.a[1] << ", " << stcAcc.a[2] << std::endl;
      // Gyro
      count=0;
      for(int i=22;i<=28;i+=3)
      {
        
        temp_sign = static_cast<int>(chrTemp[i]>>4);
        temp_hundereds = static_cast<int>(chrTemp[i] & 0x0F);
        temp_tens = static_cast<int>(chrTemp[i+1]>>4);
        temp_ones = static_cast<int>(chrTemp[i+1] & 0x0F);
        temp_tenths = static_cast<int>(chrTemp[i+2]>>4);
        temp_hunderedths = static_cast<int>(chrTemp[i+2] & 0x0F);
        if(temp_sign==0)
        {
          stcGyro.w[count] = static_cast<double>((temp_hundereds*100+temp_tens*10+temp_ones*1+temp_tenths*0.1+temp_hunderedths*0.01)/180*M_PI);
        }
        else
        {
          stcGyro.w[count] = (-1)*static_cast<double>((temp_hundereds*100+temp_tens*10+temp_ones*1+temp_tenths*0.1+temp_hunderedths*0.01)/180*M_PI);
        }
        count++; 
      } 

     //std::cout << "Temp: " << "Gyro: " << stcGyro.w[0] << ", " << stcGyro.w[1] << ", " << stcGyro.w[2] << std::endl; 
      break;
    }
  }
}

// convert serial data to tl740d data
void CopeSerialData(std::string str_in) {
  unsigned int str_length = str_in.size();

  static int sum;
  memcpy(chrTemp+usRxLength, str_in.data(), str_length);
  usRxLength += str_length;
  while (usRxLength >= 32) {
    if (chrTemp[0] != 0x68) {
      usRxLength--;
      memcpy(&chrTemp[0], &chrTemp[1], usRxLength);
    } else {
      sum = 0;
      for(int i=1; i<31; i++) {
        sum += chrTemp[i];

      }
      if((int(sum & 0x0000ff)==int(chrTemp[31]))) {

        DoParse();
        usRxLength -= 32;
        memcpy(&chrTemp[0], &chrTemp[32], usRxLength);
      } else {
        usRxLength--;
        memcpy(&chrTemp[0], &chrTemp[1], usRxLength);
      }
    }
  }
}

int main (int argc, char** argv) {
  // param
  serial::Serial serial_port;
  std::string port;
  int baudrate;
  int looprate;

  // ros init
  ros::init(argc, argv, "tl740d_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get param from launch file
  pnh.param<int>("baudrate", baudrate, 115200);
  pnh.param<std::string>("port", port, "/dev/tl740d");
  pnh.param<int>("looprate", looprate, 100);

  pnh.getParam("baudrate", baudrate);
  pnh.getParam("port", port);
  pnh.getParam("looprate", looprate);

  ROS_INFO_STREAM(port);
  ROS_INFO_STREAM(baudrate);
  ROS_INFO_STREAM(looprate);

  // ros pub and sub
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 100);
  ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps/fix", 100);

  try {
    serial_port.setPort(port);
    serial_port.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(10);
    serial_port.setTimeout(to);
    serial_port.open();
    serial_port.setRTS(false);
    serial_port.setDTR(false);
    // serial_port.open();
  } catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open serial port ");
    return -1;
  }

  // check if serial port is open
  if(serial_port.isOpen()) {
    ROS_INFO_STREAM("Serial Port initialized");
  } else {
    return -1;
  }

  ros::Rate loop_rate(looprate);
  while(ros::ok()) {
    // convert serial string to tl740d data
    CopeSerialData(serial_port.read(200));

    //std::cout << serial_port.read(200) << std::endl;

    // imu sensor msg pub
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_link";

    tf::Quaternion quate;
    quate.setRPY(stcAngle.Angle[1], stcAngle.Angle[0], stcAngle.Angle[2]);
    imu_msg.orientation.w = quate.w();
    imu_msg.orientation.x = quate.x();
    imu_msg.orientation.y = quate.y();
    imu_msg.orientation.z = quate.z();
    imu_msg.orientation_covariance[0] = 0;
    imu_msg.linear_acceleration.x = stcAcc.a[0];
    imu_msg.linear_acceleration.y = stcAcc.a[1];
    imu_msg.linear_acceleration.z = stcAcc.a[2];
    imu_msg.linear_acceleration_covariance[0] = 0;
    imu_msg.angular_velocity.x = static_cast<float>(stcGyro.w[0]);
    imu_msg.angular_velocity.y = static_cast<float>(stcGyro.w[1]);
    imu_msg.angular_velocity.z = static_cast<float>(stcGyro.w[2]);
    imu_msg.angular_velocity_covariance[0] = 0;
    imu_pub.publish(imu_msg);
    
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}
