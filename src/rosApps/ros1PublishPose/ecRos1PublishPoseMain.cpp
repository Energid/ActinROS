//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1PublishPoseMain.cpp
/// @brief main() for ROS1 publish pose example.
//
//------------------------------------------------------------------------------
#include <foundCore/ecApplication.h>
#include <systemSimulation/ecSysSimulation.h>
#include <xmlReaderWriter/ecXmlObjectReaderWriter.h>
#include <cmath>
#include <ros/ros.h>
#include <boost/algorithm/string/replace.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace
{

const EcString FORWARD_SLASH = "/";

} // Anonymous namespace

int main(int argc, char* argv[])
{
   EC_DEBUG_FLAGS;

   // Find simulation file.
   const EcString directory = Ec::Application::getDataDirectory() +  EcString("/actin/pa10");
   const EcString simulationFile = Ec::Application::findFile(directory + EcString("/pa10.ecz"));
   if (simulationFile == "")
   {
      EcPrint(Error) << "File pa10.ecz not found in data directory: " << directory << std::endl;
      return 1;
   }

   // Get end-effector topic name.
   EcSystemSimulation systemSimulation;
   EcString pa10TopicName;
   EcPrint(None) << "Reading simulation: " << simulationFile << std::endl;
   if (EcXmlObjectReaderWriter::readFromFile(systemSimulation, simulationFile))
   {
      systemSimulation.setActiveEndEffectorSet(0,0);

      pa10TopicName = systemSimulation.statedSystem().system().manipulators()[0].manipulatorLabel();
      pa10TopicName += FORWARD_SLASH;
      pa10TopicName += systemSimulation.endEffectorSet(0).endEffectors()[0].linkIdentifier();
      pa10TopicName += FORWARD_SLASH;
      pa10TopicName += systemSimulation.endEffectorSet(0).endEffectors()[0].token().token();
      // ROS can't have spaces, dashes, or periods in a topic name.
      boost::replace_all(pa10TopicName, "-", "_");
      boost::replace_all(pa10TopicName, " ", "_");
      boost::replace_all(pa10TopicName, ".", "_");
      EcPrint(None) << "PA10 end-effector topic name: " << pa10TopicName << std::endl;
   }
   else
   {
      EcPrint(Error) << "Error reading : " << simulationFile << std::endl;
      return 1;
   }

   ros::init(argc, argv, "EcRosPublishPose");
   ros::NodeHandle n;
   EcReal time = 0.0;
   const EcReal end = 50.0;
   const EcReal steps = 5000.0;
   const EcReal timestep = end/steps;
   const EcReal loops = 3.0;
   const EcReal radius = 0.2;
   ros::Rate r(1/timestep);
   geometry_msgs::PoseStamped::Ptr timeStampedPosePtr = boost::make_shared<geometry_msgs::PoseStamped>();
   timeStampedPosePtr->header.frame_id = pa10TopicName;
   timeStampedPosePtr->pose.orientation.w = 1.0;
   timeStampedPosePtr->pose.orientation.x = 0.0;
   timeStampedPosePtr->pose.orientation.y = 0.0;
   timeStampedPosePtr->pose.orientation.z = 0.0;
   ros::Publisher posePublisher = n.advertise<geometry_msgs::PoseStamped>(pa10TopicName, 1);

   // Spin
   while (n.ok())
   {
      while (time<end && n.ok())
      {
         const EcReal angle = Ec2Pi*loops*time/end;
         timeStampedPosePtr->pose.position.x = 0.3;
         timeStampedPosePtr->pose.position.y = radius*std::cos(angle);
         timeStampedPosePtr->pose.position.z = radius*std::sin(angle);
         timeStampedPosePtr->header.stamp = ros::Time::now();
         posePublisher.publish(timeStampedPosePtr);

         time=time+timestep;
         ros::spinOnce();
         r.sleep();
      }
      time = 0.0;
      EcSLEEPMS(3000); // wait 3 seconds before repeating the same motion
   }

   return 0;
}
