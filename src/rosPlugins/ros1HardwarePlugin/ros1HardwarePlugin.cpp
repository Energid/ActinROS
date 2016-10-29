//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1HardwarePlugin.cpp
//
//------------------------------------------------------------------------------
#include "ros1HardwarePlugin.h"
#include "ecRos1HardwarePluginData.h"
#include "ecRos1HardwareConfig.h"
#include <control/ecPosContSystem.h>
#include <plugins/ecIOParams.h>
#include <ros/ros.h>
#include <ros1Tools/ecRos1Tools.h>
#include <tf/tf.h>
#include <boost/algorithm/string/replace.hpp>
#include <boost/numeric/conversion/cast.hpp>

EC_PLUGIN_STUB_DEFAULT(ros1HardwarePlugin)

// These are convenience macros when overloading the arm data
#define DATA            m_Data.element()
#define ROS1HW_DATA     static_cast<EcRos1HardwarePluginData*>(const_cast<Ec::HardwarePluginData*>(DATA))
#define ROS1HW_CDATA    static_cast<const EcRos1HardwarePluginData*>(DATA)

//------------------------------------------------------------------------------
ros1HardwarePlugin::ros1HardwarePlugin
   (
   ) :
Ec::HardwarePlugin(),
m_pNodeHandle(),
m_pPrivateNodeHandle(),
m_CurrentJointState(),
m_ActinJointStates(boost::make_shared<sensor_msgs::JointState>()),
m_JointStatesReader(),
m_ActinJointStatesWriter(),
m_JointPositions(),
m_JointVelocities(),
m_JointTorques(),
m_ActiveManipulator(),
m_PositionState(),
m_GravitationalTorqueTool()
{
   m_ConfigFilename = "ros1HardwareConfig.xml";

   // These calls switch out the default data with our subclassed version
   m_Data.ECXML_REGISTER_COMPONENT_CREATOR(EcRos1HardwarePluginData);
   m_Data.setElement(EcRos1HardwarePluginData());
   if (ROS1HW_CDATA->publish_frequency.value() != 0.0)
   {
      m_UpdatePeriodInMs = static_cast<EcU32>(1000.0/ROS1HW_CDATA->publish_frequency);
   }
}

//------------------------------------------------------------------------------
Ec::HardwareStatus
ros1HardwarePlugin::initImplementation
   (
   )
{
   EcPrint(Debug) << " EcRos1HardwarePluginBase::initImplementation " << std::endl;
   EcBoolean succeeded = EcRos1Tools::intialize();
   if (succeeded)
   {
      // Create node handles
      m_pPrivateNodeHandle = boost::make_shared<ros::NodeHandle>("~");
      m_pNodeHandle        = boost::make_shared<ros::NodeHandle>();

      return initRos() ? Ec::HardwareStatusOk : Ec::HardwareStatusGeneralError;
   }
   return Ec::HardwareStatusGeneralError;
}

//------------------------------------------------------------------------------
EcBoolean
ros1HardwarePlugin::initRos
  (
  )
{
   if (ROS1HW_CDATA->publish_frequency.value() == 0.0)
   {
      m_pPrivateNodeHandle->setParam("publish_frequency", param<Ec::SimulationThreadUpdateRate,EcReal>());
   }
   else
   {
      m_pPrivateNodeHandle->setParam("publish_frequency", ROS1HW_CDATA->publish_frequency.value());
      m_UpdatePeriodInMs = static_cast<EcU32>(1000.0/ROS1HW_CDATA->publish_frequency);
   }

   try
   {
      // Make queue size twice the number of manipulators.
      m_JointStatesReader = m_pNodeHandle
         ->subscribe(ROS1HW_CDATA->feedbackJointStatesTopicName,
                     boost::numeric_cast<uint32_t>(2 * param<Ec::Manipulator, EcU32>()),
                     &ros1HardwarePlugin::jointStatesCallback, this);
   }
   catch(ros::Exception error)
   {
      EcPrint(Error) << "Failed to create ROS subscriber with exception, " << error.what() << std::endl;
      return EcFalse;
   }
   catch (boost::numeric::bad_numeric_cast &error)
   {
      EcPrint(Error) << "Failed to create ROS subscriber due to bad numeric cast, " << error.what() << std::endl;
      return EcFalse;
   }

   // Publish joint_states.
   try
   {
      // Make queue size twice the number of manipulators.
      m_ActinJointStatesWriter = m_pNodeHandle
         ->advertise<sensor_msgs::JointState>(ROS1HW_CDATA->publishJointStatesTopicName,
         boost::numeric_cast<uint32_t>(2 * param<Ec::Manipulator, EcU32>()));
   }
   catch(ros::Exception error)
   {
      EcPrint(Error) << "Failed to create ROS publisher with exception, " << error.what() << std::endl;
      return EcFalse;
   }
   catch(boost::numeric::bad_numeric_cast& error)
   {
      EcPrint(Error) << "Failed to create ROS publisher due to bad numeric cast, " << error.what() << std::endl;
      return EcFalse;
   }

   m_ActinJointStates->header.frame_id = tf::resolve(ROS1HW_CDATA->rosManipulatorConfig.tf_prefix,
      ROS1HW_CDATA->rosManipulatorConfig.rosManipulatorLabel);
   m_ActinJointStates->name.clear();
   const Ec::HardwareConfigBaseVector& vDevices = hardwareConfig();
   m_ActinJointStates->position.resize(vDevices.size());
   m_ActinJointStates->velocity.resize(vDevices.size());
   m_CurrentJointState.position.resize(vDevices.size());
   torqueEnableImplementation(m_TorqueEnabled);
   getParam<Ec::JointAngle|Ec::HardwareUnits>(m_JointPositions);
   for (EcSizeT ii=0; ii<vDevices.size(); ++ii)
   {
      const EcRos1HardwareConfig& device = dynamic_cast<const EcRos1HardwareConfig&>(vDevices[ii]);
      m_ActinJointStates->name.push_back(device.rosLinkLabel);
      // Initialize feedback joint states to model in the event the joint states topic is not published.
      m_CurrentJointState.position[ii] = m_JointPositions[device.jointMapIndex];
   }

   // Setup gravitational torque tool.
   getParam<Ec::Manipulator>(manipIndex(),m_ActiveManipulator);
   m_GravitationalTorqueTool.setManipulator(&m_ActiveManipulator);
   m_GravitationalTorqueTool.setGravityVector(paramPtr<Ec::StatedSystem,EcStatedSystem>()->environment().upGravityVector());

   return EcTrue;
}

//------------------------------------------------------------------------------
void
ros1HardwarePlugin::updateDisabledImplementation
   (
   )
{
   if (ros::ok())
   {
      ros::spinOnce(); // spin even when disabled.
   }
}

//------------------------------------------------------------------------------
Ec::HardwareStatus
ros1HardwarePlugin::getFromHardwareImplementation
   (
   )
{
   // This method assumes that m_CurrentJointState is set via the
   // jointStateCallback method
   if (ros::ok())
   {
      ros::spinOnce(); // spin once first to get latest joint positions.
      getParam<Ec::JointAngle|Ec::HardwareUnits>(m_JointPositions);

      EcPrint(Debug) << "EcRos1HardwarePluginBase::getFromHardwareImplementation\n";

      // Bail if vector the wrong size
      if(m_JointPositions.size() >= m_NumJoints)
      {
         // Read data from hardware and store in our vector.
         const Ec::HardwareConfigBaseVector& configs = hardwareConfig();
         for(EcSizeT ii=0; ii<configs.size(); ++ii)
         {
            if (configs[ii].jointMapIndex.value() < m_CurrentJointState.position.size())
            {
               EcPrint(Debug) << "Actual Position = " << m_CurrentJointState.position[configs[ii].jointMapIndex]
                              << " for index = " << configs[ii].jointMapIndex.value() << std::endl;
               m_JointPositions[ii] = m_CurrentJointState.position[configs[ii].jointMapIndex];
            }
            else
            {
               EcPrint(Error) << "Joint index " << configs[ii].jointMapIndex.value() << " is out of range for joint_states size "
                              << m_CurrentJointState.position.size() << std::endl;
               return Ec::HardwareStatusReadError;
            }
         }

         // Store the results in local, hardware units to be converted according to
         // the configuration file.
         return setParam<Ec::JointAngle|Ec::HardwareUnits>(m_JointPositions) ?
                Ec::HardwareStatusOk : Ec::HardwareStatusReadError;
      }

      return Ec::HardwareStatusIncompatState;
   }

   return Ec::HardwareStatusGeneralError;
}

//------------------------------------------------------------------------------
Ec::HardwareStatus
ros1HardwarePlugin::setToHardwareImplementation
   (
   )
{
   if (ros::ok())
   {
      ros::Time timeNow = ros::Time::now();

      getParam<Ec::JointAngle|Ec::HardwareUnits>(m_JointPositions);
      getParam<Ec::JointVelocity|Ec::HardwareUnits>(m_JointVelocities);
      if (m_TorqueEnabled)
      {
         getParam<Ec::State>(manipIndex(), m_PositionState);
         m_GravitationalTorqueTool.setPositionState(m_PositionState);
         m_JointTorques = m_GravitationalTorqueTool.gravitationalTorques().transpose()[0];
      }

      // Push simulation values to hardware
      // Bail if vector the wrong size
      const Ec::HardwareConfigBaseVector& configs = hardwareConfig();
      if(configs.size() == m_NumJoints)
      {
         for(EcSizeT ii=0; ii<configs.size(); ++ii)
         {
            if (configs[ii].jointMapIndex.value() < m_ActinJointStates->position.size())
            {
               EcPrint(Debug) << "Sending Position = " << m_JointPositions[configs[ii].jointMapIndex]
                              << " for index = " << configs[ii].jointMapIndex.value() << std::endl;
               EcPrint(Debug) << "Sending Velocity = " << m_JointVelocities[configs[ii].jointMapIndex]
                              << " for index = " << configs[ii].jointMapIndex.value() << std::endl;
               m_ActinJointStates->position[configs[ii].jointMapIndex] = m_JointPositions[configs[ii].jointMapIndex];
               m_ActinJointStates->velocity[configs[ii].jointMapIndex] = m_JointVelocities[configs[ii].jointMapIndex];
               if (m_TorqueEnabled &&
                   (configs[ii].jointMapIndex.value() < m_JointTorques.size()))
               {
                  EcPrint(Debug) << "Sending Torque = " << m_JointTorques[configs[ii].jointMapIndex]
                                 << " for index = " << configs[ii].jointMapIndex.value() << std::endl;
                  m_ActinJointStates->effort[configs[ii].jointMapIndex] = m_JointTorques[configs[ii].jointMapIndex];
               }
               else
               {
                   EcPrint(Warning) << "Joint index " << configs[ii].jointMapIndex.value() << " is out of range for joint torques size "
                                    << m_JointTorques.size() << std::endl;
               }
            }
            else
            {
               EcPrint(Error) << "Joint index " << configs[ii].jointMapIndex.value() << " is out of range for joint_states size "
                              << m_ActinJointStates->position.size() << std::endl;
               return Ec::HardwareStatusWriteError;
            }
         }
         m_ActinJointStates->header.stamp = timeNow;
         m_ActinJointStatesWriter.publish(m_ActinJointStates);

         return Ec::HardwareStatusOk;
      }

      return Ec::HardwareStatusIncompatState;
   }

   return Ec::HardwareStatusGeneralError;
}

//------------------------------------------------------------------------------
Ec::HardwareStatus
ros1HardwarePlugin::torqueEnableImplementation
  (
  const EcBoolean enable
  )
{
   if (enable)
   {
      m_ActinJointStates->effort.resize(hardwareConfig().size());
   }
   else
   {
      m_ActinJointStates->effort.clear();
   }
   return Ec::HardwareStatusOk;
}

//------------------------------------------------------------------------------
void
ros1HardwarePlugin::createConfigFile
  (
  )
{
   // Setup initial configuration file for the current active manipulator.
   EcU32 activeManipulator = 0;
   EcStringVector manipLinkNames;
   getParam<Ec::Manipulator>(activeManipulator);
   getParam<Ec::Manipulator>(activeManipulator,
      ROS1HW_DATA->rosManipulatorConfig.rosManipulatorLabel.value());
   boost::replace_all(ROS1HW_DATA->rosManipulatorConfig.rosManipulatorLabel.value(), "-", "_");
   boost::replace_all(ROS1HW_DATA->rosManipulatorConfig.rosManipulatorLabel.value(), " ", "_");
   boost::replace_all(ROS1HW_DATA->rosManipulatorConfig.rosManipulatorLabel.value(), ".", "_");
   getParam<Ec::Manipulator>(activeManipulator, manipLinkNames);
   ROS1HW_DATA->m_ManipIndex = activeManipulator;
   Ec::HardwareConfigBaseVector vJoints; // Entire vector of joints

   for(EcU32 jj=0; jj<manipLinkNames.size(); ++jj)
   {
      EcRos1HardwareConfig ros1Config;
      ros1Config.setHardwareToActinFactor(1.0);
      ros1Config.setHardwareVelocityToActinFactor(1.0);
      ros1Config.jointMapIndex = jj;
      ros1Config.servoID = jj;
      ros1Config.rosLinkLabel = boost::replace_all_copy(
         tf::resolve(ROS1HW_CDATA->rosManipulatorConfig.tf_prefix,
            manipLinkNames[jj]),"-", "_");
      boost::replace_all(ros1Config.rosLinkLabel.value(), " ", "_");
      boost::replace_all(ros1Config.rosLinkLabel.value(), ".", "_");
      vJoints.pushBack(ros1Config);
   }

   setHardwareConfig(vJoints);

   // Store config
   if(saveConfigurationToFile() != Ec::HardwareStatusOk)
   {
      EcPrint(Error) << "Failed to save config file." << std::endl;
   }
}

//------------------------------------------------------------------------------
ros::NodeHandlePtr ros1HardwarePlugin::nodeHandle
  (
  ) const
{
   return m_pNodeHandle;
}

//------------------------------------------------------------------------------
ros::NodeHandlePtr ros1HardwarePlugin::privateNodeHandle
  (
  ) const
{
   return m_pPrivateNodeHandle;
}

//---------------------------------------------------------------------------
void ros1HardwarePlugin::jointStatesCallback
   (
   sensor_msgs::JointState::ConstPtr state
   )
{
   if (state->name.size() != state->position.size())
   {
       EcERROR("ros1HardwarePlugin::jointStateCallback: Received an invalid joint state vector\n");
       return;
   }

   if (state->header.frame_id ==
       tf::resolve(ROS1HW_CDATA->rosManipulatorConfig.tf_prefix,
                   ROS1HW_CDATA->rosManipulatorConfig.rosManipulatorLabel))
   {
      const Ec::HardwareConfigBaseVector& vDevices = ROS1HW_CDATA->m_vpHardwareConfig;
      if(state->name.size() != vDevices.size())
      {
         EcERROR("ros1HardwarePlugin::jointStateCallback: Configured joint names size differs from published names size\n");
         return;
      }
      for (EcSizeT ii=0; ii<vDevices.size(); ++ii)
      {
         const EcRos1HardwareConfig& device = dynamic_cast<const EcRos1HardwareConfig&>(vDevices[ii]);
         if (state->name[ii] != device.rosLinkLabel.value())
         {
            EcERROR("ros1HardwarePlugin::jointStateCallback: Configured joint name differs from published name\n");
            return;
         }
      }
      m_CurrentJointState = *state;
   }
}
