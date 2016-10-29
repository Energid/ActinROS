//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1RobotStatePublisherPlugin.cpp
//
//------------------------------------------------------------------------------
#include <plugins/ecIOParams.h>
#include <control/ecPosContSystem.h>
#include <boost/algorithm/string/replace.hpp>
#include <boost/numeric/conversion/cast.hpp>

#include "ros1RobotStatePublisherPlugin.h"

namespace
{

const EcString FOWARD_SLASH = "/";

} //anonymous namespace

EC_PLUGIN_STUB_DEFAULT(ros1RobotStatePublisherPlugin)

//------------------------------------------------------------------------------
ros1RobotStatePublisherPlugin::ros1RobotStatePublisherPlugin
   (
   ) :
   EcRos1PluginBase(),
   m_ManipulatorNames(),
   m_ManipulatorTransforms(),
   m_Transformation(),
   m_BaseCallbackTransformation(),
   m_BaseCallbackOrientation(),
   m_BaseLinkNamesMap(),
   m_ConfigurationPtr(new EcRos1RobotStatePublisherPluginConfig()),
   m_TfBroadcaster(),
   m_TfListener(),
   m_JointStateReader(),
   m_PublishInterval(0),
   m_LastCallbackTime(),
   m_LastPublishTime(),
   m_Mutex()
{
   if (m_ConfigurationPtr->publish_frequency.value() != 0.0)
   {
      m_PublishInterval = ros::Duration(1.0/m_ConfigurationPtr->publish_frequency);
      m_UpdatePeriodInMs = static_cast<EcU32>(1000.0/m_ConfigurationPtr->publish_frequency);
   }
}

//---
// ros1RobotStatePubSubPlugin public methods
//---

//---------------------------------------------------------------------------
EcRos1RobotStatePublisherPluginConfig ros1RobotStatePublisherPlugin::configuration
   (
   ) const
{
   EcMutexScopedLock lock(m_Mutex);

   return *m_ConfigurationPtr.get();
}

//---------------------------------------------------------------------------
EcBoolean ros1RobotStatePublisherPlugin::setConfiguration
   (
   const EcRos1RobotStatePublisherPluginConfig& configuration
   )
{
   EcMutexScopedLock lock(m_Mutex);

   *m_ConfigurationPtr = configuration;

   return setupRobotStatePublicationAndSubscriptions();
}

//---------------------------------------------------------------------------
EcBoolean ros1RobotStatePublisherPlugin::readConfigurationString
   (
   EcXmlReader& stream
   )
{
   EcRos1RobotStatePublisherPluginConfig config;
   const EcBoolean success = config.read(stream);
   setConfiguration(config);

   return success;
}

//---------------------------------------------------------------------------
EcBoolean ros1RobotStatePublisherPlugin::writeConfigurationString
   (
   EcXmlWriter& stream
   ) const
{
   return configuration().write(stream);
}

//---------------------------------------------------------------------------
EcBoolean ros1RobotStatePublisherPlugin::initState
   (
   )
{
   EcMutexScopedLock lock(m_Mutex);

   return setupRobotStatePublicationAndSubscriptions();
}

//------------------------------------------------------------------------------
void ros1RobotStatePublisherPlugin::update
   (
   const EcReal time
   )
{
   EcMutexScopedLock lock(m_Mutex);

   if (ros::ok())
   {
      // Iterate over base positions.
      for (EcStringU32Map::const_iterator mapIt = m_BaseLinkNamesMap.begin();
           mapIt != m_BaseLinkNamesMap.end(); mapIt++)
      {
         if (m_TfListener->canTransform(m_ConfigurationPtr->rosWorldFrameName,
               mapIt->first, ros::Time(0)))
         {
            try
            {
               m_TfListener->lookupTransform(m_ConfigurationPtr->rosWorldFrameName,
                  mapIt->first, ros::Time(0), m_LookupTransform);
               basePositionCallback(mapIt->second, m_LookupTransform);
            }
            catch (tf::TransformException &ex)
            {
               EcWARN("ros1RobotStatePubSubPlugin::update: %s\n",ex.what());
            }
         }
      }

      ros::spinOnce();
   }
   else
   {
      EcPrint(Error) << "Cannot publish data" << std::endl;
   }
}

//---
// ros1RobotStatePubSubPlugin protected methods
//---

//------------------------------------------------------------------------------
EcBoolean ros1RobotStatePublisherPlugin::initRos
   (
   )
{
   // These two classes create ROS nodes internally so they
   // can't be created until after ROS initialization in the
   // base class.
   m_TfBroadcaster.reset(new tf::TransformBroadcaster());
   m_TfListener.reset(new tf::TransformListener(*m_pNodeHandle));
   return EcTrue;
}

//---------------------------------------------------------------------------
EcBoolean ros1RobotStatePublisherPlugin::setupRobotStatePublicationAndSubscriptions
   (
   )
{
   EcSharedMutexSharedLock lock;
   m_pPrivateNodeHandle->setParam("use_tf_static", false);
   if (m_ConfigurationPtr->publish_frequency.value() == 0.0)
   {
      EcReal simUpdateRate = param<Ec::SimulationThreadUpdateRate,EcReal>();
      m_PublishInterval = ros::Duration(1.0/simUpdateRate);
      m_pPrivateNodeHandle->setParam("publish_frequency", simUpdateRate);
   }
   else
   {
      m_pPrivateNodeHandle->setParam("publish_frequency", m_ConfigurationPtr->publish_frequency.value());
      m_PublishInterval = ros::Duration(1.0/m_ConfigurationPtr->publish_frequency);
      m_UpdatePeriodInMs = static_cast<EcU32>(1000.0/m_ConfigurationPtr->publish_frequency);
   }

   setParam<Ec::ControlSystem>(EcFalse);

   getParam<Ec::Manipulator>(m_ManipulatorNames);

   EcRos1RobotStatePublisherManipulatorConfigVector& manipConfigs = m_ConfigurationPtr->manipulatorConfigs;
   if (manipConfigs.size() != m_ManipulatorNames.size())
   {
      manipConfigs.resize(m_ManipulatorNames.size());
   }
   for (EcSizeT ii = 0; ii < m_ManipulatorNames.size(); ++ii)
   {
      // ROS can't handle dashes and spaces.
      manipConfigs[ii].manipulatorName = boost::replace_all_copy(m_ManipulatorNames[ii], "-", "_");
      boost::replace_all(manipConfigs[ii].manipulatorName.value(), " ", "_");
      boost::replace_all(manipConfigs[ii].manipulatorName.value(), ".", "_");
   }

   // Subscribe to joint_states.
   try
   {
      // Make queue size twice the number of manipulators.
      m_JointStateReader = m_pNodeHandle
         ->subscribe(m_ConfigurationPtr->feedbackJointStatesTopicName,
         boost::numeric_cast<uint32_t>(2 * m_ManipulatorNames.size()),
         &ros1RobotStatePublisherPlugin::jointStateCallback, this);
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

   m_BaseLinkNamesMap.clear();
   m_ManipulatorTransforms.resize(m_ManipulatorNames.size());
   m_LastPublishTime.resize(m_ManipulatorNames.size());
   getParam<Ec::SimulationMutex>(lock);
   const EcIndividualManipulatorVector& manipVector = *paramPtr<Ec::Manipulator,EcIndividualManipulatorVector>();
   EcString topicName;
   for(EcSizeT ii=0; ii<manipVector.size(); ++ii)
   {
      const EcString& currentTfPrefix = m_ConfigurationPtr->manipulatorConfigs[ii].tf_prefix;
      topicName = boost::replace_all_copy(
         tf::resolve(currentTfPrefix,
            manipConfigs[ii].manipulatorName.value() + FOWARD_SLASH +
            manipVector[ii].linkLabel()), "-", "_");
      boost::replace_all(topicName, " ", "_");
      boost::replace_all(topicName, ".", "_");
      try
      {
         m_BaseLinkNamesMap[topicName] = boost::numeric_cast<EcU32>(ii);
      }
      catch (boost::numeric::bad_numeric_cast& error)
      {
         EcPrint(Error) << "Bad cast exception creating base link maps, " << error.what() << std::endl;
         return EcFalse;
      }

      // Capture all of the link frame names.
      const EcIndividualManipulator& manip = manipVector[ii];
      m_ManipulatorTransforms[ii].resize(manip.jointDof());
      for(EcU32 jj=0; jj<manip.jointDof(); ++jj)
      {
         m_ManipulatorTransforms[ii][jj].header.frame_id = boost::replace_all_copy(
            tf::resolve(currentTfPrefix,
               manipConfigs[ii].manipulatorName.value() + FOWARD_SLASH +
               manip.linkByIndex(jj)->parentLink()->linkLabel()), "-", "_");
         boost::replace_all(m_ManipulatorTransforms[ii][jj].header.frame_id, " ", "_");
         m_ManipulatorTransforms[ii][jj].child_frame_id = boost::replace_all_copy(
            tf::resolve(currentTfPrefix,
               manipConfigs[ii].manipulatorName.value() + FOWARD_SLASH +
               manip.linkByIndex(jj)->linkLabel()), "-", "_");
         boost::replace_all(m_ManipulatorTransforms[ii][jj].child_frame_id, " ", "_");
         boost::replace_all(m_ManipulatorTransforms[ii][jj].child_frame_id, ".", "_");
      }
   }

   return EcTrue;
}

//---------------------------------------------------------------------------
void ros1RobotStatePublisherPlugin::jointStateCallback
   (
   sensor_msgs::JointState::ConstPtr state
   )
{
   if (state->name.size() != state->position.size())
   {
       EcERROR("Robot state publisher received an invalid joint state vector\n");
       return;
   }

   EcU32 manipulatorId = EcFoundCommon::VOIDINDEX;
   const EcRos1RobotStatePublisherManipulatorConfigVector& manipConfigs = m_ConfigurationPtr->manipulatorConfigs;
   for (EcSizeT ii = 0; ii < manipConfigs.size(); ++ii)
   {
      if (state->header.frame_id ==
          tf::resolve(manipConfigs[ii].tf_prefix, manipConfigs[ii].manipulatorName))
      {
         try
         {
            manipulatorId = boost::numeric_cast<EcU32>(ii);
            break;
         }
         catch (boost::numeric::bad_numeric_cast& error)
         {
            EcPrint(Error) << "Bad cast exception setting manipulator ID, " << error.what() << std::endl;
            return;
         }
      }
   }

   if (manipulatorId == EcFoundCommon::VOIDINDEX)
   {
      // Return if invalid ID
      return;
   }

   // check if we moved backwards in time (e.g. when playing a bag file)
   ros::Time now = ros::Time::now();
   if(m_LastCallbackTime > now)
   {
      // force re-publish of joint transforms
      EcWARN("Moved backwards in time (probably because ROS clock was reset), re-publishing joint transforms!");
      m_LastPublishTime[manipulatorId].clear();
   }
   m_LastCallbackTime = now;

   // determine least recently published joint
   ros::Time last_published = now;
   for (EcSizeT i=0; i<state->name.size(); i++)
   {
      ros::Time t = m_LastPublishTime[manipulatorId][state->name[i]];
      last_published = (t < last_published) ? t : last_published;
   }
   // note: if a joint was seen for the first time,
   //       then last_published is zero.

   // check if we need to publish
   if (state->header.stamp >= last_published + m_PublishInterval)
   {
      publishTransforms(manipulatorId, state->position, state->header.stamp);

      // store publish time in joint map
      for (EcSizeT i=0; i<state->name.size(); i++)
      {
         m_LastPublishTime[manipulatorId][state->name[i]] = state->header.stamp;
      }
   }
}

//---------------------------------------------------------------------------
void ros1RobotStatePublisherPlugin::basePositionCallback
   (
   EcU32 manipulatorId,
   const tf::StampedTransform& position
   )
{
   if (manipulatorId != EcFoundCommon::VOIDINDEX)
   {
      m_BaseCallbackTransformation.setTranslationX(position.getOrigin().x());
      m_BaseCallbackTransformation.setTranslationY(position.getOrigin().y());
      m_BaseCallbackTransformation.setTranslationZ(position.getOrigin().z());
      m_BaseCallbackOrientation.set(position.getRotation().w(),
                                    position.getRotation().x(),
                                    position.getRotation().y(),
                                    position.getRotation().z());
      m_BaseCallbackTransformation.setOrientation(m_BaseCallbackOrientation);
      setParam<Ec::Base>(manipulatorId, m_BaseCallbackTransformation);
   }
}


//---------------------------------------------------------------------------
void ros1RobotStatePublisherPlugin::publishTransforms
   (
   EcU32 manipulatorID,
   const EcRealVector &positions,
   const ros::Time& time
   )
{
   if (positions.size() != m_ManipulatorTransforms[manipulatorID].size())
   {
       EcERROR("Robot state publisher received an invalid joint state vector\n");
       return;
   }

   setParam<Ec::JointAngle>(manipulatorID, positions);
   for(EcSizeT ii=0; ii<m_ManipulatorTransforms[manipulatorID].size(); ++ii)
   {
      try
      {
         getParam<Ec::DhFrame>(manipulatorID,
            boost::numeric_cast<EcU32>(ii), m_Transformation);
         m_ManipulatorTransforms[manipulatorID][ii].header.stamp = time;
         m_ManipulatorTransforms[manipulatorID][ii].transform.translation.x = m_Transformation.translation().x();
         m_ManipulatorTransforms[manipulatorID][ii].transform.translation.y = m_Transformation.translation().y();
         m_ManipulatorTransforms[manipulatorID][ii].transform.translation.z = m_Transformation.translation().z();
         m_ManipulatorTransforms[manipulatorID][ii].transform.rotation.x = m_Transformation.orientation().x();
         m_ManipulatorTransforms[manipulatorID][ii].transform.rotation.y = m_Transformation.orientation().y();
         m_ManipulatorTransforms[manipulatorID][ii].transform.rotation.z = m_Transformation.orientation().z();
         m_ManipulatorTransforms[manipulatorID][ii].transform.rotation.w = m_Transformation.orientation().w();
      }
      catch (boost::numeric::bad_numeric_cast &error)
      {
         EcPrint(Error) << "Bad cast exception getting DhFrames, " << error.what() << std::endl;
      }
   }
   m_TfBroadcaster->sendTransform(m_ManipulatorTransforms[manipulatorID]);
}
