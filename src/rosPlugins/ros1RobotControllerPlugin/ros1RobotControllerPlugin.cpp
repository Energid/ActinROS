//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1RobotControllerPlugin.cpp
//
//------------------------------------------------------------------------------
#include <plugins/ecIOParams.h>
#include <control/ecPosContSystem.h>
#include <boost/algorithm/string/replace.hpp>
#include <boost/numeric/conversion/cast.hpp>

#include "ros1RobotControllerPlugin.h"

namespace
{

const EcString FOWARD_SLASH = "/";
const EcString ACTUAL       = "actual";

} //anonymous namespace

EC_PLUGIN_STUB_DEFAULT(ros1RobotControllerPlugin)

// These are convenience macros when overloading the arm data
#define DATA            m_Data.element()
#define ROS1RC_DATA     static_cast<EcRos1RobotControllerPluginConfig*>(const_cast<Ec::HardwarePluginData*>(DATA))
#define ROS1RC_CDATA    static_cast<const EcRos1RobotControllerPluginConfig*>(DATA)

//------------------------------------------------------------------------------
ros1RobotControllerPlugin::ros1RobotControllerPlugin
   (
   ) :
ros1HardwarePlugin(),
m_ManipulatorNames(),
m_ActiveEePoseNamesMap(),
m_Transformation(),
m_EECallbackTransformation(),
m_BaseTransformation(),
m_BaseCallbackTransformation(),
m_EECallbackOrientation(),
m_BaseCallbackOrientation(),
m_BaseLinkNamesMap(),
m_BaseTransform(),
m_InitialPositionControllerFlagVector(),
m_TfBroadcaster(),
m_TfListener(),
m_EndEffectorReaders(),
m_EndEffectorWriters(),
m_ActualEndEffectorPoses()
{
   m_ConfigFilename = "ros1RobotControllerConfig.xml";

   // These calls switch out the default data with our subclassed version
   m_Data.ECXML_REGISTER_COMPONENT_CREATOR(EcRos1RobotControllerPluginConfig);
   m_Data.setElement(EcRos1RobotControllerPluginConfig());
}

//---
// ros1RobotControllerPlugin public methods
//---

//---------------------------------------------------------------------------
EcRos1RobotControllerPluginConfig ros1RobotControllerPlugin::configuration
   (
   ) const
{
   EcMutexScopedLock lock(m_Mutex);

   return *ROS1RC_DATA;
}

//---------------------------------------------------------------------------
EcBoolean
ros1RobotControllerPlugin::setConfiguration
   (
   const EcRos1RobotControllerPluginConfig& configuration
   )
{
   EcMutexScopedLock lock(m_Mutex);

   m_Data.setElement(configuration);

   return initRos();
}

Ec::HardwareStatus
ros1RobotControllerPlugin::initImplementation
   (
   )
{
   updateInitialPositionControllerFlags();

   if (ros1HardwarePlugin::initImplementation() ==
       Ec::HardwareStatusOk)
   {
      // These two classes create ROS nodes internally so they
      // can't be created until after ROS initialization in the
      // base class.
      m_TfBroadcaster.reset(new tf::TransformBroadcaster());
      m_TfListener.reset(new tf::TransformListener(*m_pNodeHandle));

      return Ec::HardwareStatusOk;
   }
   return Ec::HardwareStatusGeneralError;
}

//---------------------------------------------------------------------------
EcBoolean
ros1RobotControllerPlugin::initRos
   (
   )
{
   if (ros1HardwarePlugin::initRos())
   {
      updatePositionControllerFlags();

      return setupRobotStatePublicationAndSubscriptions();
   }

   return EcFalse;
}

//------------------------------------------------------------------------------
Ec::HardwareStatus
ros1RobotControllerPlugin::setToHardwareImplementation
   (
   )
{
   if (ros1HardwarePlugin::setToHardwareImplementation() == Ec::HardwareStatusOk)
   {
      if (ros::ok())
      {
         ros::Time timeNow = ros::Time::now();

         // Set transform for base.
         getParam<Ec::Base>(manipIndex(), m_Transformation);
         m_BaseTransform.header.stamp = timeNow;
         m_BaseTransform.transform.translation.x = m_Transformation.translation().x();
         m_BaseTransform.transform.translation.y = m_Transformation.translation().y();
         m_BaseTransform.transform.translation.z = m_Transformation.translation().z();
         m_BaseTransform.transform.rotation.x = m_Transformation.orientation().x();
         m_BaseTransform.transform.rotation.y = m_Transformation.orientation().y();
         m_BaseTransform.transform.rotation.z = m_Transformation.orientation().z();
         m_BaseTransform.transform.rotation.w = m_Transformation.orientation().w();
         m_TfBroadcaster->sendTransform(m_BaseTransform);

         // Iterate over base positions.
         for (EcStringU32Map::const_iterator mapIt = m_BaseLinkNamesMap.begin();
              mapIt != m_BaseLinkNamesMap.end(); mapIt++)
         {
            if (m_TfListener->canTransform(ROS1RC_CDATA->rosWorldFrameName,
               mapIt->first, ros::Time(0)))
            {
               try
               {
                  m_TfListener->lookupTransform(ROS1RC_CDATA->rosWorldFrameName, mapIt->first,
                                             ros::Time(0), m_LookupTransform);
                  basePositionCallback(mapIt->second, m_LookupTransform);
               }
               catch (tf::TransformException &ex)
               {
                  EcWARN("ros1RobotStatePubSubPlugin::update: %s\n",ex.what());
               }
            }
         }

         // Iterate over end-effectors and publish actual EE positions.
         EcStringU32Map::const_iterator iterator = m_ActiveEePoseNamesMap.begin();
         if (iterator != m_ActiveEePoseNamesMap.end())
         {
            getParam<Ec::ActualEndEffector>(manipIndex(), iterator->second, m_Transformation);
            m_ActualEndEffectorPoses[iterator->second]->header.stamp = timeNow;
            m_ActualEndEffectorPoses[iterator->second]->pose.position.x = m_Transformation.translation().x();
            m_ActualEndEffectorPoses[iterator->second]->pose.position.y = m_Transformation.translation().y();
            m_ActualEndEffectorPoses[iterator->second]->pose.position.z = m_Transformation.translation().z();
            m_ActualEndEffectorPoses[iterator->second]->pose.orientation.x = m_Transformation.orientation().x();
            m_ActualEndEffectorPoses[iterator->second]->pose.orientation.y = m_Transformation.orientation().y();
            m_ActualEndEffectorPoses[iterator->second]->pose.orientation.z = m_Transformation.orientation().z();
            m_ActualEndEffectorPoses[iterator->second]->pose.orientation.w = m_Transformation.orientation().w();
            m_EndEffectorWriters[iterator->second].publish(m_ActualEndEffectorPoses[iterator->second]);
         }

         ros::spinOnce();
         return Ec::HardwareStatusOk;
      }
      else
      {
         EcPrint(Error) << "Cannot publish data" << std::endl;
         return Ec::HardwareStatusWriteError;
      }
   }

   return Ec::HardwareStatusGeneralError;
}

//------------------------------------------------------------------------------
void
ros1RobotControllerPlugin::createConfigFile
  (
  )
{
   EcRos1HardwareManipulatorConfigVector& manipConfigs = ROS1RC_DATA->manipulatorConfigs;
   if (manipConfigs.size() != param<Ec::Manipulator, EcU32>())
   {
      manipConfigs.resize(param<Ec::Manipulator, EcU32>());
   }
   getParam<Ec::Manipulator>(m_ManipulatorNames);
   for (EcSizeT ii = 0; ii < m_ManipulatorNames.size(); ++ii)
   {
      // ROS can't handle dashes and spaces.
      manipConfigs[ii].rosManipulatorLabel = boost::replace_all_copy(m_ManipulatorNames[ii], "-", "_");
      boost::replace_all(manipConfigs[ii].rosManipulatorLabel.value(), " ", "_");
      boost::replace_all(manipConfigs[ii].rosManipulatorLabel.value(), ".", "_");
   }

   ros1HardwarePlugin::createConfigFile();
}

//---
// ros1RobotControllerPlugin protected methods
//---

//---------------------------------------------------------------------------
void
ros1RobotControllerPlugin::updateInitialPositionControllerFlags
   (
   )
{

   const EcPositionControlSystem* pPcs = paramPtr<Ec::ControlSystem, EcPositionControlSystem>();
   const EcPositionControllerVector& positionControllers = pPcs->positionControllers();
   const EcSizeT numControllers = positionControllers.size();
   m_InitialPositionControllerFlagVector.resize(numControllers);
   for (EcSizeT ii = 0; ii < numControllers; ++ii)
   {
      m_InitialPositionControllerFlagVector[ii] = positionControllers[ii].isOn();
   }

   updatePositionControllerFlags();
}

//---------------------------------------------------------------------------
void
ros1RobotControllerPlugin::updatePositionControllerFlags
   (
   )
{
   const EcSizeT numControllers = m_InitialPositionControllerFlagVector.size();

   // range checking to prevent crash
   if (manipIndex() >= numControllers)
   {
      EcERROR("Invalid active manipulator index specified in configuration\n");
      return;
   }

   EcPositionControlSystem* pPcs = const_cast<EcPositionControlSystem*>(paramPtr<Ec::ControlSystem, EcPositionControlSystem>());

   const EcPositionControllerVector& positionControllers = pPcs->positionControllers();

   EcRos1HardwareManipulatorConfigVector& manipConfigs = ROS1RC_DATA->manipulatorConfigs;
   if (manipConfigs.size() != numControllers)
   {
      manipConfigs.resize(numControllers);
   }
   getParam<Ec::Manipulator>(m_ManipulatorNames);
   for (EcSizeT ii = 0; ii < m_ManipulatorNames.size(); ++ii)
   {
      // ROS can't handle dashes and spaces.
      manipConfigs[ii].rosManipulatorLabel = boost::replace_all_copy(m_ManipulatorNames[ii], "-", "_");
      boost::replace_all(manipConfigs[ii].rosManipulatorLabel.value(), " ", "_");
      boost::replace_all(manipConfigs[ii].rosManipulatorLabel.value(), ".", "_");
   }
   for (EcSizeT ii = 0; ii < numControllers; ++ii)
   {
      const EcBoolean currentFlag = positionControllers[ii].isOn();
      if ( ii != manipIndex() )
      {
         if (currentFlag)
         {
            EcPositionController positionController = positionControllers[ii];
            positionController.setIsOn(EcFalse);
            try
            {
               pPcs->exchangePositionController(boost::numeric_cast<EcU32>(ii),
                  positionController);
            }
            catch (boost::numeric::bad_numeric_cast &error)
            {
               EcPrint(Error) << "Bad cast exception exchanging position controllers, " << error.what() << std::endl;
            }
         }
      }
      else if (currentFlag != m_InitialPositionControllerFlagVector[ii])
      {
         EcPositionController positionController = positionControllers[ii];
         positionController.setIsOn(m_InitialPositionControllerFlagVector[ii]);
         try
         {
            pPcs->exchangePositionController(boost::numeric_cast<EcU32>(ii),
               positionController);
         }
         catch (boost::numeric::bad_numeric_cast &error)
         {
            EcPrint(Error) << "Bad cast exception exchanging position controllers, " << error.what() << std::endl;
         }
      }
   }
}

//---------------------------------------------------------------------------
EcBoolean ros1RobotControllerPlugin::setupRobotStatePublicationAndSubscriptions
   (
   )
{
   const EcRos1HardwareManipulatorConfigVector& manipConfigs = ROS1RC_CDATA->manipulatorConfigs;
   const EcRos1HardwareManipulatorConfig& currentManipConfig = ROS1RC_CDATA->rosManipulatorConfig;
   setParam<Ec::Manipulator>(manipIndex());

   m_ActiveEePoseNamesMap.clear();
   m_EndEffectorReaders.clear();
   m_EndEffectorWriters.clear();
   m_ActualEndEffectorPoses.clear();
   EcString topicName, publishTopicName;
   const EcEndEffectorVector endEffectors =
      param<Ec::EndEffectorSet, EcEndEffectorSetVector>()[manipIndex()].endEffectors();
   const EcSizeT numEEs=endEffectors.size();
   for(EcSizeT ii=0; ii<numEEs; ++ii)
   {
      // ROS can't handle dashes and spaces.
      topicName = boost::replace_all_copy(
               tf::resolve(currentManipConfig.tf_prefix, currentManipConfig.rosManipulatorLabel.value() +
                  FOWARD_SLASH + endEffectors[ii].linkIdentifier() +
                  FOWARD_SLASH + endEffectors[ii].token().token()), "-", "_");
      boost::replace_all(topicName, " ", "_");
      boost::replace_all(topicName, ".", "_");
      publishTopicName = topicName + FOWARD_SLASH + ACTUAL;
      try
      {
         m_ActiveEePoseNamesMap[topicName] = boost::numeric_cast<EcU32>(ii);
         m_EndEffectorReaders.push_back(m_pNodeHandle
            ->subscribe(topicName, 1, &ros1RobotControllerPlugin::endEffectorCallback, this));
         m_EndEffectorWriters.push_back(m_pNodeHandle
            ->advertise<geometry_msgs::PoseStamped>(publishTopicName, 1));
         m_ActualEndEffectorPoses.push_back(boost::make_shared<geometry_msgs::PoseStamped>());
         m_ActualEndEffectorPoses[ii]->header.frame_id = publishTopicName;
      }
      catch(ros::Exception error)
      {
         EcPrint(Error) << "Failed to create ROS subscriber with exception, " << error.what() << std::endl;
         return EcFalse;
      }
      catch (boost::numeric::bad_numeric_cast& error)
      {
         EcPrint(Error) << "Failed to create ROS publisher due to bad numeric cast, " << error.what() << std::endl;
         return EcFalse;
      }
   }

   m_BaseLinkNamesMap.clear();
   const EcIndividualManipulatorVector& manipVector = *paramPtr<Ec::Manipulator,EcIndividualManipulatorVector>();
   for(EcSizeT ii=0; ii<manipVector.size(); ++ii)
   {
      // The active manipulator needs to use a special "sensed" topic suffix so that it
      // doesn't subscribe to it's own published base position transform.
      const EcString& currentTfPrefix = manipConfigs[ii].tf_prefix;
      if (ii == manipIndex())
      {
         topicName = boost::replace_all_copy(
            tf::resolve(currentTfPrefix,
               manipConfigs[ii].rosManipulatorLabel.value() + FOWARD_SLASH +
               manipVector[ii].linkLabel() + EcString("/sensed")), "-", "_");
      }
      else
      {
         topicName = boost::replace_all_copy(
            tf::resolve(currentTfPrefix,
               manipConfigs[ii].rosManipulatorLabel.value() + FOWARD_SLASH +
               manipVector[ii].linkLabel()), "-", "_");
      }
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

      // Capture base link frame names for the active manipulator.
      if (ii == manipIndex())
      {
         const EcIndividualManipulator& manip = manipVector[ii];
         m_BaseTransform.header.frame_id = ROS1RC_CDATA->rosWorldFrameName;
         m_BaseTransform.child_frame_id = boost::replace_all_copy(
               tf::resolve(currentManipConfig.tf_prefix,
                  manipConfigs[ii].rosManipulatorLabel.value() + FOWARD_SLASH +
                  manip.linkLabel()), "-", "_");
         boost::replace_all(m_BaseTransform.child_frame_id, " ", "_");
         boost::replace_all(m_BaseTransform.child_frame_id, ".", "_");
      }
   }

   return EcTrue;
}

//---------------------------------------------------------------------------
void ros1RobotControllerPlugin::endEffectorCallback
   (
   geometry_msgs::PoseStampedConstPtr placement
   )
{
   // Determine which EE
   EcU32 manipulatorId = EcFoundCommon::VOIDINDEX;
   EcU32 eeId = EcFoundCommon::VOIDINDEX;
   EcStringU32Map::const_iterator iterator = m_ActiveEePoseNamesMap.find(placement->header.frame_id);
   if (iterator != m_ActiveEePoseNamesMap.end())
   {
      manipulatorId = manipIndex();
      eeId = m_ActiveEePoseNamesMap[placement->header.frame_id];
   }
   if ((manipulatorId != EcFoundCommon::VOIDINDEX) &&
       (eeId != EcFoundCommon::VOIDINDEX))
   {
      m_EECallbackTransformation.setTranslationX(placement->pose.position.x);
      m_EECallbackTransformation.setTranslationY(placement->pose.position.y);
      m_EECallbackTransformation.setTranslationZ(placement->pose.position.z);
      m_EECallbackOrientation.set(placement->pose.orientation.w,
                                  placement->pose.orientation.x,
                                  placement->pose.orientation.y,
                                  placement->pose.orientation.z);
      m_EECallbackTransformation.setOrientation(m_EECallbackOrientation);
      setParam<Ec::DesiredEndEffector>(manipulatorId, eeId, m_EECallbackTransformation);
   }
}

//---------------------------------------------------------------------------
void ros1RobotControllerPlugin::jointStatesCallback
   (
   sensor_msgs::JointState::ConstPtr state
   )
{
   // Call parent method.
   ros1HardwarePlugin::jointStatesCallback(state);

   if (state->name.size() != state->position.size())
   {
       EcERROR("Robot state publisher received an invalid joint state vector\n");
       return;
   }

   EcU32 manipulatorId = EcFoundCommon::VOIDINDEX;
   const EcRos1HardwareManipulatorConfigVector& manipConfigs = ROS1RC_CDATA->manipulatorConfigs;
   for (EcSizeT ii = 0; ii < manipConfigs.size(); ++ii)
   {
      if (state->header.frame_id ==
          tf::resolve(manipConfigs[ii].tf_prefix, manipConfigs[ii].rosManipulatorLabel))
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
   if (manipulatorId == manipIndex())
   {
      // Don't set joints for active manipulator
      return;
   }
   if (manipulatorId == EcFoundCommon::VOIDINDEX)
   {
      // Return if invalid ID
      return;
   }

   setParam<Ec::JointAngle>(manipulatorId, state->position);
   if(state->position.size() == state->velocity.size())
   {
      setParam<Ec::JointVelocity>(manipulatorId, state->velocity);
   }
}

//---------------------------------------------------------------------------
void ros1RobotControllerPlugin::basePositionCallback
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
      // Only set the base if it's changed.
      getParam<Ec::Base>(manipulatorId, m_BaseTransformation);
      if (!m_BaseCallbackTransformation.approxEq(m_BaseTransformation,1e-12))
      {
         setParam<Ec::Base>(manipulatorId, m_BaseCallbackTransformation);
      }
   }
}
