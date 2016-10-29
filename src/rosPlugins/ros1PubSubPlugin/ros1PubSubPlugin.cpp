//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1PubSubPlugin.cpp
//
//------------------------------------------------------------------------------
#include <plugins/ecIOParams.h>
#include <control/ecPosContSystem.h>
#include <exception>

#include "ros1PubSubPlugin.h"
#include "ecPubSubPluginRos1Layer.h"

EC_PLUGIN_STUB_DEFAULT(ros1PubSubPlugin)

//------------------------------------------------------------------------------
ros1PubSubPlugin::ros1PubSubPlugin
   (
   ) :
EcRos1PluginBase(),
m_InitialPositionControllerFlagVector(),
m_ConfigurationPtr(new EcRos1PubSubPluginConfig()),
m_LayerPtr(),
m_Mutex()
{
}

//---
// ros1PubSubPlugin public methods
//---

//---------------------------------------------------------------------------
EcRos1PubSubPluginConfig ros1PubSubPlugin::configuration
   (
   ) const
{
   EcMutexScopedLock lock(m_Mutex);

   return *m_ConfigurationPtr.get();
}

//---------------------------------------------------------------------------
EcBoolean ros1PubSubPlugin::setConfiguration
   (
   const EcRos1PubSubPluginConfig& configuration
   )
{
   EcMutexScopedLock lock(m_Mutex);

   const EcBoolean participantNeeded = isParticipantNeeded(configuration);

   if (!participantNeeded)
   {
      m_LayerPtr.reset();
   }

   *m_ConfigurationPtr = configuration;

   updatePositionControllerFlags();
   if (!participantNeeded)
   {
      return EcTrue;
   }

   if (!m_LayerPtr)
   {
      boost::shared_ptr<ecPubSubPluginRos1Layer> layerPtr
         (
         new ecPubSubPluginRos1Layer
            (
            this,
            boost::bind(&ros1PubSubPlugin::endEffectorCallback, this, _1, _2),
            boost::bind(&ros1PubSubPlugin::jointAngleCallback, this, _1, _2)
            )
         );
      if (!layerPtr)
      {
         return EcFalse;
      }

      m_LayerPtr = layerPtr;
   }

   return m_LayerPtr->setConfiguration(*m_ConfigurationPtr);
}

//---------------------------------------------------------------------------
EcBoolean ros1PubSubPlugin::readConfigurationString
   (
   EcXmlReader& stream
   )
{
   EcRos1PubSubPluginConfig config;
   const EcBoolean success = config.read(stream);
   setConfiguration(config);

   return success;
}

//---------------------------------------------------------------------------
EcBoolean ros1PubSubPlugin::writeConfigurationString
   (
   EcXmlWriter& stream
   ) const
{
   return configuration().write(stream);
}

//---------------------------------------------------------------------------
EcBoolean ros1PubSubPlugin::initState
   (
   )
{
   EcMutexScopedLock lock(m_Mutex);
   updateInitialPositionControllerFlags();
   return EcTrue;
}

//------------------------------------------------------------------------------
void ros1PubSubPlugin::update
   (
   const EcReal time
   )
{
   EcMutexScopedLock lock(m_Mutex);
   if (!m_LayerPtr || !m_LayerPtr->hasPublisher())
   {
      ros::spinOnce();
      return;
   }

   if (ros::ok())
   {
      EcRealVectorVector jointAngles;
      getParam<Ec::JointAngle>(jointAngles);

      EcManipulatorEndEffectorPlacementVector placements;
      getParam<Ec::DesiredEndEffector, EcManipulatorEndEffectorPlacementVector>(placements);

      m_LayerPtr->publishJointAngles(jointAngles);
      m_LayerPtr->publishEndEffectors(placements);

      ros::spinOnce();
   }
   else
   {
      EcPrint(Error) << "Cannot publish data" << std::endl;
   }
}

//---
// ros1PubSubPlugin protected methods
//---

//------------------------------------------------------------------------------
EcBoolean ros1PubSubPlugin::initRos
   (
   )
{
   return EcTrue;
}

//---
// ros1PubSubPlugin private methods
//---

//---------------------------------------------------------------------------
void ros1PubSubPlugin::updateInitialPositionControllerFlags
   (
   )
{
   {
      EcSharedMutexSharedLock lock;
      getParam<Ec::SimulationMutex>(lock);

      const EcPositionControlSystem* pPcs = paramPtr<Ec::ControlSystem, EcPositionControlSystem>();
      const EcPositionControllerVector& positionControllers = pPcs->positionControllers();
      const EcSizeT numControllers = positionControllers.size();
      m_InitialPositionControllerFlagVector.resize(numControllers);
      for (EcSizeT ii = 0; ii < numControllers; ++ii)
      {
         m_InitialPositionControllerFlagVector[ii] = positionControllers[ii].isOn();
      }
   }

   updatePositionControllerFlags();
}

//---------------------------------------------------------------------------
void ros1PubSubPlugin::updatePositionControllerFlags
   (
   )
{
   EcSharedMutexSharedLock lock;
   getParam<Ec::SimulationMutex>(lock);

   EcPositionControlSystem* pPcs = const_cast<EcPositionControlSystem*>(paramPtr<Ec::ControlSystem, EcPositionControlSystem>());

   const EcPositionControllerVector& positionControllers = pPcs->positionControllers();

   const EcRos1PubSubManipulatorConfigVector& manipConfigs = m_ConfigurationPtr->manipulatorConfigs;
   const EcSizeT numManipConfigs = manipConfigs.size();
   const EcSizeT numControllers = m_InitialPositionControllerFlagVector.size();
   for (EcU32 ii = 0; ii < numControllers; ++ii)
   {
      const EcBoolean currentFlag = positionControllers[ii].isOn();
      if (
            (ii < numManipConfigs) &&
            (
               (manipConfigs[ii].mode.value() == EcRos1PubSubManipulatorConfig::MODE_PUB_EE_SUB_JOINT) ||
               (manipConfigs[ii].mode.value() == EcRos1PubSubManipulatorConfig::MODE_SUB_JOINT)
            )
         )
      {
         if (currentFlag)
         {
            EcPositionController positionController = positionControllers[ii];
            positionController.setIsOn(EcFalse);
            pPcs->exchangePositionController(ii, positionController);
         }
      }
      else if (currentFlag != m_InitialPositionControllerFlagVector[ii])
      {
         EcPositionController positionController = positionControllers[ii];
         positionController.setIsOn(m_InitialPositionControllerFlagVector[ii]);
         pPcs->exchangePositionController(ii, positionController);
      }
   }
}

//---------------------------------------------------------------------------
EcBoolean ros1PubSubPlugin::isParticipantNeeded
   (
   const EcRos1PubSubPluginConfig& config
   ) const
{
   const EcRos1PubSubManipulatorConfigVector& manipConfigs = config.manipulatorConfigs;
   const EcSizeT numManipConfigs = manipConfigs.size();
   for (EcSizeT ii = 0; ii < numManipConfigs; ++ii)
   {
      if (manipConfigs[ii].mode != EcRos1PubSubManipulatorConfig::MODE_IGNORE)
      {
         return EcTrue;
      }
   }

   return EcFalse;
}

//---------------------------------------------------------------------------
void ros1PubSubPlugin::endEffectorCallback
   (
   EcU32 manipulatorId,
   const EcManipulatorEndEffectorPlacement& placement
   )
{
   setParam<Ec::DesiredEndEffector>(manipulatorId, placement);
}

//---------------------------------------------------------------------------
void ros1PubSubPlugin::jointAngleCallback
   (
   EcU32 manipulatorId,
   const EcRealVector& jointAngles
   )
{
   setParam<Ec::JointAngle>(manipulatorId, jointAngles);
}
