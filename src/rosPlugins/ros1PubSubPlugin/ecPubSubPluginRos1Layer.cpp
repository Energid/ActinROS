//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecPubSubPluginRos1Layer.cpp
//
//------------------------------------------------------------------------------
#include "ecPubSubPluginRos1Layer.h"
#include "ecRos1PubSubPluginConfig.h"
#include "ros1PubSubPlugin.h"

#include <control/ecPosContSystem.h>
#include <xmlReaderWriter/ecXmlObjectReaderWriter.h>
#include <exception>

//---
// Anonymous namespace
//---

namespace
{

const EcBoolean useCompressedXml = EcTrue;

} // Anonymous namespace


//---
// ecPubSubPluginRos1Layer constructor
//---

//---------------------------------------------------------------------------
ecPubSubPluginRos1Layer::ecPubSubPluginRos1Layer
   (
   ros1PubSubPlugin*       pPlugin,
   EndEffectorCallbackType endEffectorCallback,
   JointAngleCallbackType  jointAngleCallback
   ) :
   m_EndEffectorCallback(endEffectorCallback),
   m_JointAngleCallback(jointAngleCallback),
   m_EndEffectorReader(),
   m_JointAngleReader(),
   m_EndEffectorWriter(),
   m_JointAngleWriter(),
   m_EndEffectorSamples(),
   m_JointAngleSamples(),
   m_pPlugin(pPlugin),
   m_QueueSize(7)
{
}

//---
// ecPubSubPluginRos1Layer destructor
//---

//---------------------------------------------------------------------------
ecPubSubPluginRos1Layer::~ecPubSubPluginRos1Layer
   (
   )
{
}


//---------------------------------------------------------------------------
EcBoolean ecPubSubPluginRos1Layer::setConfiguration
   (
   const EcRos1PubSubPluginConfig& config
   )
{
   if (isEEPublisherNeeded(config))
   {
      try
      {
         m_EndEffectorWriter = m_pPlugin->nodeHandle()
            ->advertise<EcRos1::ManipulatorMessage>(config.desiredPlacementTopicName,m_QueueSize);
      }
      catch(std::runtime_error error)
      {
         EcPrint(Error) << "Failed to create ROS publisher with exception " << error.what() << std::endl;
         return EcFalse;
      }
   }
   else
   {
       destroyEndEffectorSamples();
       destroyEndEffectorWriter();
   }
    
   if (isJointPublisherNeeded(config))
   {
      try
      {
         m_JointAngleWriter = m_pPlugin->nodeHandle()
            ->advertise<EcRos1::ManipulatorJointAngles>(config.jointAnglesTopicName,m_QueueSize);
      }
      catch(std::runtime_error error)
      {
         EcPrint(Error) << "Failed to create ROS publisher with exception " << error.what() << std::endl;
         return EcFalse;
      }
   }
   else
   {
      destroyJointAngleSamples();
      destroyJointAngleWriter();
   }
   
   if (isEESubscriberNeeded(config))
   {
      try
      {
         m_EndEffectorReader = m_pPlugin->nodeHandle()
            ->subscribe(config.desiredPlacementTopicName, m_QueueSize,
                        &ecPubSubPluginRos1Layer::endEffectorDataCallback, this);
      }
      catch(std::runtime_error error)
      {
         EcPrint(Error) << "Failed to create ROS subscriber with exception " << error.what() << std::endl;
         return EcFalse;
      }
   }
   else
   {
      destroyEndEffectorReader();
   }
   
   if (isJointSubscriberNeeded(config))
   {
      try
      {
         m_EndEffectorReader = m_pPlugin->nodeHandle()
            ->subscribe(config.jointAnglesTopicName, m_QueueSize,
                     &ecPubSubPluginRos1Layer::jointAngleDataCallback, this);
      }
      catch(std::runtime_error error)
      {
         EcPrint(Error) << "Failed to create ROS subscriber with exception " << error.what() << std::endl;
         return EcFalse;
      }
   }
   else
   {
      destroyJointAngleReader();
   }

   const EcRos1PubSubManipulatorConfigVector& manipConfigs = config.manipulatorConfigs;
   const EcSizeT numManipConfigs = manipConfigs.size();

   for (EcU32 ii = 0; ii < numManipConfigs; ++ii)
   {
      switch (manipConfigs[ii].mode)
      {
         case EcRos1PubSubManipulatorConfig::MODE_PUB_EE_PUB_JOINT:
         {
            // Publish end-effectors
            endEffectorSample(ii);

            // Publish joint angles
            jointAngleSample(ii);

            break;
         }

         case EcRos1PubSubManipulatorConfig::MODE_PUB_EE_SUB_JOINT:
         {
            // Publish end-effectors
            endEffectorSample(ii);

            // Do not publish joint angles
            destroyJointAngleSample(ii);

            break;
         }

         case EcRos1PubSubManipulatorConfig::MODE_SUB_EE_PUB_JOINT:
         {
            // Publish joint angles
            jointAngleSample(ii);

            // Do not publish end-effectors
            destroyEndEffectorSample(ii);

            break;
         }

         case EcRos1PubSubManipulatorConfig::MODE_SUB_EE:
         case EcRos1PubSubManipulatorConfig::MODE_SUB_JOINT:
         case EcRos1PubSubManipulatorConfig::MODE_IGNORE:
         default:
         {
            // Do not publish end-effectors
            destroyEndEffectorSample(ii);

            // Do not publish joint angles
            destroyJointAngleSample(ii);

            break;
         }
      }
   }

   return EcTrue;
}

//---------------------------------------------------------------------------
EcBoolean ecPubSubPluginRos1Layer::hasPublisher
   (
   ) const
{
   return ((m_EndEffectorWriter.getTopic() != "") ||
           (m_JointAngleWriter.getTopic() != ""));
}

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::publishEndEffectors
   (
   const EcManipulatorEndEffectorPlacementVector& placements
   )
{
   if (m_EndEffectorWriter.getTopic() == "")
   {
      return;
   }

   const EcSizeT numManips = placements.size();
   for (EcU32 ii = 0; ii < numManips; ++ii)
   {
      publishEndEffectors(ii, placements[ii]);
   }
}

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::publishEndEffectors
   (
   EcU32                                    manipulatorId,
   const EcManipulatorEndEffectorPlacement& placement
   )
{
   if (m_EndEffectorSamples.size() <= manipulatorId)
   {
      return;
   }

   EndEffectorSamplePtr& samplePtr = m_EndEffectorSamples[manipulatorId];
   if (!samplePtr)
   {
      return;
   }

   if (useCompressedXml)
   {
      if (!EcXmlObjectReaderWriter::writeToCompressedBuffer(placement, samplePtr->message))
      {
         EcPrint(Error) << "Failed to write end-effector placement to compressed buffer." << std::endl;
         return;
      }
   }
   else
   {
      if (!EcXmlObjectReaderWriter::writeToBuffer(placement, samplePtr->message))
      {
         EcPrint(Error) << "Failed to write end-effector placement to buffer." << std::endl;
         return;
      }
   }
   
   m_EndEffectorWriter.publish(samplePtr);
}

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::publishJointAngles
   (
   const EcRealVectorVector& jointAngles
   )
{
   if (m_JointAngleWriter.getTopic() == "")
   {
      return;
   }

   const EcSizeT numManips = jointAngles.size();
   for (EcU32 ii = 0; ii < numManips; ++ii)
   {
      publishJointAngles(ii, jointAngles[ii]);
   }
}

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::publishJointAngles
   (
   EcU32               manipulatorId,
   const EcRealVector& jointAngles
   )
{
   if (m_JointAngleSamples.size() <= manipulatorId)
   {
      return;
   }

   JointAngleSamplePtr& samplePtr = m_JointAngleSamples[manipulatorId];
   if (!samplePtr)
   {
      return;
   }

   samplePtr->jointAngles = jointAngles;

   m_JointAngleWriter.publish(samplePtr);
}

//---
// ecPubSubPluginRos1Layer private methods
//---

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::endEffectorDataCallback
   (
   EcRos1::ManipulatorMessage::ConstPtr sample
   )
{
   EcManipulatorEndEffectorPlacement placement;
   if (useCompressedXml)
   {
      if (!EcXmlObjectReaderWriter::readFromCompressedBuffer(placement, sample->message))
      {
         EcPrint(Error) << "Failed to read end-effector placement from compressed buffer." << std::endl;
         return;
      }
   }
   else
   {
      if (!EcXmlObjectReaderWriter::readFromBuffer(placement, sample->message))
      {
         EcPrint(Error) << "Failed to read end-effector placement from buffer." << std::endl;
         return;
      }
   }

   m_EndEffectorCallback(sample->manipulatorId, placement);
}

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::jointAngleDataCallback
   (
   EcRos1::ManipulatorJointAngles::ConstPtr sample
   )
{
   m_JointAngleCallback(sample->manipulatorId, sample->jointAngles);
}

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::destroyEndEffectorReader
   (
   )
{
   if (m_EndEffectorReader.getTopic() != "")
   {
      m_EndEffectorReader.shutdown();
   }
}

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::destroyEndEffectorWriter
   (
   )
{
   if (m_EndEffectorWriter.getTopic() != "")
   {
      m_EndEffectorWriter.shutdown();
   }
}

//---------------------------------------------------------------------------
ecPubSubPluginRos1Layer::EndEffectorSamplePtr ecPubSubPluginRos1Layer::endEffectorSample
   (
   EcU32 manipulatorId
   )
{
   if (m_EndEffectorSamples.size() <= manipulatorId)
   {
      m_EndEffectorSamples.resize(manipulatorId + 1);
   }

   EndEffectorSamplePtr& ptr = m_EndEffectorSamples[manipulatorId];
   if (!ptr)
   {
      ptr = boost::make_shared<EcRos1::ManipulatorMessage>();
      if (ptr)
      {
         ptr->manipulatorId = manipulatorId;
      }
   }

   return ptr;
}

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::destroyEndEffectorSample
   (
   EcU32 manipulatorId
   )
{
   if (m_EndEffectorSamples.size() <= manipulatorId)
   {
      return;
   }

   EndEffectorSamplePtr& ptr = m_EndEffectorSamples[manipulatorId];
   if (ptr)
   {
      ptr.reset();
   }
}

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::destroyEndEffectorSamples
   (
   )
{
   const EcSizeT numSamples = m_EndEffectorSamples.size();
   for (EcU32 ii = 0; ii < numSamples; ++ii)
   {
      destroyEndEffectorSample(ii);
   }
   m_EndEffectorSamples.clear();
}

//---------------------------------------------------------------------------
EcBoolean ecPubSubPluginRos1Layer::isEEPublisherNeeded
   (
   const EcRos1PubSubPluginConfig& config
   ) const
{
   const EcRos1PubSubManipulatorConfigVector& manipConfigs = config.manipulatorConfigs;
   const EcSizeT numManipConfigs = manipConfigs.size();
   for (EcU32 ii = 0; ii < numManipConfigs; ++ii)
   {
      const EcU32 mode = manipConfigs[ii].mode.value();
      if (
         (mode == EcRos1PubSubManipulatorConfig::MODE_PUB_EE_PUB_JOINT) ||
         (mode == EcRos1PubSubManipulatorConfig::MODE_PUB_EE_SUB_JOINT)
         )
      {
         return EcTrue;
      }
   }

   return EcFalse;
}

//---------------------------------------------------------------------------
EcBoolean ecPubSubPluginRos1Layer::isJointPublisherNeeded
   (
   const EcRos1PubSubPluginConfig& config
   ) const
{
    const EcRos1PubSubManipulatorConfigVector& manipConfigs = config.manipulatorConfigs;
    const EcSizeT numManipConfigs = manipConfigs.size();
    for (EcU32 ii = 0; ii < numManipConfigs; ++ii)
    {
        const EcU32 mode = manipConfigs[ii].mode.value();
        if (
            (mode == EcRos1PubSubManipulatorConfig::MODE_PUB_EE_PUB_JOINT) ||
            (mode == EcRos1PubSubManipulatorConfig::MODE_SUB_EE_PUB_JOINT)
            )
        {
            return EcTrue;
        }
    }
    
    return EcFalse;
}

//---------------------------------------------------------------------------
EcBoolean ecPubSubPluginRos1Layer::isEESubscriberNeeded
   (
   const EcRos1PubSubPluginConfig& config
   ) const
{
   const EcRos1PubSubManipulatorConfigVector& manipConfigs = config.manipulatorConfigs;
   const EcSizeT numManipConfigs = manipConfigs.size();
   for (EcU32 ii = 0; ii < numManipConfigs; ++ii)
   {
      const EcU32 mode = manipConfigs[ii].mode;
      if (
         (mode == EcRos1PubSubManipulatorConfig::MODE_SUB_EE_PUB_JOINT) ||
         (mode == EcRos1PubSubManipulatorConfig::MODE_SUB_EE)
         )
      {
         return EcTrue;
      }
   }

   return EcFalse;
}

//---------------------------------------------------------------------------
EcBoolean ecPubSubPluginRos1Layer::isJointSubscriberNeeded
   (
   const EcRos1PubSubPluginConfig& config
   ) const
{
   const EcRos1PubSubManipulatorConfigVector& manipConfigs = config.manipulatorConfigs;
   const EcSizeT numManipConfigs = manipConfigs.size();
   for (EcU32 ii = 0; ii < numManipConfigs; ++ii)
   {
      const EcU32 mode = manipConfigs[ii].mode;
      if (
         (mode == EcRos1PubSubManipulatorConfig::MODE_PUB_EE_SUB_JOINT) ||
         (mode == EcRos1PubSubManipulatorConfig::MODE_SUB_JOINT)
         )
      {
         return EcTrue;
      }
   }
   
   return EcFalse;
}

//---------------------------------------------------------------------------
ecPubSubPluginRos1Layer::JointAngleSamplePtr ecPubSubPluginRos1Layer::jointAngleSample
   (
   EcU32 manipulatorId
   )
{
   if (m_JointAngleSamples.size() <= manipulatorId)
   {
      m_JointAngleSamples.resize(manipulatorId + 1);
   }

   JointAngleSamplePtr& ptr = m_JointAngleSamples[manipulatorId];
   if (!ptr)
   {
      ptr = boost::make_shared<EcRos1::ManipulatorJointAngles>();
      if (ptr)
      {
         ptr->manipulatorId = manipulatorId;
      }
   }

   return ptr;
}

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::destroyJointAngleReader
   (
   )
{
   if (m_JointAngleReader.getTopic() != "")
   {
      m_JointAngleReader.shutdown();
   }
}

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::destroyJointAngleWriter
   (
   )
{
   if (m_JointAngleWriter.getTopic() != "")
   {
      m_JointAngleWriter.shutdown();
   }
}

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::destroyJointAngleSample
   (
   EcU32 manipulatorId
   )
{
   if (m_JointAngleSamples.size() <= manipulatorId)
   {
      return;
   }

   JointAngleSamplePtr& ptr = m_JointAngleSamples[manipulatorId];
   if (ptr)
   {
      ptr.reset();
   }
}

//---------------------------------------------------------------------------
void ecPubSubPluginRos1Layer::destroyJointAngleSamples
   (
   )
{
   const EcSizeT numSamples = m_JointAngleSamples.size();
   for (EcU32 ii = 0; ii < numSamples; ++ii)
   {
      destroyJointAngleSample(ii);
   }
   m_JointAngleSamples.clear();
}
