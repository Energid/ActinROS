#ifndef ros1PubSubPlugin_H_
#define ros1PubSubPlugin_H_
//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ros1PubSubPlugin.h
/// @class ros1PubSubPlugin
/// @brief Configures and manages ROS1 publish/subscribe
/// @details This ROS1 node can act as a pusblisher or a subscriber.
//
//------------------------------------------------------------------------------
#include <ros1PluginBase/ecRos1PluginBase.h>
#include <control/ecManipEndEffectorPlace.h>
#include "ecRos1PubSubPluginConfig.h"

class ecPubSubPluginRos1Layer;

class EC_ROSPLUGINS_ROS1PUBSUBPLUGIN_DECL ros1PubSubPlugin : public EcRos1PluginBase
{
public:
   /// Get configuration object
   /// @return EcRos1PubSubPluginConfig configuration (return by value for thread safety)
   virtual EcRos1PubSubPluginConfig configuration
      (
      ) const;
    
   /// Set configuration object
   /// @param[in] configuration configuration to set
   /// @return EcBoolean Success or failure
   virtual EcBoolean setConfiguration
      (
      const EcRos1PubSubPluginConfig& configuration
      );
    
   /// @copydoc Ec::Plugin::readConfigurationString(EcXmlReader&)
   EcBoolean readConfigurationString
      (
      EcXmlReader& stream
      );
    
   /// @copydoc Ec::Plugin::writeConfigurationString(EcXmlWriter&) const
   EcBoolean writeConfigurationString
      (
      EcXmlWriter& stream
      ) const;

   /// @copydoc Ec::Plugin::initState()
   EcBoolean initState
      (
      );
    
   /// @copydoc Ec::Plugin::update
   virtual void update
      (
      const EcReal time
      );

protected:
   /// @copydoc EcRos1PluginBase::initRos
   virtual EcBoolean initRos
      (
      );
    
   /// Constructor.
   ros1PubSubPlugin
     (
     );

private:
   /// Method for setting up initial position controller isOn state for each manipulator.
   void updateInitialPositionControllerFlags
      (
      );

   /// Method for setting position controller isOn state for each manipulator
   /// according to configuration.
   void updatePositionControllerFlags
      (
      );

   /// Method to determine if ROS1 pub/sub participant is needed
   /// @return EcBoolean EcTrue if needed EcFalse otherwise
   EcBoolean isParticipantNeeded
      (
      const EcRos1PubSubPluginConfig& config
      ) const;

   /// Callback method for receiving end-effector placements.
   /// @param[in] manipulatorId Manipulator ID
   /// @param[in] placement Placement sent from another pub/sub plugin
   void endEffectorCallback
      (
      EcU32 manipulatorId,
      const EcManipulatorEndEffectorPlacement& placement
      );

   /// Callback method for receiving joint angles.
   /// @param[in] manipulatorId Manipulator ID
   /// @param[in] jointAngles Joint angles sent from another pub/sub plugin
   void jointAngleCallback
      (
      EcU32 manipulatorId,
      const EcRealVector& jointAngles
      );

   EcBooleanVector                              m_InitialPositionControllerFlagVector; ///< Initial position controller flags.
   boost::shared_ptr<EcRos1PubSubPluginConfig>  m_ConfigurationPtr;                    ///< Shared pointer to configuration parameters.
   boost::shared_ptr<ecPubSubPluginRos1Layer>   m_LayerPtr;                            ///< Shared pointer to ROS1 pub/sub layer.
   mutable EcMutex                              m_Mutex;                               ///< Handle multi-threaded access to configuration parameters.
};

#endif // ros1PubSubPlugin_H_
