//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1RobotStatePubSubConfigDialog.cpp
//
//------------------------------------------------------------------------------
#include "ecRos1RobotControllerConfigDialog.h"

#include <ros1RobotControllerPlugin/ros1RobotControllerPlugin.h>
#include <ros1RobotControllerPlugin/ecRos1RobotControllerPluginConfig.h>
#include <ros1HardwarePlugin/ecRos1HardwareConfig.h>
#include <manipulator/ecIndManipulator.h>
#include <plugins/ecIOParams.h>

#include "ui_ecRos1RobotControllerConfigDialog.h"

#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMessageBox>

#include <boost/algorithm/string/replace.hpp>
#include <boost/numeric/conversion/cast.hpp>

//---
// EcRos1RobotStatePubSubConfigDialog constructor
//---

//---------------------------------------------------------------------------
EcRos1RobotControllerConfigDialog::EcRos1RobotControllerConfigDialog
   (
   ros1RobotControllerPlugin* pPlugin,
   QWidget*         pParent
   ) :
   QDialog(pParent, Qt::WindowTitleHint|Qt::WindowSystemMenuHint),
   m_UiPtr(new Ui::EcRos1RobotControllerConfigDialog()),
   m_pPlugin(pPlugin)
{
   m_UiPtr->setupUi(this);

   setFromConfiguration();
   
   connect(m_UiPtr->buttonBox, SIGNAL(accepted()), SLOT(accept()));
   connect(m_UiPtr->buttonBox, SIGNAL(rejected()), SLOT(reject()));
}

//---
// EcRos1RobotStatePubSubConfigDialog destructor
//---

//---------------------------------------------------------------------------
EcRos1RobotControllerConfigDialog::~EcRos1RobotControllerConfigDialog
   (
   )
{
}

//---
// EcRos1RobotStatePubSubConfigDialog protected slots
//---

//---------------------------------------------------------------------------
void EcRos1RobotControllerConfigDialog::accept
   (
   )
{
   setCursor(Qt::WaitCursor);
   EcRos1RobotControllerPluginConfig config;

   config.m_ManipIndex = m_UiPtr->activeManipulatorSpinBox->value();
   config.publish_frequency = m_UiPtr->publishFrequencyDoubleSpinBox->value();
   config.publishJointStatesTopicName = m_UiPtr->actinJointStatesTopicNameLineEdit->text().toStdString();
   config.feedbackJointStatesTopicName = m_UiPtr->jointStatesTopicNameLineEdit->text().toStdString();
   config.rosWorldFrameName = m_UiPtr->systemCoordinateFrameLineEdit->text().toStdString();

   const EcInt32 numRows = m_UiPtr->tableWidget->rowCount();
   config.manipulatorConfigs.resize(numRows);
   for (EcInt32 ii = 0; ii < numRows; ++ii)
   {
      EcRos1HardwareManipulatorConfig& mconfig = config.manipulatorConfigs[ii];
      mconfig.rosManipulatorLabel =  m_UiPtr->tableWidget->item(ii,0)->text().toStdString();

      QLineEdit* pLineEdit =
         dynamic_cast<QLineEdit*>(m_UiPtr->tableWidget->cellWidget(ii, 1));
      if (pLineEdit)
      {
         mconfig.tf_prefix = pLineEdit->text().toStdString();
      }
   }

   config.rosManipulatorConfig.rosManipulatorLabel =
      config.manipulatorConfigs[config.m_ManipIndex].rosManipulatorLabel;

   EcStringVector manipLinkNames;
   m_pPlugin->getParam<Ec::Manipulator>(config.m_ManipIndex.value(), manipLinkNames);
   Ec::HardwareConfigBaseVector vJoints; // Entire vector of joints

   for(EcU32 jj=0; jj<manipLinkNames.size(); ++jj)
   {
      EcRos1HardwareConfig ros1Config;
      ros1Config.setHardwareToActinFactor(1.0);
      ros1Config.setHardwareVelocityToActinFactor(1.0);
      ros1Config.jointMapIndex = jj;
      ros1Config.servoID = jj;
      ros1Config.rosLinkLabel = boost::replace_all_copy(
         tf::resolve(config.rosManipulatorConfig.tf_prefix,
            manipLinkNames[jj]),"-", "_");
      boost::replace_all(ros1Config.rosLinkLabel.value(), " ", "_");
      boost::replace_all(ros1Config.rosLinkLabel.value(), ".", "_");
      vJoints.pushBack(ros1Config);
   }
   config.m_vpHardwareConfig = vJoints;

   EcBoolean retVal = m_pPlugin->setConfiguration(config);
   setCursor(Qt::ArrowCursor);

   if (retVal)
   {
      QDialog::accept();
   }
   else
   {
      const EcString message = EcString("Failed to configure ROS Robot Controller.");
      QMessageBox::critical(this, "ROS Robot Controller Failure", message.c_str());
   }
}

//---
// EcRos1RobotStatePubSubConfigDialog protected methods
//---

//---------------------------------------------------------------------------
void EcRos1RobotControllerConfigDialog::setFromConfiguration
   (
   )
{
   if(!m_pPlugin)
   {
      return;
   }

   EcRos1RobotControllerPluginConfig config = m_pPlugin->configuration();

   EcStringVector manipulatorNames;
   {
      EcSharedMutexLock lock;
      m_pPlugin->getParam<Ec::SimulationMutex>(lock);

      const EcIndividualManipulatorVector* pManips = m_pPlugin->paramPtr<Ec::Manipulator, EcIndividualManipulatorVector>();
      if (pManips)
      {
         const EcIndividualManipulatorVector& manips = *pManips;
         const EcSizeT numManips = manips.size();
         try
         {
            m_UiPtr->activeManipulatorSpinBox->setMaximum(boost::numeric_cast<int>(numManips)-1);
         }
         catch (boost::numeric::bad_numeric_cast& error)
         {
            EcPrint(Error) << "Bad cast exception setting spinbox maximum " << error.what() << std::endl;
         }
         for (EcU32 ii = 0; ii < numManips; ++ii)
         {
            manipulatorNames.push_back(boost::replace_all_copy(manips[ii].manipulatorLabel(), "-", "_"));
            boost::replace_all(manipulatorNames[ii], " ", "_");
            boost::replace_all(manipulatorNames[ii], ".", "_");
         }
      }
   }

   m_UiPtr->activeManipulatorSpinBox->setValue(config.m_ManipIndex);
   m_UiPtr->publishFrequencyDoubleSpinBox->setValue(config.publish_frequency);
   m_UiPtr->actinJointStatesTopicNameLineEdit->setText(config.publishJointStatesTopicName.value().c_str());
   m_UiPtr->jointStatesTopicNameLineEdit->setText(config.feedbackJointStatesTopicName.value().c_str());
   m_UiPtr->systemCoordinateFrameLineEdit->setText(config.rosWorldFrameName.value().c_str());

   const EcSizeT numManips = manipulatorNames.size();
   if (config.manipulatorConfigs.size() != numManips)
   {
      config.manipulatorConfigs.resize(numManips);
   }

   m_UiPtr->tableWidget->setRowCount(static_cast<int>(numManips));
   for (EcU32 ii = 0; ii < numManips; ++ii)
   {
      const EcString& name = manipulatorNames[ii];
      const EcRos1HardwareManipulatorConfig& mconfig = config.manipulatorConfigs[ii];

      QTableWidgetItem* pItem = new QTableWidgetItem(name.c_str());
      m_UiPtr->tableWidget->setItem(ii, 0, pItem);

      QLineEdit* pLineEdit = new QLineEdit(m_UiPtr->tableWidget);
      pLineEdit->setText(mconfig.tf_prefix.value().c_str());
      m_UiPtr->tableWidget->setCellWidget(ii, 1, pLineEdit);
   }

   m_UiPtr->tableWidget->resizeColumnsToContents();
   m_UiPtr->tableWidget->setFixedSize
      (
      m_UiPtr->tableWidget->horizontalHeader()->length() + 25,
      m_UiPtr->tableWidget->verticalHeader()->length() + 29
      );

   adjustSize();
   setFixedSize(size());
}
