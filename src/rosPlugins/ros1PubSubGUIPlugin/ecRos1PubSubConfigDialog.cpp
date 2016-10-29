//------------------------------------------------------------------------------
// Copyright (c) 2016 Energid Technologies. All rights reserved.
//
/// @file ecRos1PubSubConfigDialog.cpp
//
//------------------------------------------------------------------------------
#include "ecRos1PubSubConfigDialog.h"

#include <ros1PubSubPlugin/ros1PubSubPlugin.h>
#include <ros1PubSubPlugin/ecRos1PubSubPluginConfig.h>
#include <manipulator/ecIndManipulator.h>
#include <plugins/ecIOParams.h>

#include "ui_ecRos1PubSubConfigDialog.h"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QMessageBox>

//---
// EcRos1PubSubConfigDialog constructor
//---

//---------------------------------------------------------------------------
EcRos1PubSubConfigDialog::EcRos1PubSubConfigDialog
   (
   ros1PubSubPlugin* pPlugin,
   QWidget*         pParent
   ) :
   QDialog(pParent, Qt::WindowTitleHint|Qt::WindowSystemMenuHint),
   m_UiPtr(new Ui::EcRos1PubSubConfigDialog()),
   m_pPlugin(pPlugin)
{
   m_UiPtr->setupUi(this);

   setFromConfiguration();
   
   connect(m_UiPtr->buttonBox, SIGNAL(accepted()), SLOT(accept()));
   connect(m_UiPtr->buttonBox, SIGNAL(rejected()), SLOT(reject()));
}

//---
// EcRos1PubSubConfigDialog destructor
//---

//---------------------------------------------------------------------------
EcRos1PubSubConfigDialog::~EcRos1PubSubConfigDialog
   (
   )
{
}

//---
// EcRos1PubSubConfigDialog protected slots
//---

//---------------------------------------------------------------------------
void EcRos1PubSubConfigDialog::accept
   (
   )
{
   setCursor(Qt::WaitCursor);
   EcRos1PubSubPluginConfig config;

   config.jointAnglesTopicName      = m_UiPtr->jointTopicLineEdit->text().toStdString().c_str();
   config.desiredPlacementTopicName = m_UiPtr->eeLineEdit->text().toStdString().c_str();

   const EcInt32 numRows = m_UiPtr->tableWidget->rowCount();
   config.manipulatorConfigs.resize(numRows);
   for (EcInt32 ii = 0; ii < numRows; ++ii)
   {
      QComboBox* pModeComboBox =
         dynamic_cast<QComboBox*>(m_UiPtr->tableWidget->cellWidget(ii, 1));
      if (pModeComboBox)
      {
         EcRos1PubSubManipulatorConfig& mconfig = config.manipulatorConfigs[ii];
         mconfig.mode = pModeComboBox->currentIndex();
      }
   }

   EcBoolean retVal = m_pPlugin->setConfiguration(config);
   setCursor(Qt::ArrowCursor);

   if (retVal)
   {
      QDialog::accept();
   }
   else
   {
      const EcString message = EcString("Failed to configure ROS Publish/Subscribe.");
      QMessageBox::critical(this, "ROS Publish/Subscribe Failure", message.c_str());
   }
}

//---
// EcRos1PubSubConfigDialog protected methods
//---

//---------------------------------------------------------------------------
void EcRos1PubSubConfigDialog::setFromConfiguration
   (
   )
{
   if(!m_pPlugin)
   {
      return;
   }

   EcRos1PubSubPluginConfig config = m_pPlugin->configuration();

   m_UiPtr->jointTopicLineEdit->setText(config.jointAnglesTopicName.value().c_str());

   m_UiPtr->eeLineEdit->setText(config.desiredPlacementTopicName.value().c_str());

   EcStringVector manipulatorNames;
   {
      EcSharedMutexLock lock;
      m_pPlugin->getParam<Ec::SimulationMutex>(lock);

      const EcIndividualManipulatorVector* pManips = m_pPlugin->paramPtr<Ec::Manipulator, EcIndividualManipulatorVector>();
      if (pManips)
      {
         const EcIndividualManipulatorVector& manips = *pManips;
         const EcSizeT numManips = manips.size();
         for (EcU32 ii = 0; ii < numManips; ++ii)
         {
            manipulatorNames.push_back(manips[ii].manipulatorLabel());
         }
      }
   }

   const EcSizeT numManips = manipulatorNames.size();
   if (config.manipulatorConfigs.size() != numManips)
   {
      config.manipulatorConfigs.resize(numManips);
   }

   m_UiPtr->tableWidget->setRowCount(static_cast<int>(numManips));
   for (EcU32 ii = 0; ii < numManips; ++ii)
   {
      const EcString& name = manipulatorNames[ii];
      const EcRos1PubSubManipulatorConfig& mconfig = config.manipulatorConfigs[ii];

      QTableWidgetItem* pItem = new QTableWidgetItem(name.c_str());
      m_UiPtr->tableWidget->setItem(ii, 0, pItem);

      QComboBox* pModeComboBox = new QComboBox(m_UiPtr->tableWidget);
      pModeComboBox->addItem("IGNORE");
      pModeComboBox->addItem("PUB_EE_PUB_JOINT");
      pModeComboBox->addItem("PUB_EE_SUB_JOINT");
      pModeComboBox->addItem("SUB_EE_PUB_JOINT");
      pModeComboBox->addItem("SUB_EE");
      pModeComboBox->addItem("SUB_JOINT");
      pModeComboBox->setCurrentIndex(mconfig.mode);
      m_UiPtr->tableWidget->setCellWidget(ii, 1, pModeComboBox);
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
