/////////////////////////////////////////////////////////////////////////////////
//                                                                             //
//               Copyright (C) 2011-2025 - The DESY CMS Group                  //
//                           All rights reserved                               //
//                                                                             //
//      The CMStkModLab source code is licensed under the GNU GPL v3.0.        //
//      You have the right to modify and/or redistribute this source code      //
//      under the terms specified in the license, which may be found online    //
//      at http://www.gnu.org/licenses or at License.txt.                      //
//                                                                             //
/////////////////////////////////////////////////////////////////////////////////

#include <nqlogger.h>
#include <AssemblyAssemblyActionWidgetGroup.h>

#include <QMessageBox>

AssemblyAssemblyActionWidgetGroup::AssemblyAssemblyActionWidgetGroup(QWidget* parent)
 : QWidget(parent)

 , layout_(nullptr)

 , label_(nullptr)
 , button_(nullptr)
 , checkbox_(nullptr)

 , qobject_(nullptr)
 , start_slot_(nullptr)
 , stop_signal_(nullptr)

 , inhibit_dialogue_(false)
{
  // layout
  layout_ = new QVBoxLayout;
  this->setLayout(layout_);

  main_button_layout_ = new QHBoxLayout;
  label_ = new QLabel;
  label_->setStyleSheet("QLabel { font-weight : bold; }");

  button_ = new QPushButton;
  button_->setStyleSheet(
    "Text-align: left;"
    "padding-left:   4px;"
    "padding-right:  4px;"
    "padding-top:    3px;"
    "padding-bottom: 3px;"
  );

  connect(this->button(), SIGNAL(clicked()), this, SLOT(next_action()));

  checkbox_ = new QCheckBox("Done");
  checkbox_->setTristate(true);
  checkbox_->setCheckState(Qt::Unchecked);
  connect(checkbox_, SIGNAL(stateChanged(int)), this, SLOT(disable(int)));

  toggle_button_ = new QToolButton;
  toggle_button_->setStyleSheet("QToolButton { border: none; padding-left: 0px; padding-right: 0px; width: 16px}");
  toggle_button_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  toggle_button_->setArrowType(Qt::ArrowType::RightArrow);
  //toggle_button_->setText("None");
  toggle_button_->setCheckable(true);
  toggle_button_->setChecked(false);
  connect(toggle_button_, SIGNAL(clicked(bool)), this, SLOT(toggle_view(bool)));

  main_button_layout_->addWidget(toggle_button_, 2, Qt::AlignLeft);
  main_button_layout_->addWidget(label_,  2, Qt::AlignRight);
  main_button_layout_->addWidget(button_, 40);
  main_button_layout_->addWidget(new QLabel, 46);
  main_button_layout_->addWidget(checkbox_, 10);

  layout_->addLayout(main_button_layout_);

  individual_actions_layout_ = new QVBoxLayout();
  layout_->addLayout(individual_actions_layout_);
}

void AssemblyAssemblyActionWidgetGroup::disable(const bool to_be_disabled)
{
  this->disable(int(to_be_disabled ? 2 : 0));
}

void AssemblyAssemblyActionWidgetGroup::disable(const int state)
{
  if(state == 2)
  {
    if(!inhibit_dialogue_)
    {
      QMessageBox* msgBox = new QMessageBox;
      msgBox->setStyleSheet("QLabel{min-width: 300px;}");
      msgBox->setInformativeText("WARNING: this will not perform any action but just mark the step as \"done\".\nWould you like to proceed?");
      msgBox->setStandardButtons(QMessageBox::No | QMessageBox::Yes);
      msgBox->setDefaultButton(QMessageBox::Yes);
      int ret = msgBox->exec();
      switch(ret)
      {
        case QMessageBox::No:
          checkbox_->setCheckState(Qt::Unchecked);
          return;
        case QMessageBox::Yes:
          break;
        default: return;
      }
    } else {
      inhibit_dialogue_ = false;
    }
    label_->setEnabled(false);
    button_->setEnabled(false);

    for(auto& action : m_vec_actions){
        action->disableSilently();
    }
  }
  else if(state == 0)
  {
    label_->setEnabled(true);
    button_->setEnabled(true);

    n_actions_performed_ = 0;

    for(auto& action : m_vec_actions){
        action->disable(false);
    }
  } else {
    inhibit_dialogue_ = true;

    label_->setEnabled(false);
    button_->setEnabled(false);
  }

  return;
}

void AssemblyAssemblyActionWidgetGroup::connect_group(const QObject* qobject, const char* start_slot, const char* stop_signal, const char* abort_signal)
{
  if(qobject_){ this->reset_group(); }

  if(qobject)
  {
    qobject_ = qobject;
    start_slot_ = start_slot;
    stop_signal_ = stop_signal;
    abort_signal_ = abort_signal;

    //connect(this->button(), SIGNAL(clicked()), this, SLOT(next_action()));

    n_actions_performed_ = 0;
  }
  else
  {
    NQLog("AssemblyAssemblyActionWidgetGroup", NQLog::Warning) << "connect_group"
       << ": invalid (NULL) input pointer to QObject, no action taken";
  }
}

void AssemblyAssemblyActionWidgetGroup::reset_group()
{
  qobject_ = nullptr;
  start_slot_ = nullptr;
  stop_signal_ = nullptr;
  abort_signal_ = nullptr;

  n_actions_performed_ = 0;
}

void AssemblyAssemblyActionWidgetGroup::next_action()
{
  NQLog("AssemblyAssemblyActionWidgetGroup", NQLog::Warning) << "next_action: "
       << "Initiating action #" << n_actions_performed_+1 << " out of " << m_vec_actions.size();

  button_->setStyleSheet(
         "Text-align: left;"
         "padding-left:   4px;"
         "padding-right:  4px;"
         "padding-top:    3px;"
         "padding-bottom: 3px;"
         "color: red;"
       );

  if(qobject_)
  {
    if(n_actions_performed_ > 0){
        disconnect(this, SIGNAL(perform_action_request()), m_vec_actions.at(n_actions_performed_-1), SLOT(start_action()));
        disconnect(qobject_, m_vec_actions.at(n_actions_performed_-1)->stop_signal(), this, SLOT(next_action()));
    }

    connect(this, SIGNAL(perform_action_request()), m_vec_actions.at(n_actions_performed_), SLOT(start_action()));

    if(n_actions_performed_ < m_vec_actions.size()-1){
        NQLog("AssemblyAssemblyActionWidgetGroup", NQLog::Warning) << "next_action: "
            << "Connecting stop signal (" << m_vec_actions.at(n_actions_performed_)->stop_signal() << ") to next action.";

        connect(qobject_, m_vec_actions.at(n_actions_performed_)->stop_signal(), this, SLOT(next_action()));

        if(abort_signal_ != nullptr)
        {
            connect(qobject_, abort_signal_, this, SLOT(abort_group()));
        }
    } else {
        connect(qobject_, m_vec_actions.at(n_actions_performed_)->stop_signal(), this, SLOT(disable_group()));
    }

    ++n_actions_performed_;
    emit perform_action_request();
  }
  else
  {
    NQLog("AssemblyAssemblyActionWidgetGroup", NQLog::Warning) << "start_group"
       << ": invalid (NULL) pointer to QObject, no action taken";
  }
}

void AssemblyAssemblyActionWidgetGroup::disable_group()
{
  if(qobject_)
  {
    disconnect(qobject_, m_vec_actions.at(n_actions_performed_-1)->stop_signal(), this, SLOT(disable_group()));

    if(n_actions_performed_ > 0){
      disconnect(this, SIGNAL(perform_action_request()), m_vec_actions.at(n_actions_performed_-1), SLOT(start_action()));
      disconnect(qobject_, m_vec_actions.at(n_actions_performed_-1)->stop_signal(), this, SLOT(next_action()));
    }

    inhibit_dialogue_ = true;
    checkbox_->setCheckState(Qt::Checked);

    button_->setStyleSheet(
           "Text-align: left;"
           "padding-left:   4px;"
           "padding-right:  4px;"
           "padding-top:    3px;"
           "padding-bottom: 3px;"
         );

    n_actions_performed_ = 0;
    NQLog("AssemblyAssemblyActionWidgetGroup", NQLog::Warning) << "disable_group"
       << ": n_actions_performed = 0";

  }
  else
  {
    NQLog("AssemblyAssemblyActionWidgetGroup", NQLog::Warning) << "disable_group"
       << ": invalid (NULL) pointer to QObject, no action taken";
  }
}

void AssemblyAssemblyActionWidgetGroup::abort_group()
{
  if(qobject_)
  {
    disconnect(this, SIGNAL(group_request()), qobject_, start_slot_);

    disconnect(qobject_, stop_signal_, this, SLOT(disable_group()));

    disconnect(qobject_, abort_signal_, this, SLOT(abort_group()));

    inhibit_dialogue_ = true;
  }
  else
  {
    NQLog("AssemblyAssemblyActionWidgetGroup", NQLog::Warning) << "abort_group"
       << ": invalid (NULL) pointer to QObject, no action taken";
  }
}

int AssemblyAssemblyActionWidgetGroup::add_action(AssemblyAssemblyActionWidget* action){
    m_vec_actions.push_back(action);

    NQLog("AssemblyAssemblyActionWidgetGroup", NQLog::Warning) << "toggle_view"
       << ": Add layout!";
    individual_actions_layout_->addWidget(action);
    action->setVisible(false);
    action->set_belongs_to_group(true);

    return m_vec_actions.size();
}

void AssemblyAssemblyActionWidgetGroup::toggle_view(bool checked){
    NQLog("AssemblyAssemblyActionWidgetGroup", NQLog::Warning) << "toggle_view"
       << ": This should be toggled now";

    toggle_button_->setArrowType(checked ? Qt::ArrowType::DownArrow : Qt::ArrowType::RightArrow);

    for(auto& action : m_vec_actions){
        action->setVisible(checked);
    }
}

void AssemblyAssemblyActionWidgetGroup::refresh_checkStates(int checked){
    NQLog("AssemblyAssemblyActionWidgetGroup", NQLog::Warning) << "refresh_checkStates"
       << ": Let's check all states!";

       int actions_checked = 0;
       for(auto& action : m_vec_actions){
           actions_checked += (action->checkbox()->isChecked() ? 1 : 0);
       }

       if(actions_checked==m_vec_actions.size()){
           checkbox_->setCheckState(Qt::Checked);
       } else if(actions_checked==0){
           checkbox_->setCheckState(Qt::Unchecked);
       } else {
           checkbox_->setCheckState(Qt::PartiallyChecked);
       }
}
