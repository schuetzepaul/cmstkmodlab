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

#ifndef ASSEMBLYASSEMBLYACTIONWIDGETGROUP_H
#define ASSEMBLYASSEMBLYACTIONWIDGETGROUP_H

#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QToolButton>
#include <QCheckBox>
#include <QLabel>
#include <AssemblyAssemblyActionWidget.h>

class AssemblyAssemblyActionWidgetGroup : public QWidget
{
 Q_OBJECT

 public:
  explicit AssemblyAssemblyActionWidgetGroup(QWidget* parent=nullptr);
  virtual ~AssemblyAssemblyActionWidgetGroup() {}

  QVBoxLayout* layout() const { return layout_; }

  QLabel* label() const { return label_; }
  QPushButton* button() const { return button_; }
  QCheckBox* checkbox() const { return checkbox_; }

  void connect_group(const QObject*, const char*, const char*, const char* = nullptr);

  int add_action(AssemblyAssemblyActionWidget*);

 protected:
  QVBoxLayout* layout_;
  QHBoxLayout* main_button_layout_;
  QVBoxLayout* individual_actions_layout_;

  QLabel* label_;
  QPushButton* button_;
  QCheckBox* checkbox_;

  QToolButton* toggle_button_;

  const QObject* qobject_;
  const char* start_slot_;
  const char* stop_signal_;
  const char* abort_signal_;

  bool inhibit_dialogue_;

  uint n_actions_performed_;

  std::vector<AssemblyAssemblyActionWidget*> m_vec_actions;

 public slots:
  void disable(const bool b=true);
  void disable(const int);

  void reset_group();
  void next_action();
  void disable_group();
  void abort_group();

  void toggle_view(bool);

  void refresh_checkStates(int);

 signals:
  void group_request();
  void perform_action_request();
};

#endif // ASSEMBLYASSEMBLYACTIONWIDGETGROUP_H
