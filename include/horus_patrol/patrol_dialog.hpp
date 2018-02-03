#ifndef PATROL_DIALOG_H
#define PATROL_DIALOG_H

#include <QMessageBox>
#include "ui_patrol_dialog.h"
#include "ros_node.hpp"
#include "routemanager_dialog.hpp"
#include "rviz_interface.hpp"
#include "horus_logger.hpp"

namespace HorusPatrol
{

class PatrolDialog : public QDialog
{
  Q_OBJECT

public:
  explicit PatrolDialog(ROSNode* node, horusRViz* rviz_interface,
                        bool admin_profile, QWidget* parent = 0);
  ~PatrolDialog();

public Q_SLOTS:
  void on_routeList_comboBox_currentIndexChanged(int index);
  void on_initialPose_checkBox_clicked(bool checked);
  void on_randomRoute_checkBox_clicked(bool checked);
  void on_routeManager_pushbutton_clicked();
  void on_save_pushButton_clicked();

private:
  bool admin_;
  horusRViz* myviz_;
  ROSNode* ros_node_;
  RouteManagerDialog* route_manager_dialog_;
  Ui::PatrolDialog* ui_;
  void updateRoutesList();
};
}
#endif // PATROLDIALOG_H
