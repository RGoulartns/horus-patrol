#ifndef ROUTEMANAGER_DIALOG_HPP
#define ROUTEMANAGER_DIALOG_HPP

#include <QSqlTableModel>
#include <QSqlQuery>
#include <QDir>
#include <QMessageBox>
#include <QDialogButtonBox>
#include "ros_node.hpp"
#include "rviz_interface.hpp"
#include "horus_logger.hpp"
#include "ui_routemanager_dialog.h"

namespace HorusPatrol
{

class RouteManagerDialog : public QDialog
{
  Q_OBJECT

public:
  explicit RouteManagerDialog(ROSNode* node, horusRViz* rviz_interface,
                              QWidget* parent = 0);
  ~RouteManagerDialog();

public Q_SLOTS:
  void robotLocalized();
  void updateButtonsState();
  void tableViewRowChanged(const QModelIndex& index);
  void on_addPlace_pushButton_clicked();
  void on_clearPlace_pushButton_clicked();
  void on_deleteRoute_pushButton_clicked();
  void on_routeList_comboBox_currentIndexChanged(int index);
  void on_deleteIndex_pushButton_clicked();
  void on_getPose_pushButton_clicked();
  void on_routeManager_tabWidget_currentChanged(int index);
  void on_save_pushButton_clicked();

private:
  Ui::RouteManagerDialog* ui_;
  horusRViz* myviz_;
  ROSNode* ros_node_;
  QSqlTableModel* model_;
  geometry_msgs::PoseWithCovarianceStamped place_pose_;
  QString route_name_, horus_threats_path;
  void updateComboBoxRoutesList();
  void updateDialog();
};
}

#endif // ROUTE_MANAGER_HPP
