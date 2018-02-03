#include "../include/horus_patrol/patrol_dialog.hpp"

namespace HorusPatrol
{

PatrolDialog::PatrolDialog(ROSNode* node, horusRViz* rviz_interface,
                           bool admin_profile, QWidget* parent)
    : QDialog(parent), ros_node_(node), myviz_(rviz_interface),
      ui_(new Ui::PatrolDialog), admin_(admin_profile)
{
  QSqlQuery query;
  query.exec("SELECT SCHEMA_NAME FROM INFORMATION_SCHEMA.SCHEMATA WHERE "
             "SCHEMA_NAME = 'routes'");
  if (!query.next())
  {
    query.exec("CREATE DATABASE IF NOT EXISTS `routes`;");
    query.exec("CREATE DATABASE IF NOT EXISTS `threats`;");
    QMessageBox::information(this, tr("Horus Patrol Information"),
                             tr("A new 'Routes' and 'Threats' databases were created."));
  }
  query.exec("USE routes;");

  ui_->setupUi(this);
  if (!admin_)
    ui_->routeManager_pushbutton->setVisible(false);
  ui_->save_pushButton->setEnabled(false);
  updateRoutesList();

  QObject::connect(ui_->cancel_pushButton, SIGNAL(clicked()), this,
                   SLOT(reject()));
  QObject::connect(ui_->save_pushButton, SIGNAL(clicked()), this,
                   SLOT(accept()));
}

PatrolDialog::~PatrolDialog() { delete ui_; }

void PatrolDialog::on_randomRoute_checkBox_clicked(bool checked)
{
  if (checked)
  {
    qsrand(qrand());
    short int random_route =
        qrand() % (ui_->routeList_comboBox->count() - 1) + 1;
    ui_->routeList_comboBox->setCurrentIndex(random_route);
    ui_->routeList_comboBox->setEnabled(false);
  }
  else
    ui_->routeList_comboBox->setEnabled(true);
}

void PatrolDialog::on_initialPose_checkBox_clicked(bool checked)
{
  if (checked)
    QMessageBox::warning(
        this, tr("Horus Patrol information"),
        tr("WARNING!<br>Use this mode only if the robot knows where it is."));
}

void PatrolDialog::on_routeManager_pushbutton_clicked()
{
  route_manager_dialog_ = new RouteManagerDialog(ros_node_, myviz_, this);
  ros_node_->managing_route = true;
  route_manager_dialog_->exec();
  myviz_->setParent(this);
  updateRoutesList();
  ros_node_->managing_route = false;
  myviz_->robot_->setEnabled(false);
  delete route_manager_dialog_;
}

void PatrolDialog::on_save_pushButton_clicked()
{
  short int count = 0;
  QSqlQuery query;
  if (query.exec(
          "SELECT `Name`, `PositionX`, `PositionY`, `Orientation` FROM `" +
          ui_->routeList_comboBox->currentText() + "`;"))
  {
    query.next();
    ros_node_->patrol_initial_pose_name = query.value(0).toString();
    ros_node_->patrol_initial_pose_x = query.value(1).toFloat();
    ros_node_->patrol_initial_pose_y = query.value(2).toFloat();
    ros_node_->patrol_initial_pose_orientation = query.value(3).toFloat();

    while (query.next())
    {
      count++;
      ros_node_->patrol_places_name.push_back(query.value(0).toString());
      ros_node_->patrol_places_x.push_back(query.value(1).toFloat());
      ros_node_->patrol_places_y.push_back(query.value(2).toFloat());
      ros_node_->patrol_places_orientation.push_back(query.value(3).toFloat());
    }
    ros_node_->total_patrol_places = count;
    ros_node_->selected_route = ui_->routeList_comboBox->currentText();
    if (ui_->initialPose_checkBox->isChecked())
    {
      ros_node_->patrol_initial_pose_name = "Custom Initial Pose";
    }
  }
  else
  {
    QMessageBox::critical(this, tr("Horus Patrol error"),
                          tr("Something went wrong. Could not set route."));
  }
}

void PatrolDialog::on_routeList_comboBox_currentIndexChanged(int index)
{
  ui_->textBrowser->clear();
  if (index)
  {
    ui_->save_pushButton->setEnabled(true);
    ui_->textBrowser->insertHtml("<b>Patrol Sequence:</b><br>");
    try
    {
      QSqlQuery query;
      query.exec("SELECT `Name` FROM `" +
                 ui_->routeList_comboBox->currentText() + "`;");

      short int count = 0;
      while (query.next())
      {
        count++;
        ui_->textBrowser->insertHtml(
            QString("   %1)  " + query.value(0).toString()).arg(count));
        ui_->textBrowser->insertHtml("<br>");
      }
    }
    catch (...)
    {
      QMessageBox::critical(this, tr("Database Error"),
                            tr("Could not load information from database."));
    }
  }
  else
  {
    ui_->save_pushButton->setEnabled(false);
  }
}

void PatrolDialog::updateRoutesList()
{
  ui_->routeList_comboBox->blockSignals(true);
  ui_->routeList_comboBox->clear();

  QSqlQuery query;
  query.exec("SELECT COUNT(*) FROM information_schema.tables WHERE "
             "table_schema = 'routes';");
  query.next();
  if (query.value(0).toInt() < 2)
  {
    ui_->routeList_comboBox->addItem("No Routes Registered");
    ui_->routeList_comboBox->setEnabled(false);
    QString message;
    if (admin_)
      message = "There are no Routes registered in the System.<br><br>You must "
                "create at least one route to use the patrolling mode.";
    else
      message = "There are no Routes registered in the System.<br><br>You must "
                "create at least one route to use the patrolling "
                "mode.<br> Contact your System Administrator for "
                "support.";
    QMessageBox::information(this, "Horus Patrol Information", message);
  }
  else
  {
    ui_->routeList_comboBox->addItem("Select a Route...");
    ui_->routeList_comboBox->setEnabled(true);
    try
    {
      // load registered routes
      query.exec("SHOW TABLES FROM routes;");
      while (query.next())
      {
        if (query.value(0).toString().compare("temp_route"))
          ui_->routeList_comboBox->addItem(query.value(0).toString());
      }
    }
    catch (...)
    {
      QMessageBox::critical(this, tr("Horus Patrol error"),
                            tr("Could not access 'routes' database."));
    }
  }
  on_routeList_comboBox_currentIndexChanged(0);
  ui_->routeList_comboBox->blockSignals(false);
}
}
