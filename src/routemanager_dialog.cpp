#include "../include/horus_patrol/routemanager_dialog.hpp"

namespace HorusPatrol
{

RouteManagerDialog::RouteManagerDialog(ROSNode* node, horusRViz* rviz_interface,
                                       QWidget* parent)
    : QDialog(parent), ros_node_(node), myviz_(rviz_interface),
      route_name_("temp_route"), ui_(new Ui::RouteManagerDialog)
{

  // create routes threats folder
  horus_threats_path = QSettings("Horus Settings", "horus_patrol_settings")
                           .value("horus_path")
                           .toString() +
                       "/Detected Threats/";
  if (!QDir(horus_threats_path).exists())
    QDir().mkdir(horus_threats_path);

  ui_->setupUi(this);
  ui_->gridLayout_2->addWidget(myviz_, 0, 0, 0, 1);
  ui_->poseX_lineEdit->setValidator(new QDoubleValidator(this));
  ui_->poseY_lineEdit->setValidator(new QDoubleValidator(this));
  ui_->orientation_lineEdit->setValidator(new QDoubleValidator(this));
  ui_->placeInfo_groupBox->setStyleSheet(
      "QGroupBox { font-weight: bold; } "
      "QGroupBox::title {subcontrol-position: top;}");
  model_ = new QSqlTableModel(this);
  model_->setTable(route_name_);
  model_->setEditStrategy(QSqlTableModel::OnFieldChange);
  model_->select();
  ui_->route_tableView->setModel(model_);
  ui_->route_tableView->sortByColumn(0, Qt::AscendingOrder);
  // ui_->route_tableView->setSelectionMode(QAbstractItemView::SingleSelection);

  QObject::connect(ui_->cancel_pushButton, SIGNAL(clicked()), this,
                   SLOT(reject()));
  QObject::connect(ros_node_, SIGNAL(robotLocalized()), this,
                   SLOT(robotLocalized()));
  QObject::connect(ui_->name_lineEdit, SIGNAL(textChanged(QString)), this,
                   SLOT(updateButtonsState()));
  QObject::connect(ui_->poseX_lineEdit, SIGNAL(textChanged(QString)), this,
                   SLOT(updateButtonsState()));
  QObject::connect(ui_->poseY_lineEdit, SIGNAL(textChanged(QString)), this,
                   SLOT(updateButtonsState()));
  QObject::connect(ui_->orientation_lineEdit, SIGNAL(textChanged(QString)),
                   this, SLOT(updateButtonsState()));
  QObject::connect(ui_->route_tableView->selectionModel(),
                   SIGNAL(currentRowChanged(QModelIndex, QModelIndex)), this,
                   SLOT(tableViewRowChanged(QModelIndex)));
  QObject::connect(ui_->index_spinBox, SIGNAL(valueChanged(int)),
                   ui_->route_tableView, SLOT(selectRow(int)));

  QSqlQuery query;
  query.exec("USE routes;");
  // check if it needs a new temp_route
  query.exec("SELECT * FROM information_schema.tables WHERE "
             "table_schema = `routes` AND table_name = `" +
             route_name_ + "` LIMIT 1;");
  if (!query.next())
  {
    query.exec("CREATE TABLE `" + route_name_ +
               "` (`Index` INT NOT NULL, `Name` VARCHAR(60), `PositionX` "
               "FLOAT NOT NULL, `PositionY` FLOAT NOT NULL, `Orientation` "
               "FLOAT NOT NULL, PRIMARY KEY (`Index`));");
  }

  place_pose_.header.frame_id = "map";
  place_pose_.pose.pose.position.z = 0;

  ui_->routeManager_tabWidget->setCurrentIndex(0);
  on_routeManager_tabWidget_currentChanged(0);
}

RouteManagerDialog::~RouteManagerDialog() { delete ui_; }

void RouteManagerDialog::on_save_pushButton_clicked()
{
  QDialog* dialog = new QDialog(this);
  dialog->setWindowTitle("Save Place");
  dialog->setFixedSize(240, 150);
  QVBoxLayout* verticalLayout = new QVBoxLayout(dialog);
  QSpacerItem* verticalSpacer =
      new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
  verticalLayout->addItem(verticalSpacer);
  QHBoxLayout* horizontalLayout = new QHBoxLayout();
  QLabel* label = new QLabel(dialog);
  label->setText("Route Name: ");
  horizontalLayout->addWidget(label);
  QLineEdit* lineEdit = new QLineEdit(dialog);
  lineEdit->setText("Route Name");
  lineEdit->setFocus();
  horizontalLayout->addWidget(lineEdit);
  verticalLayout->addLayout(horizontalLayout);
  QSpacerItem* verticalSpacer_2 =
      new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
  verticalLayout->addItem(verticalSpacer_2);
  QDialogButtonBox* buttonBox = new QDialogButtonBox(dialog);
  buttonBox->setOrientation(Qt::Horizontal);
  buttonBox->setStandardButtons(QDialogButtonBox::Cancel |
                                QDialogButtonBox::Save);
  buttonBox->setCenterButtons(true);
  verticalLayout->addWidget(buttonBox);
  QObject::connect(buttonBox, SIGNAL(accepted()), dialog, SLOT(accept()));
  QObject::connect(buttonBox, SIGNAL(rejected()), dialog, SLOT(reject()));

  bool save_route = false;
  QSqlQuery query;

  do
  {
    dialog->exec();
    if (dialog->result())
    {
      if (lineEdit->text().isEmpty())
      {
        QMessageBox::critical(this, tr("Save Route"),
                              tr("Name must not be empty."));
      }
      else
      {
        if (query.exec("SELECT * FROM " + lineEdit->text() + ";"))
        {
          QMessageBox::StandardButton reply;
          reply = QMessageBox::question(
              this, tr("Save Route"),
              QString(
                  "Route '" + lineEdit->text() +
                  "' already exists."
                  "<br>Do you want to replace it? All Threats will be lost."),
              QMessageBox::Yes | QMessageBox::No);
          if (reply == QMessageBox::Yes)
          {
            query.exec("DROP TABLE `routes`.`" + lineEdit->text() + "`;");
            query.exec("DROP TABLE `threats`.`" + lineEdit->text() + "`;");
            QDir(horus_threats_path + lineEdit->text()).removeRecursively();

            save_route = true;
          }
        }
        else
        {
          save_route = true;
        }
      }
    }
  } while ((!save_route) && (dialog->result()));
  if (save_route)
  {
    // create tab
    if (!ui_->routeManager_tabWidget->currentIndex())
    {
      try
      {
        query.exec("ALTER TABLE `routes`.`" + route_name_ +
                   "` RENAME TO `routes`.`" + lineEdit->text() + "`;");
        query.exec("CREATE TABLE `" + route_name_ +
                   "` (`Index` INT NOT NULL, `Name` VARCHAR(60), `PositionX` "
                   "FLOAT NOT NULL, `PositionY` FLOAT NOT NULL, `Orientation` "
                   "FLOAT NOT NULL, PRIMARY KEY (`Index`));");
        query.exec("CREATE TABLE `threats`.`" + lineEdit->text() +
                   "` (`Time` VARCHAR(30),`SurroundingArea` VARCHAR(60),"
                   "`PositionX` FLOAT NOT NULL,`PositionY` FLOAT NOT "
                   "NULL, `Orientation` FLOAT NOT NULL, PRIMARY KEY "
                   "(`Time`));");
        QDir(horus_threats_path).mkdir(lineEdit->text());

        HorusLogger* logger = new HorusLogger(
            this, "PatrolDetections.txt",
            QString("NEW Route created. Name: `%1`").arg(lineEdit->text()));
        logger->start();

        QMessageBox::information(this, tr("Horus Patrol information"),
                                 tr(QString("Route '" + lineEdit->text() +
                                            "' saved successfully.").toUtf8()));

        updateDialog();
      }
      catch (...)
      {
        QMessageBox::critical(this, tr("Database Error"),
                              tr("Could not save information."));
      }
    }
    // edit tab
    else
    {
      try
      {
        query.exec("ALTER TABLE `routes`.`" + route_name_ +
                   "` RENAME TO  `routes`.`" + lineEdit->text() + "`;");
        query.exec("ALTER TABLE `threats`.`" + route_name_ +
                   "` RENAME TO  `threats`.`" + lineEdit->text() + "`;");
        QFile::rename(horus_threats_path + route_name_,
                      horus_threats_path + lineEdit->text());

        HorusLogger* logger = new HorusLogger(
            this, "PatrolDetections.txt", QString("Route '%1' renamed to '%2'")
                                              .arg(route_name_)
                                              .arg(lineEdit->text()));
        logger->start();

        QMessageBox::information(
            this, tr("Horus Patrol information"),
            tr(QString("Route '" + route_name_ + "' was renamed to '" +
                       lineEdit->text() + "' successfully.").toUtf8()));

        updateComboBoxRoutesList();
        updateDialog();
      }
      catch (...)
      {
        QMessageBox::critical(this, tr("Database Error"),
                              tr("Could not save information."));
      }
    }
  }
  delete dialog;
}

void RouteManagerDialog::on_routeManager_tabWidget_currentChanged(int index)
{
  // setup dialog tab
  if (!index)
  {
    // create
    ui_->gridLayout->addWidget(ui_->routeManager_widget, 0, 0, 1, 1);
    ui_->routeList_comboBox->setVisible(false);
    ui_->deleteRoute_pushButton->setText("Clear Route");
    ui_->save_pushButton->setText("Save");
    ui_->cancel_pushButton->setText("C&ancel");

    route_name_ = "temp_route";
  }
  else
  {
    // edit
    ui_->gridLayout_4->addWidget(ui_->routeManager_widget, 0, 0, 1, 1);
    ui_->routeList_comboBox->setVisible(true);
    ui_->deleteRoute_pushButton->setText("Delete Route");
    ui_->save_pushButton->setText("Rename Route");
    ui_->cancel_pushButton->setText("Ex&it");

    updateComboBoxRoutesList();
  }

  updateDialog();
}

void RouteManagerDialog::on_deleteRoute_pushButton_clicked()
{
  QSqlQuery query;
  // create tab
  if (!ui_->routeManager_tabWidget->currentIndex())
  {
    query.exec("TRUNCATE TABLE `" + route_name_ + "`;");
    updateDialog();
  }
  // edit tab
  else
  {
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(
        this, tr("Delete Route"),
        tr(QString("Are you sure you want to delete '%1'?"
                   "<br>All saved threats will be deleted.")
               .arg(route_name_)
               .toUtf8()),
        QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes)
    {
      QSqlQuery query;
      query.exec("DROP TABLE `routes`.`" + route_name_ + "`;");
      query.exec("DROP TABLE `threats`.`" + route_name_ + "`;");
      QDir(horus_threats_path + route_name_).removeRecursively();

      HorusLogger* logger =
          new HorusLogger(this, "PatrolDetections.txt",
                          QString("Route '%1' Deleted.").arg(route_name_));
      logger->start();

      updateComboBoxRoutesList();
      updateDialog();
    }
  }
}

void RouteManagerDialog::on_getPose_pushButton_clicked()
{
  myviz_->toolmanager_ = myviz_->manager_->getToolManager();
  myviz_->mytool_ = myviz_->toolmanager_->addTool("rviz/SetInitialPose");
  myviz_->toolmanager_->setCurrentTool(myviz_->mytool_);
}

void RouteManagerDialog::on_addPlace_pushButton_clicked()
{
  QSqlQuery query;
  if (!query.exec(
          QString("INSERT INTO `routes`.`" + route_name_ +
                  "` (`Index`,`Name`,`PositionX`,`PositionY`,`Orientation`)"
                  " VALUES ('%1','%2','%3','%4',%5);")
              .arg(ui_->index_spinBox->value())
              .arg(ui_->name_lineEdit->text())
              .arg(ui_->poseX_lineEdit->text())
              .arg(ui_->poseY_lineEdit->text())
              .arg(ui_->orientation_lineEdit->text())))
  {
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(
        this, tr("A&dd Place"),
        tr("There is already a place added at Index  %1."
           "<br>Do you want to replace it?").arg(ui_->index_spinBox->value()),
        QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes)
    {
      query.exec(QString("UPDATE `routes`.`" + route_name_ +
                         "` SET `Name`='%1', `PositionX`='%2', "
                         "`PositionY`='%3', `Orientation`='%4' WHERE "
                         "`Index`='%5';")
                     .arg(ui_->name_lineEdit->text())
                     .arg(ui_->poseX_lineEdit->text())
                     .arg(ui_->poseY_lineEdit->text())
                     .arg(ui_->orientation_lineEdit->text())
                     .arg(ui_->index_spinBox->value()));

      updateDialog();
    }
  }
  else
  {
    updateDialog();
  }
}

void RouteManagerDialog::on_clearPlace_pushButton_clicked()
{
  ui_->name_lineEdit->clear();
  ui_->poseX_lineEdit->clear();
  ui_->poseY_lineEdit->clear();
  ui_->orientation_lineEdit->clear();
}

void RouteManagerDialog::on_deleteIndex_pushButton_clicked()
{
  QDialog* dialog = new QDialog(this);
  dialog->setWindowTitle("Delete Place");
  dialog->setFixedSize(240, 150);
  QVBoxLayout* verticalLayout = new QVBoxLayout(dialog);
  QSpacerItem* verticalSpacer_2 =
      new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
  verticalLayout->addItem(verticalSpacer_2);
  QHBoxLayout* horizontalLayout = new QHBoxLayout;
  QSpacerItem* horizontalSpacer =
      new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
  horizontalLayout->addItem(horizontalSpacer);
  QLabel* label = new QLabel(dialog);
  label->setText("Place Index");
  horizontalLayout->addWidget(label);
  QSpinBox* spinBox = new QSpinBox(dialog);
  spinBox->setMinimumSize(QSize(55, 0));
  spinBox->setMinimum(0);
  spinBox->setMaximum(ui_->index_spinBox->maximum() - 1);
  spinBox->setValue(ui_->index_spinBox->value());
  horizontalLayout->addWidget(spinBox);
  QSpacerItem* horizontalSpacer_2 =
      new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
  horizontalLayout->addItem(horizontalSpacer_2);
  verticalLayout->addLayout(horizontalLayout);
  QSpacerItem* verticalSpacer =
      new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
  verticalLayout->addItem(verticalSpacer);
  QDialogButtonBox* buttonBox = new QDialogButtonBox(dialog);
  buttonBox->setOrientation(Qt::Horizontal);
  buttonBox->setStandardButtons(QDialogButtonBox::Cancel |
                                QDialogButtonBox::Ok);
  buttonBox->setCenterButtons(true);
  verticalLayout->addWidget(buttonBox);
  QObject::connect(buttonBox, SIGNAL(accepted()), dialog, SLOT(accept()));
  QObject::connect(buttonBox, SIGNAL(rejected()), dialog, SLOT(reject()));
  dialog->exec();
  if (dialog->result())
  {
    QSqlQuery query;
    query.exec("SELECT COUNT(`Index`) FROM `" + route_name_ +
               "` WHERE `Index` = " + spinBox->cleanText() + ";");
    query.next();
    if (!query.value(0).toInt())
    {
      QMessageBox::critical(this, tr("Delete Place"), tr("Invalid Index"));
    }
    else
    {
      query.exec("DELETE FROM `" + route_name_ + "` WHERE `Index` = " +
                 spinBox->cleanText() + ";");

      // verify if the route is not empty
      query.exec("SELECT COUNT(`Index`) FROM `" + route_name_ + "`;");
      query.next();
      if (query.value(0).toInt())
      {
        // update indexes values after a place removal
        query.exec("SELECT `Index` FROM `" + route_name_ +
                   "` WHERE `Index` > " + spinBox->cleanText() + ";");
        query.next();
        for (int i = spinBox->cleanText().toInt(); i < spinBox->maximum(); i++)
        {
          QSqlQuery query2;
          query2.exec(QString("UPDATE `routes`.`" + route_name_ +
                              "` SET `Index`=%1 WHERE `Index`=%2;")
                          .arg(i)
                          .arg(query.value(0).toInt()));
          query.next();
        }
      }
      updateDialog();
    }
  }
  delete dialog;
}

void RouteManagerDialog::on_routeList_comboBox_currentIndexChanged(int index)
{
  route_name_ = ui_->routeList_comboBox->currentText();
  updateDialog();
}

void RouteManagerDialog::tableViewRowChanged(const QModelIndex& index)
{
  QSqlQuery query;
  query.exec(QString("SELECT * FROM `" + route_name_ + "` WHERE `Index`= %1 ;")
                 .arg(index.row()));
  query.next();
  ui_->index_spinBox->blockSignals(true);
  ui_->index_spinBox->setValue(query.value(0).toInt());
  ui_->index_spinBox->blockSignals(false);
  ui_->name_lineEdit->setText(query.value(1).toString());
  ui_->poseX_lineEdit->setText(query.value(2).toString());
  ui_->poseY_lineEdit->setText(query.value(3).toString());
  ui_->orientation_lineEdit->setText(query.value(4).toString());

  place_pose_.pose.pose.position.x = query.value(2).toDouble();
  place_pose_.pose.pose.position.y = query.value(3).toDouble();
  place_pose_.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(query.value(3).toDouble());
  ros_node_->initial_pose_pub.publish(place_pose_);
}

void RouteManagerDialog::updateButtonsState()
{
  if ((!ui_->name_lineEdit->text().isEmpty()) &&
      (!ui_->poseX_lineEdit->text().isEmpty()) &&
      (!ui_->poseY_lineEdit->text().isEmpty()) &&
      (!ui_->orientation_lineEdit->text().isEmpty()))
  {
    ui_->clearPlace_pushButton->setEnabled(true);
    ui_->addPlace_pushButton->setEnabled(true);
  }
  else
  {
    ui_->clearPlace_pushButton->setEnabled(false);
    ui_->addPlace_pushButton->setEnabled(false);
  }
}

void RouteManagerDialog::robotLocalized()
{
  ui_->poseX_lineEdit->setText(
      QString::number(ros_node_->custom_initial_pose.pose.pose.position.x));
  ui_->poseY_lineEdit->setText(
      QString::number(ros_node_->custom_initial_pose.pose.pose.position.y));
  ui_->orientation_lineEdit->setText(QString::number(
      tf::getYaw(ros_node_->custom_initial_pose.pose.pose.orientation)));
}

void RouteManagerDialog::updateComboBoxRoutesList()
{
  ui_->routeList_comboBox->blockSignals(true);
  ui_->routeList_comboBox->clear();

  QSqlQuery query;
  if (query.exec("SHOW TABLES FROM `routes`;"))
  {
    while (query.next())
    {
      if (query.value(0).toString() != "temp_route")
      {
        ui_->routeList_comboBox->addItem(query.value(0).toString());
      }
    }
    if (!ui_->routeList_comboBox->count())
    {
      QMessageBox::information(this, tr("Horus Patrol information"),
                               tr("There are no registered routes.<br>Please, "
                                  "create a route first."));
      ui_->routeManager_tabWidget->setCurrentIndex(0);
      ui_->routeList_comboBox->blockSignals(false);
      return;
    }
  }
  else
  {
    QMessageBox::critical(
        this, tr("Horus Patrol error"),
        tr("Something went wrong when attempting to access the database."));
  }

  on_routeList_comboBox_currentIndexChanged(0);
  ui_->routeList_comboBox->blockSignals(false);
}

void RouteManagerDialog::updateDialog()
{
  QSqlQuery query;
  query.exec("SELECT COUNT(0) FROM `" + route_name_ + "`;");
  query.next();

  if (!query.value(0).toInt())
  {
    ui_->name_lineEdit->setText("Initial Pose");
    ui_->addPlace_pushButton->setText("Add Initial Pose");
    ui_->placeInfo_groupBox->setTitle("Initial Pose Information");
    if (!ui_->routeManager_tabWidget->currentIndex())
      ui_->deleteRoute_pushButton->setEnabled(false);
    else
      ui_->deleteRoute_pushButton->setEnabled(true);
  }
  else
  {
    ui_->name_lineEdit->clear();
    ui_->addPlace_pushButton->setText("A&dd Place");
    ui_->placeInfo_groupBox->setTitle("Place Information");
    ui_->deleteRoute_pushButton->setEnabled(true);
  }

  if (query.value(0).toInt() < 2)
    ui_->save_pushButton->setEnabled(false);
  else
    ui_->save_pushButton->setEnabled(true);

  if (query.value(0).toInt() < 3)
    ui_->deleteIndex_pushButton->setEnabled(false);
  else
    ui_->deleteIndex_pushButton->setEnabled(true);

  ui_->poseX_lineEdit->clear();
  ui_->poseY_lineEdit->clear();
  ui_->orientation_lineEdit->clear();

  ui_->index_spinBox->blockSignals(true);
  ui_->index_spinBox->setMaximum(query.value(0).toInt());
  ui_->index_spinBox->setValue(query.value(0).toInt());
  ui_->index_spinBox->blockSignals(false);

  model_->setTable("`" + route_name_ + "`");
  model_->select();
  ui_->route_tableView->setModel(model_);
  ui_->route_tableView->setVisible(false);
  ui_->route_tableView->resizeColumnsToContents();
  ui_->route_tableView->setFixedWidth(
      ui_->route_tableView->horizontalHeader()->length());
  ui_->routeManager_tabWidget->setFixedWidth(
      ui_->route_tableView->horizontalHeader()->length());
  ui_->route_tableView->setVisible(true);
}
}
