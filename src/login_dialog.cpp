#include "include/horus_patrol/login_dialog.hpp"

namespace HorusPatrol
{

LoginDialog::LoginDialog(QWidget* parent)
    : QDialog(parent), ui_(new Ui::LoginDialog), admin_(false)
{
  HorusLogger* logger =
      new HorusLogger(this, "PatrolDetections.txt", "Horus Patrol executed.");
  logger->start();

  horusdb_ = QSqlDatabase::addDatabase("QMYSQL");
  horusdb_.setHostName("localhost");
  horusdb_.setUserName("root");
  horusdb_.setPassword("123456");

  if (!horusdb_.open())
  {
    QMessageBox::critical(this, tr("Horus Patrol error"),
                          tr("Couldn't connect to the database."));
    QApplication::quit();
    this->close();
  }
  else
  {
    QSqlQuery query;
    query.exec("SELECT SCHEMA_NAME FROM INFORMATION_SCHEMA.SCHEMATA WHERE "
               "SCHEMA_NAME = 'users'");
    if (!query.next())
    {
      query.exec(
          "CREATE DATABASE IF NOT EXISTS users DEFAULT CHARACTER SET latin1;");
      query.exec("use users;");
      query.exec("CREATE TABLE IF NOT EXISTS users_info (user_name "
                 "varchar(30), user_surname "
                 "varchar(30),user_profile varchar(20), username varchar(20), "
                 "user_password varchar(20), PRIMARY KEY (username));");

      QMessageBox::information(this, tr("Horus Patrol Information"),
                               tr("A new 'Users' database was created.<br>"
                                  "You must create at least one Administrator "
                                  "to access the system."));
      user_manager_dialog_ = new UserManagerDialog(this, true);
      user_manager_dialog_->exec();
      delete user_manager_dialog_;
    }
    else
    {
      // check if there is at least one administrator registered
      horusdb_.setDatabaseName("users");
      query.exec("use users;");
      bool admin_exist = false;
      query.exec("SELECT user_profile FROM users_info");
      while (query.next())
      {
        if (query.value(0).toString() == "Administrator")
        {
          admin_exist = true;
          break;
        }
      }
      if (!admin_exist)
      {
        QMessageBox::information(
            this, tr("Horus Patrol Information"),
            tr("Unable to find an user with Administrator's profile.<br>"
               "You must create at least one Administrator to access the "
               "system."));
        user_manager_dialog_ = new UserManagerDialog(this, true);
        user_manager_dialog_->exec();
        delete user_manager_dialog_;
      }
    }

    ui_->setupUi(this);
    this->setFixedSize(380, 200);

    QObject::connect(ui_->quit_pushButton, SIGNAL(clicked()), this,
                     SLOT(reject()));
  }
}

LoginDialog::~LoginDialog() { delete ui_; }

bool LoginDialog::getUserProfile() { return admin_; }

void LoginDialog::on_manageUsers_pushButton_clicked()
{
  user_manager_dialog_ = new UserManagerDialog(this, false);
  user_manager_dialog_->exec();
  delete user_manager_dialog_;
}

void LoginDialog::on_login_pushButton_clicked()
{
  QSqlQuery query;
  if (query.exec("SELECT * FROM users_info WHERE username=aes_encrypt('" +
                 ui_->user_lineEdit->text() +
                 "','rgoulart_horus_patrol')"
                 " AND user_password=aes_encrypt('" +
                 ui_->password_lineEdit->text() + "','rgoulart_horus_patrol')"))
  {
    if (query.next())
    {
      HorusLogger* logger = new HorusLogger(
          this, "PatrolDetections.txt",
          "User: " + ui_->user_lineEdit->text() + " logged-in.");
      logger->start();

      if ((query.value(2).toString() == "Administrator") ||
          (query.value(2).toString() == "God"))
      {
        admin_ = true;
      }

      this->accept();
    }
    // TODO should Horus be shown in the log?
    else
    {
      HorusLogger* logger =
          new HorusLogger(this, "PatrolDetections.txt",
                          "Failed attempt to log in as username: " +
                              ui_->user_lineEdit->text() + ".");
      logger->start();

      QMessageBox::warning(
          this, tr("Horus Patrol warning"),
          tr("Invalid username or password.<br>Please try again."));

      ui_->password_lineEdit->clear();
      ui_->user_lineEdit->clear();
      ui_->user_lineEdit->setFocus();
    }
  }
}
}
