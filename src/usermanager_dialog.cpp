#include "include/horus_patrol/usermanager_dialog.hpp"

namespace HorusPatrol
{

UserManagerDialog::UserManagerDialog(QWidget* parent, bool first_admin)
    : QDialog(parent), god_(false), ui_(new Ui::UserManagerDialog)
{
  ui_->setupUi(this);
  if (first_admin)
  {
    ui_->manageUser_tabWidget->setCurrentIndex(0);
    ui_->manageUser_tabWidget->setTabEnabled(1, false);
    ui_->adminLogin_widget->setVisible(false);
    ui_->userManagerAction_pushButton->setEnabled(false);
    ui_->createProfile_comboBox->setCurrentIndex(1);
    ui_->createProfile_comboBox->setEnabled(false);
    ui_->deleteUser_pushButton->setVisible(false);
    ui_->userManagerAction_pushButton->setText("Add User");
    this->setFixedSize(380, 320);
  }
  else
  {
    this->setFixedSize(380, 270);
    ui_->manageUser_tabWidget->setVisible(false);
    ui_->deleteUser_pushButton->setVisible(false);
  }
  QObject::connect(ui_->exit_pushButton, SIGNAL(clicked()), this,
                   SLOT(reject()));
  QObject::connect(ui_->createName_lineEdit, SIGNAL(textChanged(QString)), this,
                   SLOT(checkInfoFill()));
  QObject::connect(ui_->createSurname_lineEdit, SIGNAL(textChanged(QString)),
                   this, SLOT(checkInfoFill()));
  QObject::connect(ui_->createUsername_lineEdit, SIGNAL(textChanged(QString)),
                   this, SLOT(checkInfoFill()));
  QObject::connect(ui_->createPassword_lineEdit, SIGNAL(textChanged(QString)),
                   this, SLOT(checkInfoFill()));
  QObject::connect(ui_->createConfirmPassword_lineEdit,
                   SIGNAL(textChanged(QString)), this, SLOT(checkInfoFill()));

  QObject::connect(ui_->editName_lineEdit, SIGNAL(textChanged(QString)), this,
                   SLOT(checkInfoFill()));
  QObject::connect(ui_->editSurname_lineEdit, SIGNAL(textChanged(QString)),
                   this, SLOT(checkInfoFill()));
  QObject::connect(ui_->editPassword_lineEdit, SIGNAL(textChanged(QString)),
                   this, SLOT(checkInfoFill()));
  QObject::connect(ui_->editConfirmPassword_lineEdit,
                   SIGNAL(textChanged(QString)), this, SLOT(checkInfoFill()));
  QObject::connect(ui_->editProfile_comboBox,
                   SIGNAL(currentTextChanged(QString)), this,
                   SLOT(checkInfoFill()));
}

UserManagerDialog::~UserManagerDialog() { delete ui_; }

void UserManagerDialog::on_userManagerAction_pushButton_clicked()
{
  // verifying admin account
  if (!ui_->userManagerAction_pushButton->text().compare("Login"))
  {
    QSqlQuery query;
    if (query.exec(
            "select user_profile from users_info where username=aes_encrypt('" +
            ui_->user_lineEdit->text() +
            "','rgoulart_horus_patrol') and user_password=aes_encrypt('" +
            ui_->password_lineEdit->text() + "','rgoulart_horus_patrol')"))
    {
      if (query.next())
      {
        if (!query.value(0).toString().compare("Administrator") ||
            (!query.value(0).toString().compare("God")))
        {
          ui_->manageUser_tabWidget->setCurrentIndex(0);
          ui_->manageUser_tabWidget->setVisible(true);
          ui_->adminLogin_widget->setVisible(false);
          ui_->userManagerAction_pushButton->setEnabled(false);
          ui_->userManagerAction_pushButton->setText("Add User");
          this->setFixedSize(380, 320);

          if (!query.value(0).toString().compare("Administrator"))
          {
            HorusLogger* logger =
                new HorusLogger(this, "PatrolDetections.txt",
                                "Username: " + ui_->user_lineEdit->text() +
                                    " accessed users' manager.");
            logger->start();
          }
          else
            god_ = true;
        }
        // not administrator
        else
        {
          QMessageBox::critical(
              this, tr("Horus Patrol error"),
              tr("This username is not an admistrator.<br>Username must have "
                 "administrator privileges."));

          ui_->user_lineEdit->clear();
          ui_->password_lineEdit->clear();
          ui_->user_lineEdit->setFocus();
        }
      }
      // invalid login
      else
      {
        QMessageBox::critical(
            this, tr("Horus Patrol error"),
            tr("Invalid username or password.<br>Please try again."));

        ui_->user_lineEdit->clear();
        ui_->password_lineEdit->clear();
        ui_->user_lineEdit->setFocus();

        HorusLogger* logger = new HorusLogger(
            this, "PatrolDetections.txt",
            "Failed attempt to login in the user manager as username: " +
                ui_->user_lineEdit->text() + ".");
        logger->start();
      }
    }
  }
  // creating user tab
  else if (!ui_->userManagerAction_pushButton->text().compare("Add User"))
  {
    if (ui_->createPassword_lineEdit->text().compare(
            ui_->createConfirmPassword_lineEdit->text()))
    {
      QMessageBox::critical(
          this, tr("Horus Patrol error"),
          tr("Passwords do not match.<br>Please, try again."));

      ui_->createConfirmPassword_lineEdit->clear();
      ui_->createPassword_lineEdit->clear();
      ui_->createPassword_lineEdit->setFocus();
    }
    else
    {
      QSqlQuery query;
      if (query.exec("insert into users_info values ('" +
                     ui_->createName_lineEdit->text() + "','" +
                     ui_->createSurname_lineEdit->text() + "',"
                                                           "'" +
                     ui_->createProfile_comboBox->currentText() +
                     "',aes_encrypt('" + ui_->createUsername_lineEdit->text() +
                     "',"
                     "'rgoulart_horus_patrol'), aes_encrypt('" +
                     ui_->createPassword_lineEdit->text() +
                     "','rgoulart_horus_patrol'));"))
      {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(
            this, tr("Horus Patrol question"),
            tr("User Created.<br>Do you want to create another user?"),
            QMessageBox::Yes | QMessageBox::No);

        if (reply == QMessageBox::Yes)
        {
          ui_->createUsername_lineEdit->clear();
          ui_->createPassword_lineEdit->clear();
          ui_->createConfirmPassword_lineEdit->clear();
          ui_->createProfile_comboBox->setCurrentIndex(0);
          ui_->createSurname_lineEdit->clear();
          ui_->createName_lineEdit->clear();
          ui_->createName_lineEdit->setFocus();

          // if it was the first user being created
          ui_->createProfile_comboBox->setEnabled(true);
          ui_->manageUser_tabWidget->setTabEnabled(1, true);
        }
        else
        {
          this->accept();
        }
      }
      else
      {
        QMessageBox::critical(
            this, tr("Horus Patrol error"),
            tr("Username already exist.<br>Please, choose another username."));
      }
    }
  }
  // edit user tab (btn == save)
  else
  {
    if (ui_->editPassword_lineEdit->text().compare(
            ui_->editConfirmPassword_lineEdit->text()))
    {
      QMessageBox::critical(
          this, tr("Horus Patrol error"),
          tr("Passwords do not match.<br>Please, try again."));

      ui_->editConfirmPassword_lineEdit->clear();
      ui_->editPassword_lineEdit->clear();
      ui_->editPassword_lineEdit->setFocus();
    }
    //verify if user is attempting to remove the last administrator from the system
    else if ( (ui_->editProfile_comboBox->currentText().compare(
                 "Administrator")) &&
             verifyLastAdmin() )
    {
      QMessageBox::information(
          this, tr("Horus Patrol information"),
          QString("User " + ui_->username_comboBox->currentText() +
                  " must have an Administrator profile.<br>At least one user "
                  "must have "
                  "Administrator profile in the system."));
    }
    else
    {
      QMessageBox::StandardButton reply;
      reply = QMessageBox::question(
          this, tr("Horus Patrol question"),
          tr("Are you sure you want to edit this user profile?"),
          QMessageBox::Yes | QMessageBox::No);

      if (reply == QMessageBox::Yes)
      {
        QSqlQuery query;
        if (query.exec("update `users`.`users_info` set `user_name`='" +
                       ui_->editName_lineEdit->text() + "',"
                                                        " `user_surname`='" +
                       ui_->editSurname_lineEdit->text() + "',"
                                                           " `user_profile`='" +
                       ui_->editProfile_comboBox->currentText() +
                       "',"
                       "`user_password`=aes_encrypt('" +
                       ui_->editPassword_lineEdit->text() +
                       "','rgoulart_horus_patrol')"
                       " where `username`=aes_encrypt('" +
                       ui_->username_comboBox->currentText() +
                       "','rgoulart_horus_patrol');"))
        {
          QMessageBox::information(this, tr("Horus Patrol information"),
                                   tr("User Saved."));
        }
        else
        {
          QMessageBox::critical(
              this, tr("Horus Patrol error"),
              tr("Something went wrong!<br>Could not save user."));
        }
      }
    }
  }
}

void UserManagerDialog::checkInfoFill()
{
  if (((!ui_->manageUser_tabWidget->currentIndex()) &&
       ((!ui_->createName_lineEdit->text().isEmpty()) &&
        (!ui_->createSurname_lineEdit->text().isEmpty()) &&
        (!ui_->createUsername_lineEdit->text().isEmpty()) &&
        (!ui_->createPassword_lineEdit->text().isEmpty()) &&
        (!ui_->createConfirmPassword_lineEdit->text().isEmpty()))) ||
      ((ui_->manageUser_tabWidget->currentIndex()) &&
       ((!ui_->editName_lineEdit->text().isEmpty()) &&
        (!ui_->editSurname_lineEdit->text().isEmpty()) &&
        (!ui_->editPassword_lineEdit->text().isEmpty()) &&
        (!ui_->editConfirmPassword_lineEdit->text().isEmpty()))))
  {
    ui_->userManagerAction_pushButton->setEnabled(true);
  }
  else
  {
    ui_->userManagerAction_pushButton->setEnabled(false);
  }
}

void UserManagerDialog::on_manageUser_tabWidget_currentChanged(int index)
{
  checkInfoFill();
  if (!index)
  {
    ui_->deleteUser_pushButton->setVisible(false);
    ui_->userManagerAction_pushButton->setText("Add User");
  }
  else
  {
    ui_->deleteUser_pushButton->setVisible(true);
    ui_->userManagerAction_pushButton->setText("Save");
    updateUsernameComboBox();
  }
}

void UserManagerDialog::updateUsernameComboBox()
{
  QSqlQuery query;
  if (query.exec("select aes_decrypt(username,'rgoulart_horus_patrol') from "
                 "users_info;"))
  {
    ui_->username_comboBox->clear();
    while (query.next())
    {
      if ((god_) || (query.value(0).toString().compare("Horus")))
      {
        ui_->username_comboBox->addItem(query.value(0).toString());
      }
    }
  }
}

void UserManagerDialog::on_username_comboBox_currentIndexChanged(int index)
{
  QSqlQuery query;
  if (query.exec("select user_name, user_surname, user_profile, "
                 "aes_decrypt(user_password,'rgoulart_horus_patrol') from "
                 "users_info where username = aes_encrypt('" +
                 ui_->username_comboBox->currentText() +
                 "','rgoulart_horus_patrol');"))
  {
    if (query.next())
    {
      if (ui_->username_comboBox->currentText() == "Horus")
      {
        ui_->editName_lineEdit->setEnabled(false);
        ui_->editSurname_lineEdit->setEnabled(false);
        ui_->editProfile_comboBox->addItem(query.value(2).toString());
        ui_->editProfile_comboBox->setCurrentIndex(2);
        ui_->editProfile_comboBox->setEnabled(false);
        ui_->deleteUser_pushButton->setEnabled(false);
      }
      else
      {
        ui_->editName_lineEdit->setEnabled(true);
        ui_->editSurname_lineEdit->setEnabled(true);
        ui_->editProfile_comboBox->removeItem(2);
        ui_->editProfile_comboBox->setCurrentIndex(
            ui_->editProfile_comboBox->findText(query.value(2).toString()));
        ui_->editProfile_comboBox->setEnabled(true);
        ui_->deleteUser_pushButton->setEnabled(true);
      }
      ui_->editName_lineEdit->setText(query.value(0).toString());
      ui_->editSurname_lineEdit->setText(query.value(1).toString());
      ui_->editPassword_lineEdit->setText(query.value(3).toString());
      ui_->editConfirmPassword_lineEdit->setText(query.value(3).toString());
      ui_->userManagerAction_pushButton->setEnabled(false);
    }
  }
}

void UserManagerDialog::on_deleteUser_pushButton_clicked()
{
  if (!verifyLastAdmin())
  {
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(
        this, tr("Horus Patrol question"),
        QString("Are you sure you want to delete username = " +
                ui_->username_comboBox->currentText() + "?"),
        QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes)
    {
      QSqlQuery query;
      if (query.exec("delete from `users`.`users_info` where "
                     "`username`=aes_encrypt('" +
                     ui_->username_comboBox->currentText() +
                     "','rgoulart_horus_patrol');"))
      {
        QMessageBox::information(this, tr("Horus Patrol information"),
                                 tr("User Deleted."));
      }
      else
      {
        QMessageBox::critical(
            this, tr("Horus Patrol error"),
            tr("Something went wrong!<br>Could not delete user."));
      }

      updateUsernameComboBox();
    }
  }
  else
  {
    QMessageBox::information(
        this, tr("Horus Patrol information"),
        QString("User " + ui_->username_comboBox->currentText() +
                " cannot be deleted.<br>At least one user must have "
                "Administrator profile."));
  }
}

bool UserManagerDialog::verifyLastAdmin()
{
  QSqlQuery query;
  short int admin_num = 0;

  // verify if there is only 1 admin
  query.exec("SELECT user_profile from users_info;");
  while (query.next())
  {
    if (!query.value(0).toString().compare("Administrator"))
      admin_num++;
  }
  // verify if current user is admin
  query.exec("SELECT user_profile FROM users_info WHERE "
             "`username`=AES_ENCRYPT('" +
             ui_->username_comboBox->currentText() +
             "','rgoulart_horus_patrol');");
  query.next();

  if ((admin_num == 1) && (!query.value(0).toString().compare("Administrator")))
  {
    return true;
  }
  else
  {
    return false;
  }
}
}
