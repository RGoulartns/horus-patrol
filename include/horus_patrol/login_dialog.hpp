#ifndef LOGIN_DIALOG_HPP
#define LOGIN_DIALOG_HPP

#include <QSqlDatabase>
#include <QSqlQuery>
#include <QMessageBox>
#include "ui_login_dialog.h"
#include "horus_logger.hpp"
#include "usermanager_dialog.hpp"

namespace HorusPatrol
{

class LoginDialog : public QDialog
{
  Q_OBJECT

public:
  explicit LoginDialog(QWidget* parent = 0);
  ~LoginDialog();
  bool getUserProfile();

private Q_SLOTS:
  void on_login_pushButton_clicked();
  void on_manageUsers_pushButton_clicked();

private:
  bool admin_;
  Ui::LoginDialog* ui_;
  UserManagerDialog* user_manager_dialog_;
  QSqlDatabase horusdb_;
};
}

#endif // LOGIN_DIALOG_HPP
