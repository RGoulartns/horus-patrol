#ifndef USERMANAGER_DIALOG_HPP
#define USERMANAGER_DIALOG_HPP

#include <QSqlQuery>
#include <QMessageBox>
#include "horus_logger.hpp"
#include "ui_usermanager_dialog.h"

namespace HorusPatrol
{

class UserManagerDialog : public QDialog
{
    Q_OBJECT

public:
    explicit UserManagerDialog(QWidget *parent = 0, bool first_admin = false);
    ~UserManagerDialog();

private Q_SLOTS:
    void checkInfoFill();
    void updateUsernameComboBox();
    bool verifyLastAdmin();

    void on_deleteUser_pushButton_clicked();
    void on_userManagerAction_pushButton_clicked();
    void on_manageUser_tabWidget_currentChanged(int index);
    void on_username_comboBox_currentIndexChanged(int index);

private:
    Ui::UserManagerDialog *ui_;
    bool god_;
};
}

#endif
