#ifndef CMOVE_DIALOG_H
#define CMOVE_DIALOG_H

#include <QDialog>
#include "ui_cmove_dialog.h"
#include "ros_node.hpp"


namespace HorusPatrol
{

class CustomMoveDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CustomMoveDialog(ROSNode *node, QWidget *parent = 0);
    ~CustomMoveDialog();


public Q_SLOTS:
    void on_buttonBox_accepted();
    void on_shapes_comboBox_currentIndexChanged(int index);


private:
    Ui::CustomMoveDialog *ui;
    ROSNode* ros_node_;
};
}

#endif // CMOVE_DIALOG_H
