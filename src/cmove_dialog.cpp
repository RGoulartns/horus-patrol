#include "../include/horus_patrol/cmove_dialog.hpp"

namespace HorusPatrol
{

CustomMoveDialog::CustomMoveDialog(ROSNode *node, QWidget *parent) :
    QDialog(parent),
    ros_node_(node),
    ui(new Ui::CustomMoveDialog)
{
    ui->setupUi(this);
    ui->widget_2->setVisible(false);
    ui->repetition_spinBox->setEnabled(false);
    ui->extraturn_spinBox->setEnabled(false);
    ui->buttonBox->setEnabled(false);

    QObject::connect(ui->buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    QObject::connect(ui->buttonBox_2, SIGNAL(rejected()), this, SLOT(reject()));
    QObject::connect(ui->repetition_checkBox, SIGNAL(clicked(bool)), ui->repetition_spinBox, SLOT(setEnabled(bool)));
    QObject::connect(ui->extraturn_checkBox, SIGNAL(clicked(bool)), ui->extraturn_spinBox, SLOT(setEnabled(bool)));
}

CustomMoveDialog::~CustomMoveDialog()
{
    delete ui;
}

void CustomMoveDialog::on_shapes_comboBox_currentIndexChanged(int index)
{
    if(!index)
    {
        ui->widget_2->setVisible(false);
        ui->buttonBox->setEnabled(false);
    }
    else
    {
        ui->widget_2->setVisible(true);
        ui->buttonBox->setEnabled(true);
        if(index == 1) //rect
        {
            ui->doubleSpinBox_1->setPrefix(QApplication::translate("CustomMoveDialog", "Side 1:  ", 0));
            ui->doubleSpinBox_2->setVisible(true);
            ui->doubleSpinBox_2->setPrefix(QApplication::translate("CustomMoveDialog", "Side 2:  ", 0));
            ui->extraturn_widget->setVisible(true);
        }
        else if(index == 2) //Square
        {
            ui->doubleSpinBox_1->setPrefix(QApplication::translate("CustomMoveDialog", "Side:  ", 0));
            ui->doubleSpinBox_2->setVisible(false);
            ui->extraturn_widget->setVisible(true);
        }
        else if(index == 3) //Circle
        {
            ui->doubleSpinBox_1->setPrefix(QApplication::translate("CustomMoveDialog", "Radius:  ", 0));
            ui->doubleSpinBox_2->setVisible(false);
            ui->extraturn_widget->setVisible(false);
        }
    }
}

void CustomMoveDialog::on_buttonBox_accepted()
{
    ros_node_->selected_cmove = ui->shapes_comboBox->currentText();

    ros_node_->side1 = ui->doubleSpinBox_1->value();
    if(ui->repetition_checkBox->isChecked()) ros_node_->nlaps = ui->repetition_spinBox->value();
    else ros_node_->nlaps = 1;
    if(ui->extraturn_checkBox->isChecked()) ros_node_->vertices_rotation = ui->extraturn_spinBox->value();
    else ros_node_->vertices_rotation = 0;
    ros_node_->clock_rotation = ui->rotationdirection_comboBox->currentIndex();
    //if(ros_node_->clock_rotation) ros_node_->custommoveturn = 0.027;
    //else ros_node_->custommoveturn = -0.121;

    if(ui->shapes_comboBox->currentIndex() == 1) ros_node_->side2 = ui->doubleSpinBox_2->value();
    else if(ui->shapes_comboBox->currentIndex() == 2) ros_node_->side2 = ros_node_->side1;
    else if(ui->shapes_comboBox->currentIndex() == 3)
    {
        ros_node_->side1 = 0;
        ros_node_->side2 = 0;
        if(ui->repetition_checkBox->isChecked()) ros_node_->nlaps = 0;
        else ros_node_->nlaps = 0;
        ros_node_->vertices_rotation = 0;
        ros_node_->clock_rotation = ui->rotationdirection_comboBox->currentIndex();
    }
}
}
