#include "fangcontrol.h"
#include "ui_fangcontrol.h"

FangControl::FangControl(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::FangControl)
{
    current_state = NO_CONNECTION;
    ui->setupUi(this);

    //arduino = new QSerialPort;
    //qDebug() << "Number of available ports: " << QSerialPortInfo::availablePorts().length();

    //connect(agent, SIGNAL(deviceDiscovered(QBluetoothDeviceInfo)), this, SLOT(deviceDiscovered(QBluetoothDeviceInfo)));
    //agent->start();

//    QButtonGroup* walk_group = new QButtonGroup(parent);
//    QButtonGroup* stationary_group = new QButtonGroup(parent);
//    walk_group->addButton(ui->gait1_submode);
//    walk_group->addButton(ui->gait2_submode);
//    walk_group->addButton(ui->gait3_submode);

//    stationary_group->addButton(ui->tilt_submode);
//    stationary_group->addButton(ui->rotate_submode);
//    stationary_group->addButton(ui->ht_submode);
//    stationary_group->addButton(ui->vt_submode);
}

FangControl::~FangControl()
{
    delete ui;
    //connect(agent, SIGNAL(deviceDiscovered(QBluetoothDeviceInfo)), this, SLOT(deviceDiscovered(QBluetoothDeviceInfo)));
    //agent->start();
}

void FangControl::on_right_button_pressed()
{
    if(current_state != NO_CONNECTION)
        QMessageBox::information(this, "Right Button", "Right Button Pressed!");
}

void FangControl::on_down_button_pressed()
{
    if(current_state != NO_CONNECTION)
        QMessageBox::information(this, "Down Button", "Down Button Pressed!");
}

void FangControl::on_left_button_pressed()
{
    if(current_state != NO_CONNECTION)
        QMessageBox::information(this, "Left Button", "Left Button Pressed!");
}

void FangControl::on_up_button_pressed()
{
    if(current_state != NO_CONNECTION)
        QMessageBox::information(this, "Up Button", "Up Button Pressed!");
}

void FangControl::on_l1_button_pressed()
{
    if(current_state != NO_CONNECTION)
        QMessageBox::information(this, "L1 Button", "L1 Button Pressed!");
}

void FangControl::on_r1_button_pressed()
{
    if(current_state != NO_CONNECTION)
        QMessageBox::information(this, "R1 Button", "R1 Button Pressed!");
}

void FangControl::on_reset_button_pressed()
{
    if(current_state != NO_CONNECTION)
        QMessageBox::information(this, "Reset Button", "Reset Button Pressed!");
}

void FangControl::on_sit_button_pressed()
{
    if(current_state != NO_CONNECTION)
        QMessageBox::information(this, "Sit Button", "Sit Button Pressed!");
}

void FangControl::on_stand_button_pressed()
{
    if(current_state != NO_CONNECTION)
        QMessageBox::information(this, "Stand Button", "Stand Button Pressed!");
}

void FangControl::on_connect_button_clicked()
{
    if(current_state == NO_CONNECTION) {
        QMessageBox::information(this, "Connect Button", "Connected!");
        current_state = CONNECTED;
    }
//    static const QString serviceUuid; //TODO
//    socket = new QBluetoothSocket(QBluetoothServiceInfo::RfcommProtocol);

//    //TODO - Need bluetooth address and service uuid
//    socket->connectToService(QBluetoothAddress(btAddr), QBluetoothUuid(serviceUuid), QIODevice::ReadWrite);
}

void FangControl::on_mode_button_clicked()
{
    if(current_state != NO_CONNECTION) {
        if(ui->translate_m->isChecked()) {
            ui->translate_m->setChecked(0);
            //ui->stationary_mode->setChecked(0);
            ui->rotate_m->setChecked(1);
            //qDebug() << "test1";
        } else if(ui->rotate_m->isChecked()){
            //ui->stationary_mode->setChecked(0);
            ui->rotate_m->setChecked(0);
            ui->walk_g1_m->setChecked(1);
        } else if(ui->walk_g1_m->isChecked()){
            //ui->stationary_mode->setChecked(0);
            ui->walk_g1_m->setChecked(0);
            ui->walk_g2_m->setChecked(1);
        } else if(ui->walk_g2_m->isChecked()){
            //ui->stationary_mode->setChecked(0);
            ui->walk_g2_m->setChecked(0);
            ui->translate_m->setChecked(1);
        } else {
            ui->translate_m->setChecked(1);
        }
    }
}

//void FangControl::on_submode_button_clicked()
//{
//    if(ui->walk_mode->isChecked()) {
//        qDebug() << "test1";
//        //stationary_group->setExclusive(0);
//        qDebug() << "test2";
//        ui->tilt_submode->setChecked(0);
//        //stationary_group->setExclusive(1);
//        qDebug() << "test3";
//        ui->gait1_submode->setChecked(1);
//    } else {
//        //walk_group->setExclusive(0);
//        ui->gait1_submode->setChecked(0);
//        //walk_group->setExclusive(1);
//        ui->tilt_submode->setChecked(1);
//    }
//}

//void FangControl::deviceDiscovered(const QBluetoothDeviceInfo &device)
//{
//    ui->listWidget->addItem(device.address().toString());
//}
