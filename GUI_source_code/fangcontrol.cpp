#include "fangcontrol.h"
#include "ui_fangcontrol.h"

FangControl::FangControl(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::FangControl)
{
    ui->setupUi(this);

    state_m = new QStateMachine(this);

    NO_CONNECTION = new QState();
    CONNECTED = new QState();
    ST_MODE = new QState();
    SR_MODE = new QState();
    WG1_MODE = new QState();
    WG2_MODE = new QState();

    QEventTransition *t_start = new QEventTransition(ui->connect_button, QEvent::MouseButtonPress);
    NO_CONNECTION->assignProperty(ui->status_label, "text", "<b style=\"color:red;\">Not Connected</b>");
    t_start->setTargetState(CONNECTED);
    NO_CONNECTION->addTransition(t_start);

    QEventTransition *t_st_mode_init = new QEventTransition(ui->mode_button, QEvent::MouseButtonPress);
    CONNECTED->assignProperty(ui->status_label, "text", "<b style=\"color:green;\">Connected</b>");
    CONNECTED->assignProperty(ui->mode_button, "enabled", true);
    t_st_mode_init->setTargetState(ST_MODE);
    CONNECTED->addTransition(t_st_mode_init);

    QEventTransition *t_sr_mode = new QEventTransition(ui->mode_button, QEvent::MouseButtonPress);
    //ST_MODE->assignProperty(ui->keypad_frame, "enabled", true);
    ST_MODE->assignProperty(ui->translate_m, "text", "<b>Stationary - Translate</b>");
    ST_MODE->assignProperty(ui->rotate_m, "text", "Stationary - Rotate");
    ST_MODE->assignProperty(ui->walk_g1_m, "text", "Walk - Gait 1");
    ST_MODE->assignProperty(ui->walk_g2_m, "text", "Walk - Gait 2");
    ST_MODE->assignProperty(ui->l1_button, "text", "+Z");
    ST_MODE->assignProperty(ui->r1_button, "text", "-Z");
    t_sr_mode->setTargetState(SR_MODE);
    ST_MODE->addTransition(t_sr_mode);

    QEventTransition *t_wg1_mode = new QEventTransition(ui->mode_button, QEvent::MouseButtonPress);
    SR_MODE->assignProperty(ui->translate_m, "text", "Stationary - Translate");
    SR_MODE->assignProperty(ui->rotate_m, "text", "<b>Stationary - Rotate</b>");
    SR_MODE->assignProperty(ui->walk_g1_m, "text", "Walk - Gait 1");
    SR_MODE->assignProperty(ui->walk_g2_m, "text", "Walk - Gait 2");
    SR_MODE->assignProperty(ui->l1_button, "text", "+YAW");
    SR_MODE->assignProperty(ui->r1_button, "text", "-YAW");
    t_wg1_mode->setTargetState(WG1_MODE);
    SR_MODE->addTransition(t_wg1_mode);

    QEventTransition *t_wg2_mode = new QEventTransition(ui->mode_button, QEvent::MouseButtonPress);
    WG1_MODE->assignProperty(ui->translate_m, "text", "Stationary - Translate");
    WG1_MODE->assignProperty(ui->rotate_m, "text", "Stationary - Rotate");
    WG1_MODE->assignProperty(ui->walk_g1_m, "text", "<b>Walk - Gait 1</b>");
    WG1_MODE->assignProperty(ui->walk_g2_m, "text", "Walk - Gait 2");
    t_wg2_mode->setTargetState(WG2_MODE);
    WG1_MODE->addTransition(t_wg2_mode);

    QEventTransition *t_st_mode = new QEventTransition(ui->mode_button, QEvent::MouseButtonPress);
    WG2_MODE->assignProperty(ui->translate_m, "text", "Stationary - Translate");
    WG2_MODE->assignProperty(ui->rotate_m, "text", "Stationary - Rotate");
    WG2_MODE->assignProperty(ui->walk_g1_m, "text", "Walk - Gait 1");
    WG2_MODE->assignProperty(ui->walk_g2_m, "text", "<b>Walk - Gait 2</b>");
    t_st_mode->setTargetState(ST_MODE);
    WG2_MODE->addTransition(t_st_mode);

    state_m->addState(NO_CONNECTION);
    state_m->addState(CONNECTED);
    state_m->addState(ST_MODE);
    state_m->addState(SR_MODE);
    state_m->addState(WG1_MODE);
    state_m->addState(WG2_MODE);
    state_m->setInitialState(NO_CONNECTION);
    state_m->start();

    //connect(discoveryAgent, SIGNAL(deviceDiscovered(QBluetoothDeviceInfo)), this, SLOT(deviceDiscovered(QBluetoothDeviceInfo)));
    //discoveryAgent->start();
    socket = new QBluetoothSocket(QBluetoothServiceInfo::RfcommProtocol);
}

FangControl::~FangControl()
{
    delete ui;
    //connect(agent, SIGNAL(deviceDiscovered(QBluetoothDeviceInfo)), this, SLOT(deviceDiscovered(QBluetoothDeviceInfo)));
    //agent->start();
}

void FangControl::on_right_button_pressed()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("st_right_button_p");
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("sr_right_button_p");
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("wg1_right_button_p");
    else if(state_m->configuration().contains(WG2_MODE))
        socket->write("wg2_right_button_p");
    else
        QMessageBox::information(this, "Right Button Release", "Error, Invalid State!");
}

void FangControl::on_right_button_released()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("st_right_button_r");
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("sr_right_button_r");
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("wg1_right_button_r");
    else if(state_m->configuration().contains(WG2_MODE))
        socket->write("wg2_right_button_r");
    else
        QMessageBox::information(this, "Right Button Release", "Error, Invalid State!");
}

void FangControl::on_down_button_pressed()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("st_down_button_p");
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("sr_down_button_p");
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("wg1_down_button_p");
    else if(state_m->configuration().contains(WG2_MODE))
        socket->write("wg2_down_button_p");
    else
        QMessageBox::information(this, "Down Button Press", "Error, Invalid State!");
}


void FangControl::on_down_button_released()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("st_down_button_r");
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("sr_down_button_r");
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("wg1_down_button_r");
    else if(state_m->configuration().contains(WG2_MODE))
        socket->write("wg2_down_button_r");
    else
        QMessageBox::information(this, "Down Button Release", "Error, Invalid State!");
}

void FangControl::on_left_button_pressed()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("st_left_button_p");
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("sr_left_button_p");
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("wg1_left_button_p");
    else if(state_m->configuration().contains(WG2_MODE))
        socket->write("wg2_left_button_p");
    else
        QMessageBox::information(this, "Left Button Pressed", "Error, Invalid State!");
}

void FangControl::on_left_button_released()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("st_left_button_r");
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("sr_left_button_r");
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("wg1_left_button_r");
    else if(state_m->configuration().contains(WG2_MODE))
        socket->write("wg2_left_button_r");
    else
        QMessageBox::information(this, "Left Button Release", "Error, Invalid State!");
}

void FangControl::on_up_button_pressed()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("st_up_button_p");
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("sr_up_button_p");
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("wg1_up_button_p");
    else if(state_m->configuration().contains(WG2_MODE))
        socket->write("wg2_up_button_p");
    else
        QMessageBox::information(this, "Up Button Press", "Error, Invalid State!");
}

void FangControl::on_up_button_released()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("st_up_button_r");
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("sr_up_button_r");
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("wg1_up_button_r");
    else if(state_m->configuration().contains(WG2_MODE))
        socket->write("wg2_up_button_r");
    else
        QMessageBox::information(this, "Up Button Release", "Error, Invalid State!");
}

void FangControl::on_l1_button_pressed()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("+Z_p");
    else
        socket->write("+YAW_p");
}

void FangControl::on_l1_button_released()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("+Z_r");
    else
        socket->write("+YAW_r");
}

void FangControl::on_r1_button_pressed()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("-Z_p");
    else
        socket->write("-YAW_p");
}

void FangControl::on_r1_button_released()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("-Z_r");
    else
        socket->write("-YAW_r");
}

void FangControl::on_reset_button_pressed()
{
    if(!(state_m->configuration().contains(NO_CONNECTION)))
//        QMessageBox::information(this, "Reset Button", "Reset Button Pressed!");
//    else
        socket->write("rst");
}

void FangControl::on_sit_button_pressed()
{
    if(!(state_m->configuration().contains(NO_CONNECTION)))
    //    QMessageBox::information(this, "Sit Button", "Sit Button Pressed!");
    //else
        socket->write("sit");
}

void FangControl::on_stand_button_pressed()
{
    if(!(state_m->configuration().contains(NO_CONNECTION)))
//        QMessageBox::information(this, "Stand Button", "Stand Button Pressed!");
//    else
        socket->write("stand");
}

void FangControl::on_connect_button_clicked(bool connect_enable)
{
    if((state_m->configuration().contains(CONNECTED)) && !connect_enable) {
        //QMessageBox::information(this, "Connect Button", "Connected!");
        connect_enable = true;
        qDebug("Connecting...");
        static const QString serviceUuid("e8e10f95-1a70-4b27-9ccf-02010264e9c"); //Lifted from bluetooth chat example
        qDebug() << "service id is: " << serviceUuid;
        //socket->connectToService(QBluetoothAddress(btAddr), QBluetoothUuid(serviceUuid), QIODevice::ReadWrite);
        socket->connectToService(QBluetoothAddress(btAddr), QBluetoothUuid(QBluetoothUuid::SerialPort));
        qDebug("test...");
        connect(socket, &QBluetoothSocket::readyRead, this, &FangControl::readSocket);
        connect(socket, &QBluetoothSocket::connected, this, QOverload<>::of(&FangControl::connected));
        connect(socket, &QBluetoothSocket::disconnected, this, &FangControl::disconnected);
        connect(socket, QOverload<QBluetoothSocket::SocketError>::of(&QBluetoothSocket::error),
                this, &FangControl::onSocketErrorOccurred);
        qDebug("Connected.");
        ui->connect_button->setEnabled(false);
    }
}

void FangControl::on_mode_button_clicked()
{

    if(!state_m->configuration().contains(CONNECTED)) {
        ui->keypad_groupBox->setEnabled(true);
        ui->l1_button->setEnabled(true);
        ui->r1_button->setEnabled(true);
        ui->left_button->setEnabled(true);
        ui->right_button->setEnabled(true);
        ui->up_button->setEnabled(true);
        ui->down_button->setEnabled(true);
        ui->reset_button->setEnabled(true);
        ui->sit_button->setEnabled(true);
        ui->stand_button->setEnabled(true);
    }
}

//void FangControl::deviceDiscovered(const QBluetoothDeviceInfo &device)
//{
//    ui->devicesList->addItem(device.address().toString()); // Gets all devices and puts them in list widget
//}

//void FangControl::on_devicesList_itemClicked(QListWidgetItem *item)
//{
//    btAddr = item->text();  // On double click selects item from list object and sets btAddr
//}

void FangControl::onSocketErrorOccurred(QBluetoothSocket::SocketError error)
{
    if (error == QBluetoothSocket::NoSocketError)
        return;

    QMetaEnum metaEnum = QMetaEnum::fromType<QBluetoothSocket::SocketError>();
    QString errorString = socket->peerName() + QLatin1Char(' ')
            + metaEnum.valueToKey(error) + QLatin1String(" occurred");

    emit socketErrorOccurred(errorString);
}

void FangControl::connected()
{
    emit connected(socket->peerName());
}

void FangControl::readSocket()
{
    if (!socket)
        return;

    while (socket->canReadLine()) {
        QByteArray line = socket->readLine();
        emit messageReceived(socket->peerName(),
                             QString::fromUtf8(line.constData(), line.length()));
    }
}
