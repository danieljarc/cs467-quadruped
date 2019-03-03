#include "fangcontrol.h"
#include "ui_fangcontrol.h"

/**********************************************
 * Member Function - Constructor
 *
 * Description: Sets up UI and state machine by
 * creating all states and transitions in which
 * the GUI will cycle through. Also creates the
 * bluetooth socket.
 *********************************************/
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

    QEventTransition *t_start = new QEventTransition(ui->connect_button, QEvent::MouseButtonPress);
    NO_CONNECTION->assignProperty(ui->status_label, "text", "<b style=\"color:red;\">Not Connected</b>");
    t_start->setTargetState(CONNECTED);
    NO_CONNECTION->addTransition(t_start);

    QEventTransition *t_st_mode_init = new QEventTransition(ui->mode_button, QEvent::MouseButtonPress);
    CONNECTED->assignProperty(ui->mode_button, "enabled", true);
    t_st_mode_init->setTargetState(ST_MODE);
    CONNECTED->addTransition(t_st_mode_init);

    QEventTransition *t_sr_mode = new QEventTransition(ui->mode_button, QEvent::MouseButtonPress);
    ST_MODE->assignProperty(ui->translate_m, "text", "<b>Stationary - Translate</b>");
    ST_MODE->assignProperty(ui->rotate_m, "text", "Stationary - Rotate");
    ST_MODE->assignProperty(ui->walk_g1_m, "text", "Walk");
    ST_MODE->assignProperty(ui->l1_button, "text", "+Z");
    ST_MODE->assignProperty(ui->r1_button, "text", "-Z");
    t_sr_mode->setTargetState(SR_MODE);
    ST_MODE->addTransition(t_sr_mode);

    QEventTransition *t_wg1_mode = new QEventTransition(ui->mode_button, QEvent::MouseButtonPress);
    SR_MODE->assignProperty(ui->translate_m, "text", "Stationary - Translate");
    SR_MODE->assignProperty(ui->rotate_m, "text", "<b>Stationary - Rotate</b>");
    SR_MODE->assignProperty(ui->walk_g1_m, "text", "Walk");
    SR_MODE->assignProperty(ui->l1_button, "text", "+YAW");
    SR_MODE->assignProperty(ui->r1_button, "text", "-YAW");
    t_wg1_mode->setTargetState(WG1_MODE);
    SR_MODE->addTransition(t_wg1_mode);

    QEventTransition *t_st_mode = new QEventTransition(ui->mode_button, QEvent::MouseButtonPress);
    WG1_MODE->assignProperty(ui->translate_m, "text", "Stationary - Translate");
    WG1_MODE->assignProperty(ui->rotate_m, "text", "Stationary - Rotate");
    WG1_MODE->assignProperty(ui->walk_g1_m, "text", "<b>Walk</b>");
    WG1_MODE->assignProperty(ui->l1_button, "text", "<--");
    WG1_MODE->assignProperty(ui->r1_button, "text", "-->");
    t_st_mode->setTargetState(ST_MODE);
    WG1_MODE->addTransition(t_st_mode);

    state_m->addState(NO_CONNECTION);
    state_m->addState(CONNECTED);
    state_m->addState(ST_MODE);
    state_m->addState(SR_MODE);
    state_m->addState(WG1_MODE);
    state_m->setInitialState(NO_CONNECTION);
    state_m->start();

    socket = new QBluetoothSocket(QBluetoothServiceInfo::RfcommProtocol);
}

/**********************************************
 * Member Function - Destructor
 *
 * Description: Disconnects bluetooth socket
 * and collapses the UI
 *********************************************/
FangControl::~FangControl()
{
    socket->disconnectFromService();
    delete ui;
}

/**********************************************
 * Member Function - Right Button Slot
 *
 * Description: Sends different byte to signal
 * different right-button behavior depending
 * on mode
 *********************************************/
void FangControl::on_right_button_pressed()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("d");
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("l");
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("h");
    else
        QMessageBox::information(this, "Right Button Release", "Error, Invalid State!");
}

/**********************************************
 * Member Function - Backward Button Slot
 *
 * Description: Sends different byte to signal
 * different backward-button behavior depending
 * on mode
 *********************************************/
void FangControl::on_down_button_pressed()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("s");
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("i");
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("g");
    else
        QMessageBox::information(this, "Down Button Press", "Error, Invalid State!");
}

/**********************************************
 * Member Function - Left Button Slot
 *
 * Description: Sends different byte to signal
 * different left-button behavior depending
 * on mode
 *********************************************/
void FangControl::on_left_button_pressed()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("a");
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("j");
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("f");
    else
        QMessageBox::information(this, "Left Button Pressed", "Error, Invalid State!");
}

/**********************************************
 * Member Function - Forward Button Slot
 *
 * Description: Sends different byte to signal
 * different forward-button behavior depending
 * on mode
 *********************************************/
void FangControl::on_up_button_pressed()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("w");
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("k");
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("t");
    else
        QMessageBox::information(this, "Up Button Press", "Error, Invalid State!");
}

/**********************************************
 * Member Function - Top-Left Button Slot
 *
 * Description: Sends different byte to signal
 * different top-left-button behavior depending
 * on mode
 *
 * Z+, Yaw+, <-- Buttons
 *********************************************/
void FangControl::on_l1_button_pressed()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("q"); // Z+
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("u"); // Yaw-
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("r"); // <--
    else
        QMessageBox::information(this, "L1 Button Release", "Error, Invalid State!");
}

/**********************************************
 * Member Function - Top-Right Button Slot
 *
 * Description: Sends different byte to signal
 * different top-right-button behavior depending
 * on mode
 *
 * Z-, Yaw-, --> Buttons
 *********************************************/
void FangControl::on_r1_button_pressed()
{
    if(state_m->configuration().contains(ST_MODE))
        socket->write("e"); // Z-
    else if(state_m->configuration().contains(SR_MODE))
        socket->write("o"); // Yaw-
    else if(state_m->configuration().contains(WG1_MODE))
        socket->write("y"); // -->
    else
        QMessageBox::information(this, "L1 Button Release", "Error, Invalid State!");
}

/**********************************************
 * Member Function - Center Button Slot
 *
 * Description: Sends different byte to signal
 * different center-button behavior depending
 * on mode
 *********************************************/
void FangControl::on_center_button_pressed()
{
    if(!(state_m->configuration().contains(NO_CONNECTION)))
        socket->write("/");
}

/**********************************************
 * Member Function - Sit Button Slot
 *
 * Description: Sends different byte to signal
 * different sit-button behavior depending
 * on mode
 *********************************************/
void FangControl::on_sit_button_pressed()
{
    if(!(state_m->configuration().contains(NO_CONNECTION)))
        socket->write(",");
}

/**********************************************
 * Member Function - Stand Button Slot
 *
 * Description: Sends different byte to signal
 * different stand-button behavior depending
 * on mode
 *********************************************/
void FangControl::on_stand_button_pressed()
{
    if(!(state_m->configuration().contains(NO_CONNECTION)))
        socket->write(".");
}

/**********************************************
 * Member Function - Connect Button Slot
 *
 * Description: Establishes bluetooth connection
 * via socket generated from the constructor.
 * The bluetooth address is hard-coded in
 * the header
 *********************************************/
void FangControl::on_connect_button_clicked(bool connect_enable)
{
    if((state_m->configuration().contains(CONNECTED)) && !connect_enable) {
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
        ui->status_label->setText("<b style=\"color:green;\">Connected</b>");
        qDebug("Connected.");
        ui->connect_button->setEnabled(false);
    }
}

/**********************************************
 * Member Function - Mode Button Slot
 *
 * Description: Enables the other buttons when
 * Mode states are cyclable
 *********************************************/
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
        ui->center_button->setEnabled(true);
        ui->sit_button->setEnabled(true);
        ui->stand_button->setEnabled(true);
    }
}

/**********************************************
 * Member Function - Socket Error Slot
 *
 * Description: Generates an error signal when
 * a socket error occurs
 *********************************************/
void FangControl::onSocketErrorOccurred(QBluetoothSocket::SocketError error)
{
    if (error == QBluetoothSocket::NoSocketError)
        return;

    QMetaEnum metaEnum = QMetaEnum::fromType<QBluetoothSocket::SocketError>();
    QString errorString = socket->peerName() + QLatin1Char(' ')
            + metaEnum.valueToKey(error) + QLatin1String(" occurred");

    emit socketErrorOccurred(errorString);
}

/**********************************************
 * Member Function - Connected Slot
 *
 * Description: Generates a connected signal
 * when the socket connection has been
 * established
 *********************************************/
void FangControl::connected()
{
    emit connected(socket->peerName());
}

/**********************************************
 * Member Function - readSocket Slot
 *
 * Description: Generates a socket read signal
 * for transfers received
 *********************************************/
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
