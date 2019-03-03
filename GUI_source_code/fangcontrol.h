#ifndef FANGCONTROL_H
#define FANGCONTROL_H

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <QStateMachine>
#include <QEventTransition>
#include <QPropertyAnimation>
#include <QtGui>
#include <QtBluetooth>
#include <QDialog>
#include <QSerialPort>
#include <QMainWindow>
#include <QMessageBox>
#include <QButtonGroup>
#include <QBluetoothSocket>
#include <QListWidget>

namespace Ui {
class FangControl;
}

class FangControl : public QMainWindow
{
    Q_OBJECT

public:
    explicit FangControl(QWidget *parent = nullptr);
    ~FangControl();

private slots:
    void on_right_button_pressed();

    void on_down_button_pressed();

    void on_left_button_pressed();

    void on_up_button_pressed();

    void on_connect_button_clicked(bool connect_enable);

    void on_mode_button_clicked();

    void on_l1_button_pressed();

    void on_r1_button_pressed();

    void on_center_button_pressed();

    void on_sit_button_pressed();

    void on_stand_button_pressed();

    void connected();

    void onSocketErrorOccurred(QBluetoothSocket::SocketError);

    void readSocket();

signals:
    void connected(const QString &name);
    void disconnected();
    void socketErrorOccurred(const QString &errorString);
    void messageReceived(const QString &sender, const QString &message);

private:
    QStateMachine *state_m;
    QState *NO_CONNECTION;
    QState *CONNECTED;
    QState *ST_MODE;
    QState *SR_MODE;
    QState *WG1_MODE;

    Ui::FangControl *ui;
    QSerialPort *arduino;
    QString arduino_port_name;
    bool connect_enable;
    static const quint16 arduino_uno_vendor_id;
    static const quint16 arduino_uno_product_id;
    bool arduino_available;

    QBluetoothSocket *socket = nullptr; // Sets up RFCOMM socket similar to TCP socket
  //  QString btAddr = "98:D3:11:FC:1B:B0"; // Hexidecimal address of bluetooth device
      QString btAddr = "20:18:02:05:28:79"; // Hexidecimal address of bluetooth device
};

#endif // FANGCONTROL_H
