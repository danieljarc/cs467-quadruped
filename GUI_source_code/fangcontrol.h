#ifndef FANGCONTROL_H
#define FANGCONTROL_H

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
//#include <QtCore>
#include <QtGui>
#include <QtBluetooth>
#include <QDialog>
#include <QSerialPort>
#include <QMainWindow>
#include <QMessageBox>
#include <QButtonGroup>
//#include <QBluetoothDeviceDiscoveryAgent>
//#include <QBluetoothSocket>

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

    void on_connect_button_clicked();

    void on_mode_button_clicked();

    void on_l1_button_pressed();

    void on_r1_button_pressed();

    void on_reset_button_pressed();

    void on_sit_button_pressed();

    void on_stand_button_pressed();


    //void on_listWidget_itemClicked(QListWidgetItem *item);

    //void deviceDiscovered(const QBluetoothDeviceInfo &device);

private:
    enum State {NO_CONNECTION, CONNECTED, ST_MODE, SR_MODE, WG1_MODE, WG2_MODE};
    State current_state;
    Ui::FangControl *ui;
    QSerialPort *arduino;
    QString arduino_port_name;
    static const quint16 arduino_uno_vendor_id;
    static const quint16 arduino_uno_product_id;
    bool arduino_available;

    //QBluetoothDeviceDiscoveryAgent *agent = new QBluetoothDeviceDiscoveryAgent;
    //QBluetoothSocket *socket;
    QString btAddr;

//    QButtonGroup* walk_group;
//    QButtonGroup* stationary_group;
};

#endif // FANGCONTROL_H
