#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "joystickwidget.h"
#include <QUdpSocket>
#include <QLabel>
#include <QTimer>

class JoystickWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onLeftJoystickMoved(float x, float y);
    void onRightJoystickMoved(float x, float y);
    void onUdpDataReceived();

private:
    QUdpSocket *udpSender;
    QString serverIp = "192.168.1.124";
    quint16 serverPort = 40000;

    QUdpSocket *udpReceiver;
    quint16 clientPort = 40001;

    JoystickWidget *leftJoystick;
    JoystickWidget *rightJoystick;
    QLabel *udpDataLabel;

    QTimer *sendTimer;

    float leftX = 0.0f;
    float leftY = 0.0f;
    float rightX = 0.0f;
    float rightY = 0.0f;

    void sendJoystickData();
};
#endif // MAINWINDOW_H
