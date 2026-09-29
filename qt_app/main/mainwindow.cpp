#include "mainwindow.h"
#include <QHBoxLayout>
#include <QWidget>
#include <QLabel>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    udpSender = new QUdpSocket(this);

    udpReceiver = new QUdpSocket(this);
    udpReceiver->bind(QHostAddress::AnyIPv4, clientPort);
    connect(udpReceiver, &QUdpSocket::readyRead, this, &MainWindow::onUdpDataReceived);

    sendTimer = new QTimer(this);
    connect(sendTimer, &QTimer::timeout, this, &MainWindow::sendJoystickData);
    sendTimer->start(100); // every 100ms

    QWidget *central = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(central);

    udpDataLabel = new QLabel("Waiting for UDP data...");
    mainLayout->addWidget(udpDataLabel);

    QHBoxLayout *joystickLayout = new QHBoxLayout();

    leftJoystick = new JoystickWidget(this);
    rightJoystick = new JoystickWidget(this);
    joystickLayout->addWidget(leftJoystick);
    joystickLayout->addWidget(rightJoystick);

    mainLayout->addLayout(joystickLayout);

    central->setLayout(mainLayout);
    setCentralWidget(central);

    connect(leftJoystick, &JoystickWidget::directionChanged, this, &MainWindow::onLeftJoystickMoved);
    connect(rightJoystick, &JoystickWidget::directionChanged, this, &MainWindow::onRightJoystickMoved);
}

MainWindow::~MainWindow()
{
    
}

void MainWindow::sendJoystickData()
{
    QByteArray data = QString("%1 %2 %3 %4")
                          .arg(leftX).arg(-leftY).arg(rightX).arg(-rightY).toUtf8();
    udpSender->writeDatagram(data, QHostAddress(serverIp), serverPort);
}

void MainWindow::onLeftJoystickMoved(float x, float y)
{
    leftX = x;
    leftY = y;
}

void MainWindow::onRightJoystickMoved(float x, float y)
{
    rightX = x;
    rightY = y;
}

void MainWindow::onUdpDataReceived()
{
    while (udpReceiver->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(udpReceiver->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        udpReceiver->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
        QString message = QString::fromUtf8(datagram);

        udpDataLabel->setText(QString("From %1:%2 → %3")
                                  .arg(sender.toString())
                                  .arg(senderPort)
                                  .arg(message));
    }
}
