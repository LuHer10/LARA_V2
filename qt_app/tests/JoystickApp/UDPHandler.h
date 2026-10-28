#ifndef UDPHANDLER_H
#define UDPHANDLER_H

#include <QObject>
#include <QUdpSocket>

class UDPHandler : public QObject
{
    Q_OBJECT
public:
    explicit UDPHandler(QObject *parent = nullptr);

    /*Q_INVOKABLE void sendJoystickData(float lx, float ly,
                                      float rx, float ry,
                                      int8_t grip);
    */

    Q_INVOKABLE void sendJoystickData(float lx, float ly,
                                      float rx, float ry,
                                      int8_t d1, int8_t d2, int8_t d3,
                                      int8_t mode);

signals:
    void dataReceived(QString data);

private slots:
    void processPendingDatagrams();

private:
    QUdpSocket senderSocket;
    QUdpSocket receiverSocket;

    QHostAddress serverAddress = QHostAddress("192.168.0.105");  // Replace with your server IP
    quint16 sendPort = 40000;
    quint16 receivePort = 40001;
};

#endif // UDPHANDLER_H
