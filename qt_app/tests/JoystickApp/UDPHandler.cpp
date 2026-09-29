#include "UDPHandler.h"

UDPHandler::UDPHandler(QObject *parent) : QObject(parent)
{
    receiverSocket.bind(QHostAddress::AnyIPv4, receivePort);
    connect(&receiverSocket, &QUdpSocket::readyRead, this, &UDPHandler::processPendingDatagrams);
}

/*void UDPHandler::sendJoystickData(float lx, float ly, float rx, float ry)
{
    QString message = QString("%1 %2 %3 %4").arg(lx).arg(ly).arg(rx).arg(ry);
    QByteArray datagram = message.toUtf8();
    senderSocket.writeDatagram(datagram, serverAddress, sendPort);
}*/

void UDPHandler::sendJoystickData(float lx, float ly, float rx, float ry, int8_t grip)
{
    QByteArray datagram;
    QDataStream stream(&datagram, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian); // Ensure consistent byte order

    // Header byte
    quint8 header = 0xAB;
    stream << header;

    // Append raw float values
    stream.writeRawData(reinterpret_cast<const char*>(&lx), sizeof(float));
    stream.writeRawData(reinterpret_cast<const char*>(&ly), sizeof(float));
    stream.writeRawData(reinterpret_cast<const char*>(&rx), sizeof(float));
    stream.writeRawData(reinterpret_cast<const char*>(&ry), sizeof(float));
    stream.writeRawData(reinterpret_cast<const char*>(&grip), sizeof(int8_t));

    senderSocket.writeDatagram(datagram, serverAddress, sendPort);
}


/*void UDPHandler::processPendingDatagrams()
{
    while (receiverSocket.hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(receiverSocket.pendingDatagramSize());
        receiverSocket.readDatagram(datagram.data(), datagram.size());

        QString receivedMessage = QString::fromUtf8(datagram);
        emit dataReceived(receivedMessage);
    }
}*/

void UDPHandler::processPendingDatagrams()
{


    while (receiverSocket.hasPendingDatagrams()) {
        QByteArray datagram;

        datagram.resize(receiverSocket.pendingDatagramSize());
        receiverSocket.readDatagram(datagram.data(), datagram.size());

        QDataStream in(&datagram, QIODevice::ReadOnly);
        in.setByteOrder(QDataStream::LittleEndian);
        quint8 header;
        in >> header;
        if (header != 0xAB) {
            qWarning() << "Invalid header byte. Discarding packet.";
            return;
        }

        float x, y, ang;

        in.readRawData(reinterpret_cast<char*>(&x), sizeof(float));
        in.readRawData(reinterpret_cast<char*>(&y), sizeof(float));
        in.readRawData(reinterpret_cast<char*>(&ang), sizeof(float));

        QString receivedMessage = QString("%1 %2 %3")
                                      .arg(QString::number(x, 'f', 4))
                                      .arg(QString::number(y, 'f', 4))
                                      .arg(QString::number(ang, 'f', 4));
        emit dataReceived(receivedMessage);
    }
}

