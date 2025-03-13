//#include "hydroPeleng.h"
//#include <QDataStream>
//#include <QFile>


//PelengProtocol::PelengProtocol(QString portName, int baudRate, QObject *parent)
//{
//    h_port.setBaudRate(baudRate);
//    h_port.setPortName(portName);
//    h_port.open(QIODevice::ReadWrite);

//    QTimer *timer = new QTimer(this);
//    connect(&h_port, &QSerialPort::readyRead, this, &PelengProtocol::readData);
//    connect(&h_port, &QSerialPort::readyRead, this, &PelengProtocol::readyReadForTimer);
//    connect(timer, &QTimer::timeout, this, &PelengProtocol::timeoutSlot);
//    timer->start(3000);

//}

//void PelengProtocol::readyReadForTimer() {
//    time.restart();
//}

//void PelengProtocol::readData() {
//    h_buffer.append(h_port.readAll());
//    parseBuffer();
//}

//void PelengProtocol::parseBuffer() {
//    static int count;
//    if ( h_buffer.size() <= 2 ) {
//        return;
//    }

////    if (correctChecksum(h_buffer.mid(1))) {
//        //qDebug()<<++count;
//        uint8_t msg;
//        auto tmp = h_buffer.mid(0, 1);
//        QDataStream stream(&tmp, QIODevice::ReadOnly);
//        stream.setByteOrder(QDataStream::LittleEndian);
//        stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
//        stream >> msg;
//        emit newMessageDetected(msg);
//        data = msg;
//        h_buffer.remove(0, 1);

////    else {
////        h_buffer.remove(0, index+1);
////    }

//    return;
//}



#include "hydroPeleng.h"
#include <QDataStream>
#include <QFile>
#include <QDebug>

PelengProtocol::PelengProtocol(QString portName, int baudRate, QObject *parent)
{
    h_port.setBaudRate(baudRate);
    h_port.setPortName(portName);
//    h_port.open(QIODevice::ReadWrite);
    if (h_port.open(QIODevice::ReadWrite))
        qDebug() << "good connect Pelengator";
    else {
        qDebug() << "bed connect Pelengator";}

    QTimer *timer = new QTimer(this);
    connect(&h_port, &QSerialPort::readyRead, this, &PelengProtocol::readData);
    qDebug() << "portName" << portName;
    qDebug() << "baudRate" << baudRate;
    qDebug() << "connect pelengator";
}


void PelengProtocol::readData() {
    h_buffer.append(h_port.readAll());
    parseBuffer();
}

void PelengProtocol::parseBuffer() {
    if (h_buffer.isEmpty()) {
        return;
    }
    // Чтение первого байта из буфера
    uint8_t msg = static_cast<uint8_t>(h_buffer.at(0));

    // Эмитируем сигнал с полученным байтом
//    emit newMessageDetected(msg);
    data = msg;

    // Удаляем обработанный байт из буфера
    h_buffer.remove(0, 1);
}
