#ifndef PC_PROTOCOL_H
#define PC_PROTOCOL_H

#include "udp_protocol.h"
#include "protocol.h"

//const QString ConfigFile = "protocols.conf";
//класс обмена АНПА- планировщик, который создается в АНПА
namespace ControlSystem {
class PC_Protocol: public QObject, public MetaUdpProtocol {
    Q_OBJECT
public:
explicit PC_Protocol(const QString & config = "protocols.conf",
                       const QString & name = "AUV", QObject *parent = 0){
    udpProtocol = new UdpProtocol <FromHighLevel, ToHighLevel> (config, name, parent);
    connect(timer,SIGNAL(timeout()),SLOT(sendData()));
    connect(udpProtocol->getReceiveSocket(),SIGNAL(readyRead()),SLOT(receiveData()));
    set_ip_receiver(udpProtocol->ip_receiver());
    set_ip_sender (udpProtocol->ip_sender());
    set_port_receiver(udpProtocol->port_receiver());
    set_port_sender (udpProtocol->port_sender());
}
signals:
    void dataReceived();
public slots:
    //запуск обмена
    void startExchange(){
        timer->start(1000/udpProtocol->getFrequency());
    }
    //остановить обмен
    void stopExhange(){
        timer->stop();
    }
    void sendData(){
 //       qDebug()<<"send data";
        udpProtocol->send_data = send_data;
        udpProtocol->sendData();
    }
    void receiveData(){
        udpProtocol->receiveData();
        rec_data = udpProtocol->rec_data;
//        qDebug() << "rec_data.size = " << rec_data.size();
        emit dataReceived();
    }
public:
    ToHighLevel send_data;
    FromHighLevel rec_data;
    UdpProtocol <FromHighLevel, ToHighLevel> *udpProtocol;

    bool bindState(){return udpProtocol->bindState();}
};
} //namespace ControlSystem

namespace Pult {
class PC_Protocol: public QObject, public MetaUdpProtocol {
    Q_OBJECT
public:
explicit PC_Protocol(const QString & config = "protocols.conf",
                       const QString & name = "pult", QObject *parent = 0)
{
    udpProtocol = new UdpProtocol <ToHighLevel, FromHighLevel> (config, name, parent);
    connect(timer,SIGNAL(timeout()),SLOT(sendData()));
    connect(udpProtocol->getReceiveSocket(),SIGNAL(readyRead()),SLOT(receiveData()));
    set_ip_receiver(udpProtocol->ip_receiver());
    set_ip_sender (udpProtocol->ip_sender());
    set_port_receiver(udpProtocol->port_receiver());
    set_port_sender (udpProtocol->port_sender());
    qDebug() << "rec_data.size =" << sizeof(rec_data);
    qDebug() << "send_data.size =" << sizeof(send_data);
}
signals:
    void dataReceived();
public slots:
    //запуск обмена
    void startExchange(){
        timer->start(1000/udpProtocol->getFrequency());
    }
    //остановить обмен
    void stopExhange(){
        timer->stop();
    }
    void sendData(){
 //       qDebug()<<"send data";
        udpProtocol->send_data = send_data;
        udpProtocol->sendData();
    }

    void receiveData(){
        udpProtocol->receiveData();
        rec_data = udpProtocol->rec_data;
//        qDebug() << "rec_data.size = " << rec_data.size();
        emit dataReceived();
//        static_cast<unsigned char>(rec_data.cSMode);
    }
public:
    ToHighLevel  rec_data;
    FromHighLevel send_data;
    UdpProtocol <ToHighLevel, FromHighLevel> *udpProtocol;

    bool bindState(){return udpProtocol->bindState();}
};
} //namespace Pult


#endif // PC_PROTOCOL_H
