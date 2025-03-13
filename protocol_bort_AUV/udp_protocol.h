#ifndef UDP_PROTOCOL_H
#define UDP_PROTOCOL_H

#include <QObject>
#include <QDebug>
#include <QTimer>
#include <QUdpSocket>
#include "kx_pult/qpiconfig.h"


template <class ReceiveStruct, class SendStruct>

class UdpProtocol : public QObject {
public:
    ReceiveStruct rec_data; //структура для приема данных
    SendStruct send_data; //структура для отпарвки данных
    explicit UdpProtocol (const QString & config = "protocols.conf",
                          const QString & name = "pult", QObject *parent = 0){
        QPIConfig conf(config, QIODevice::ReadOnly);
        QPIConfig::Entry & e(conf.getValue(name));
        //прежде чем создать соединение по сети между пультом и
        //ТНПА необходимо задать ip адреса и порты для отправки и приема данных
        //эти значения считываем из config-файла, имя которого передается
        //в конструктор класса (принцип аналогичен config-файлам kx_pult'a)
        m_ip_sender = e.getValue("sender.ip").value(); //ip верхнего уровня
        m_ip_receiver = e.getValue("receiver.ip").value(); //ip ROV_Model
        m_port_sender = e.getValue("sender.port", 0); //порт верхнего уровня
        m_port_receiver = e.getValue("receiver.port", 0); //порт ROV_Model
        m_frequency = e.getValue("receiver.frequency", 20); //частота передачи данных
        //если в config-файле не указана частота передачи данных, то
        //по умолчанию в качестве этого значения принимается 20
        //(второй параметр в функции getValue())

        qInfo() << "Sender (ROV) ip:" << m_ip_sender << "port:" << m_port_sender;
        qInfo() << "Receiver (Pult) ip:" << m_ip_receiver << "port:" << m_port_receiver;
        qInfo() << "Frequency:" << m_frequency;

        receiveSocket = new QUdpSocket(); //создаем сокет на прием данных
        bindState_=receiveSocket->bind(m_ip_receiver, m_port_receiver);
        qInfo()<<"bind receive socket: "<<bindState_;
        qInfo()<<"socket error: "<<receiveSocket->errorString();
        //слушаем данные приходящие на ip-адрес ip_receiver,
        //на порт port_receiver
        sendSocket = new QUdpSocket(); //создаем сокет на передачу данных
        qInfo()<<"Sends to ip:" << m_ip_sender <<"port:" << m_port_sender;
        //соединим сигнал таймера с функцией sendData, отсылающей данные
        //connect(timer, SIGNAL(timeout()), SLOT(sendData()));
        //соединим сигнал сокета receiveSocket о том, что данные готовы для
        //чтения с функцией receiveData()
        //connect(receiveSocket, SIGNAL(readyRead()),SLOT(receiveData()));

        //send_data.mode=Ruchnoi;
    }
    ~UdpProtocol(){
        delete receiveSocket;
        delete sendSocket;
    }
    float getFrequency(){
        return m_frequency;
    }

    QUdpSocket *getReceiveSocket(){
        return receiveSocket;
    }
    QHostAddress ip_receiver(){return m_ip_receiver;}
    QHostAddress ip_sender(){return m_ip_sender;}
    int port_receiver(){return m_port_receiver;}
    int port_sender(){return m_port_sender;}
    float frequency(){return m_frequency;}
    bool bindState(){return bindState_;}
    void setCheckState(bool state) {checkState = state;}
private:
    bool bindState_=false;
    bool checkState = true;
    QUdpSocket *receiveSocket; //UDP-socket для приема данных
    QUdpSocket *sendSocket; //UDP-socket для отпарвки данных
    QHostAddress m_ip_receiver, m_ip_sender;
    int m_port_receiver, m_port_sender; //номера портов приема и передачи
    float m_frequency; //частота обмена данными ТНПА с ПУ
    //функция вычисления контрольной суммы
//    uint checksum_i(const void * data, int size){ // function for checksum (uint)
//        uint c = 0;
//        for (int i = 0; i < size; ++i)
//            c += ((const uchar*)data)[i];
//        return ~(c + 1);
//    }

    uint16_t getChecksum16b(const void * vector) {
        uint16_t len = sizeof(vector)-2;
        uint16_t crc = 0xFFFF;
        uint8_t i;
        uint8_t g = 0;

        while (len--) {
            crc ^= ((const uchar*)vector)[g++] << 8;

            for (i = 0; i < 8; i++) crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
        }
        return crc;
    }

    //проверка верна ли контрольная сумма
    bool validate(const ReceiveStruct & data) {
        if (!checkState) {return true;}
//        else return (data.checksum == checksum_i(&data, sizeof(data) - 4));
       else return (data.checksum == getChecksum16b(&data));
    }

    void aboutSend() {
//        send_data.checksum = checksum_i(&send_data, sizeof(send_data) - 4);
        send_data.checksum = getChecksum16b(&send_data);
    } //функция, которая вычисляет контроьную сумму и записывает
    //ее в переменную checksum структуры отправки

public slots:
    void sendData(){
//        qDebug() << "send_data" << sizeof(send_data);
        aboutSend();//считаем контрольную сумму и записываем ее в переменную checksum структуры send_data (типа SendStruct)
        //Отсылаем структуру send_data на ip_sender на порт port_sender
        sendSocket->writeDatagram((char *)&send_data, sizeof(send_data),m_ip_sender, m_port_sender);
        //qDebug() <<"send to ip:" << m_ip_sender << "port:" << m_port_sender;   06/04/24
        //qDebug() <<"reseive to ip: " << m_ip_receiver << "port: " << m_port_receiver;  06/04/24
        //qDebug() << "sizeof(send_data): " <<(sizeof(send_data));  06/04/24
    } //метод для отправки данных, который можно вызывать по сигналу таймера
    void receiveData(){
        while(receiveSocket->hasPendingDatagrams()) {
            ReceiveStruct rec;// создаем локальную переменную для приема данных до проверки
            receiveSocket->readDatagram((char *)&rec, sizeof(rec)); //считываем данные
//            qDebug() << "sizeof(rec): "<< sizeof(rec);
//            if (!validate(rec)) {
//                //Функция validate возвращает true - если контрольная сумма верна
//                //и возвращает false, если это не так
//                //в этой части функции контрольная сумма не верна
//                qDebug() << "Checksum validate" << validate(rec);
//                continue;
//                //оператор continue выполняет пропуск оставшейся части кода
//                //      тела цикла и переходит к следующей итерации цикла
//            }
//            else {qDebug() << "Checksum true" << validate(rec); }
            rec_data = rec;//Если контрльная сумма верна, то записываем
            //принятые данные в структуру rec_data
        }
    } //слот для приема данных, который соединим с
    //сигналом readyRead() сокета receiveSocket, который создается
    //при наличии принятых пакетов
    int sendMessage(QByteArray ba){
        return (sendSocket->writeDatagram(ba,m_ip_sender,m_port_sender));
        }
    };

    class MetaUdpProtocol {
    public:
        MetaUdpProtocol(){
            timer = new QTimer();
        }
        virtual QHostAddress ip_receiver(){
            return m_ip_receiver;
        }
        virtual QHostAddress ip_sender(){
            return m_ip_sender;
        }
        virtual int port_receiver(){
            return m_port_receiver;
        }
        virtual int port_sender(){
            return m_port_sender;
        }
        virtual QString errorReceiverPort(){
            return m_errorReceiverPort;
        }
        virtual QString errorSenderPort(){
            return m_errorSenderPort;
        }
        virtual int frequency(){
            return m_frequency;
        }
        QTimer *timer; //таймер для отправки данных с определенной частотой
        virtual void set_ip_receiver   (QHostAddress ip) {m_ip_receiver = ip;}
        virtual void set_ip_sender     (QHostAddress ip) {m_ip_sender = ip;  }
        virtual void set_port_receiver (int port)        {m_port_receiver=port;}
        virtual void set_port_sender   (int port)        {m_port_sender = port;}
    private:
        QString m_errorReceiverPort, m_errorSenderPort;
        QHostAddress m_ip_receiver, m_ip_sender; //receiver - наше приложение
                                             //sender - модель ТНПА
        int m_port_receiver, m_port_sender; //номера портов приема и передачи
        float m_frequency; //частота обмена данными ТНПА с ПУ
};

#endif // UDP_PROTOCOL_H
