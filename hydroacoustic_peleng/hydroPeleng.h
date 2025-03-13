//#ifndef PELENGPROTOCOL_H
//#define PELENGPROTOCOL_H

//#include <QObject>
//#include <QSerialPort>
//#include <QDebug>
//#include <QTimer>
//#include <QTime>
//;
////класс протокола
//#pragma pack(push,1)

//class PelengProtocol : public QObject
//{
//    Q_OBJECT
//public:
//    explicit PelengProtocol(QString portName, int baudRate = 115200, QObject *parent = 0);
//    uint8_t data;//выходная структура

//signals:
//    void newMessageDetected(uint8_t msg);

//public slots:
//    void readData(); //слот, который будет вызываться в ответ на readyRead
//    void readyReadForTimer();
//    void timeoutSlot();
//protected:

//    void parseBuffer();
//    QByteArray h_buffer;
//    QSerialPort h_port; //объект COM-порта
//    int baudRate = 115200; //бодрейт
//    QTime time;
//    QTimer *timer;
//};

//#endif // PELENGPROTOCOL_H

#ifndef HYDROPELENG_H
#define HYDROPELENG_H

#include <QObject>
#include <QSerialPort>
#include <QTimer>
#include <QByteArray>

class PelengProtocol : public QObject
{
    Q_OBJECT

public:
    explicit PelengProtocol(QString portName, int baudRate = 115200, QObject *parent = nullptr);

signals:
//    void newMessageDetected(uint8_t message); // Сигнал для передачи принятого байта

private slots:
    void readData();          // Слот для чтения данных из порта

public:
    void parseBuffer();       // Метод для разбора буфера

    int baudRate = 115200;
    QSerialPort h_port;       // Объект для работы с последовательным портом
    QByteArray h_buffer;      // Буфер для хранения принятых данных
    uint8_t data = 0;             // Переменная для хранения принятого байта
};

#endif // HYDROPELENG_H
