#ifndef HYDROACOUSTICS_H
#define HYDROACOUSTICS_H

//#include <QObject>
#include <QSerialPort>
#include <QTimer>
#include "json_parser.h"
#include <string>


#pragma pack(push,1)

struct Channel
{
    quint8 txCh = 0;
    quint8 rxCh = 0;
};

struct ChannelRound
{
    quint8 txCh1 = 0;
    quint8 txCh2 = 0;
    quint8 txCh3 = 0;
    quint8 txCh4 = 0;
    quint8 rxCh = 0;
    quint8 Number = 0;
};

struct PUWVE //ответ от модема на включение командного режима
{
    unsigned char isPTMode = 0; //если 1, то пакетный режим включен
    unsigned char ptLocalAddress = 0; //адрес локального модема в пакетном режиме

};

struct PUWV0 // реакция устройства на поступивший от управляющей системы запрос
{
    unsigned char cmdID = 0; //Идентификатор обрабатываемой команды
    unsigned char errCode = 0; //Error code
};

struct PUWV4
{
    unsigned char txChID = 0;
    unsigned char rcCmdID = 0;
    int counterID1 = 0;
    int counterID2 = 0;
    int counterID3 = 0;
    int counterID4 = 0;
    int counter = 0;
    int counterAll = 0;
};

struct PUWV3
{
    unsigned char txChID = 0;
    unsigned char rcCmdID = 0;
    float propTime = 0;
    float MSR = 0;
    float Value = 0;
    float distanceID1 = 0;
    float distanceID2 = 0;
    float distanceID3 = 0;
    float distanceID4 = 0;
    float tempID1 = 0;
    float tempID2 = 0;
    float tempID3 = 0;
    float tempID4 = 0;
    float temp = 0;
    float distance = 0;
    int counterID1 = 0;
    int counterID2 = 0;
    int counterID3 = 0;
    int counterID4 = 0;
    int counter = 0;
    int counterAll = 0;
};

struct PUWV7
{
    float Pressure_mBar = 0;
    float Temperature_C = 0;
    float Depth_m = 0;
    float VCC_V = 0;
};


struct PUWVH // Передача пакета данных не увенчалась успехом.
{
    unsigned char target_ptAddress = 0; //Идентификатор обрабатываемой команды
    unsigned char maxTries = 0; //Предпринятое число попыток
    long dataPacket = 0; //Массив байт в HEX-формате с префиксом ‘0x’, например для строки ‘123’ 0x313233. Максимальный размер пакета 64 байта.

};


struct PUWVI // Пакет данных успешно передан.
{
    unsigned char target_ptAddress = 0; //	Адрес удаленного модема, 0 .. 254
    unsigned char maxTries = 0; //Предпринятое число попыток
    long dataPacket = 0; //Массив байт в HEX-формате с префиксом ‘0x’, например для строки ‘123’ 0x313233. Максимальный размер пакета 64 байта.

};

struct PUWVJ // Пакет данных успешно принят
{
    unsigned char sender_ptAddress = 0; //	Адрес удаленного модема, 0 .. 254
    long dataPacket = 0; //Массив байт в HEX-формате с префиксом ‘0x’, например для строки ‘123’ 0x313233. Максимальный размер пакета 64 байта.
};


struct uWave
{
    PUWVE puwve;
    PUWV0 puwv0;
    PUWVH puwvh;
    PUWV7 puwv7;
    PUWV4 puwv4;
    PUWV3 puwv3;
    PUWVI puwvi;
    PUWVJ puwvj;
    int counterACK = 0;
    int counterACK1 = 0;
    int counterACK2 = 0;
    int counterACK3 = 0;
    int counterACK4 = 0;
    QByteArray infoModem;
    uint distance_real = 0;
    uint warning = 0;
};


#pragma pack(pop)

class Hydroacoustics : public QObject
{
    Q_OBJECT
public:
    explicit Hydroacoustics(QString portName, int baudRate = 9600,
                            QObject *parent = nullptr);
    float salinity = 0.;
    uWave uwave;
    void settings();
    void parseBuffer();
    void readData();
    void parsePUWV0(QByteArray msg);
    void parsePUWVE(QByteArray msg);
    void parsePUWVH(QByteArray msg);
    void parsePUWVI(QByteArray msg);
    void parsePUWV7(QByteArray msg);
    void parsePUWV4(QByteArray msg);
    void parsePUWV3(QByteArray msg);
    void parsePUWVJ(QByteArray msg);

    void stopCounter();
    void clearAll();
    QTimer timerRound;
    QByteArray request_PUWVF(int idModem);//формирование команды включения пакетного режима (возможно не работает)
    QByteArray request_PUWVG(int idModem, int maxTries, int dataPacket); //формирование посылки, которую передаем на другой модем в пакетном режиме?
    //idModem - Адрес удаленного модема, maxTries - Максимальное число попыток,
    //dataPacket - массив байт в HEX-формате с префиксом ‘0x’, например для строки ‘123’ 0x313233
    QByteArray request_PUWV2(int idModem, int idChennel);
    QByteArray request_PUWV1(float STY = 0., int idModem = 1, int idChennel = 0, float gravityAcc = 9.8067);
    Channel chD;
    ChannelRound chR;


protected:
    QSerialPort ha;
    QByteArray ha_buffer;
    int crc (QByteArray msg);
    QByteArray crc_MSG (QByteArray msg);
    int crc_real(qint8 crc_in);
    int roundCounter = 1;

signals:
    void cmd1Received(); //
    void initCmd2Done();
    void initCmd1Done();
    void initStolbik();
    void initDone();
    void newMessageDetectedACKIdle(uWave uwave);
    void updateData(uWave uwave);
public slots:
    void onPaket(); //включение пакетного режима
    void waitStolbik();
    void transferPaket(int idModem, int maxTries, int dataPacket); //передача сообщения в пакетном режиме
    void sendCmd2();
    void modeIdle();
    void modeDirect();
    void modeRound();
    void settingsChannelDirect(Channel chDirect);
    void settingsChannelRound(ChannelRound chRound);


};

#endif // HYDROACOUSTICS_H
