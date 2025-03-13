#ifndef JSON_PARSER_H
#define JSON_PARSER_H

#include <QObject>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QIODevice>
#include <QFile>
#include <QDebug>

struct Settings
{
    QString comIMU;
    QString comHydro;
    QString puwv1_channel_settings;
    QString receiver_ip;
    int receiver_port;
    QString sender_ip;
    int sender_port;
    float sender_frequency;
    bool mode_package_delivery = false; //Выбор режима как часто отправлять посылку о состоянии модема, если 0, то при отправке, если  1, то каждые 3 секунды
};

class Json_parser
{
public:
    Json_parser();
    Settings set;
    QString val;
    QFile file;
    void parser(QString val);
};

#endif // JSON_PARSER_H
