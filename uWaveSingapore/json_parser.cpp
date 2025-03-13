#include "json_parser.h"

Json_parser::Json_parser()
{
    QString val;
    QFile file;

    file.setFileName("settings.json");
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    val = file.readAll();
    file.close();
    parser(val);

}

void Json_parser::parser(QString val)
{

    QJsonDocument doc = QJsonDocument::fromJson(val.toUtf8());
    QJsonObject json = doc.object();
//    set.comIMU = json["COM_IMU"].toString();
    set.comHydro = json["hydroUWave"].toString();
}
