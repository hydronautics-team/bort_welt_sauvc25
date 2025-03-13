#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <QDebug>

#pragma pack(push,1)

struct ControlData
{ //упраление от верхнего уровня на нижний и реальные данные с БСО
    float surge     = 0;
    float sway      = 0;
    float depth     = 0;
    float roll      = 0;
    float pitch     = 0;
    float yaw       = 0;
};

struct ControlContoursFlags
{ //флаги замыкания контуров (если 1, то замкнуты, 0 - разомкнуты)
    uint8_t stab_depth = 0;
    uint8_t stab_roll = 0;
    uint8_t stab_pitch = 0;
    uint8_t stab_yaw = 0;
};

struct ServoMotors
{
    uint8_t dropper = 0;  // 0-закрыта, 256 - открыта
    uint8_t grabber = 0;
};

struct ToHighLevel
{
    uint8_t reset_imu = 0; //сброс БСО
    ControlContoursFlags controlContoursFlags; //флаги замыкания контуров (если больше 0, то замкнуты)
    ControlData controlData; //данные с датчиков, марш и лаг - скорости, остальное - реальный сигнал
    float accel_X = 0;
    float accel_Y = 0;
    ServoMotors servoMotors; //обратная связь от упраления сервами на сброс/подъем
    char stolb_first;
    char stolb_second;
    char stolb_third;
    float distance_hydroacustic;
    uint8_t peleng_hydroacustic = 0; //угол пеленга
    float depth_repentance = 0; //отстояние от дна
    uint16_t checksum;
};

//структура данных, которая передается от верхнего уровня на нижний
struct FromHighLevel
{
    uint8_t reset_imu = 0; //сброс БСО
    ControlContoursFlags controlContoursFlags; //флаги замыкания контуров (если больше 0, то замкнуты)
    ControlData controlData; //данные для отработки, марш и лаг - скорости, остальное - входной сигнал для контуров
    float accel_X = 0;
    float accel_Y = 0;
    ServoMotors servoMotors; //упраление сервами на сброс/подъем
    char stolb_first;
    char stolb_second;
    char stolb_third;
    float distance_hydroacustic;
    uint16_t checksum;
};

#pragma pack (pop)


#endif // PROTOCOL_H
