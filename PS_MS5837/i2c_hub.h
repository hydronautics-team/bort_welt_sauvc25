#ifndef I2C_HUB_H
#define I2C_HUB_H

#include <QObject>
#include "ms5837.h"

#define DEFAULT_I2C_HUB_ADDRESS 0x70
#define ENABLE_MASK 0x08
#define DEFAULT_CHANNEL 0
#define COUNT_CHANNEL 8

class I2C_HUB : public QObject
{
    Q_OBJECT
public:
    explicit I2C_HUB(, QObject *parent = nullptr);
    bool initHub();
    void getPressusre(); //читаем два датчика
    void setBusChannel(uint8_t channel);
    int fd =0;
    float depth[2]; //выходной массив

private:
    uint8_t _i2cHubAddr;
    MS5837 * sensor_1 = nullptr;

signals:

};

#endif // I2C_HUB_H
