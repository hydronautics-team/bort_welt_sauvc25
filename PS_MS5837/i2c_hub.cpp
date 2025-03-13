#include "i2c_hub.h"

I2C_HUB::I2C_HUB(uint8_t i2cHubAddr, QObject *parent)
{
    sensor_1 = new MS5837();
    _i2cHubAddr = i2cHubAddr;
    initHub();
}

bool I2C_HUB::initHub()
{
    fd = wiringPiI2CSetup( _i2cHubAddr);
    qDebug() << "fd: " << fd;
   if (fd == -1)
   {
       qDebug() <<"потеряна связь c хабом";
       return -1;
   }
       qDebug()<<"связь с хабом есть)";
    I2C_HUB::setBusChannel(1);
    sensor_1->init();
}

void I2C_HUB::getPressusre()
{
    I2C_HUB::setBusChannel(1);
//    sensor_1->pressure();
    I2C_HUB::setBusChannel(2);
//    sensor_2->pressure();
    depth[0] = sensor_1->depth();
    depth[1] = sensor_2->depth();
    qDebug() << "sensor_1->pressure" << sensor_1->pressure();
    qDebug() << "sensor_2->pressure" << sensor_2->pressure();
}

void I2C_HUB::setBusChannel(uint8_t channel)
{
    if (channel >= COUNT_CHANNEL) {
        return;
    }
    wiringPiI2CWrite(fd ,(channel | ENABLE_MASK));
}
