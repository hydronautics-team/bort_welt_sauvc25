#ifndef MS5837_H
#define MS5837_H

#include "wiringPi.h"
#include "wiringPiI2C.h"
#include <iostream>
#include <QDebug>
#include <cmath>
#include <QTimer>
#include <QTime>
#include <QObject>
#include <QThread>
#include "linux/i2c.h"

extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#define DEFAULT_I2C_HUB_ADDRESS 0x70
#define ENABLE_MASK 0x08
#define DEFAULT_CHANNEL 0
#define COUNT_CHANNEL 8

//для использования kx-pult
extern double X[2000][2];
extern QVector<double> K;

class MS5837 : public QObject
{
    Q_OBJECT
public:

    MS5837(uint8_t i2cHubAddr = DEFAULT_I2C_HUB_ADDRESS, QObject *parent = nullptr);

    double return_depth_1();
    double return_depth_2();


    static const double Pa;
    static const double bar;
    static const double mbar;

    static const uint8_t MS5837_30BA;
    static const uint8_t MS5837_02BA;
    static const uint8_t MS5837_UNRECOGNISED;
    int devId = 0x76;
    int fd_0 =0;
    int fd_1 =0;

    bool init(int &fd);
    bool begin(); // Calls init()
    void setBusChannel(uint8_t channel);

    /** Установите модель датчика MS5837. Допустимыми параметрами являются MS5837::MS5837_30BA (по умолчанию)
     * и MS5837::MS5837_02BA.
     */
    void setModel(uint8_t model);
    uint8_t getModel();

    /** Укажите плотность рабочей жидкости в кг/м^3. По умолчанию используется для
     * морской воды. Для пресной воды должно быть 997.
     */

    void setFluidDensity(double density);

    /** Считывание с I2C занимает до 40 мс, поэтому возможно экономное использование.
     */
    void read(int &fd);

    /** Возвращаемое давление в мбар или мбар*коэффициент пересчета.
     */
    double pressure(double conversion = 1.0f);

    /** Возвращаемая температура в градусах C.
     */
    double temperature();

    /** Возвращаемая глубина в метрах (допустима для работы в несжимаемой
     * только жидкости. Используется плотность, установленная для пресной или морской воды.
     */
    double depth();

    double depth_1 = 0;
    double depth_2 = 0;



    /** Возвращаемая высота в метрах (действительна только для работы в воздухе).
     */
    double altitude();
    __u16 C[8];
    unsigned char* per;
    __u32 D1_pres, D2_temp;
    int32_t TEMP;
    int32_t P;
    uint8_t _model;

    QTimer t;

//    double fluidDensity = 1029;
    double fluidDensity = 88.6;

    /** Выполняет вычисления в соответствии с техническими данными датчика для преобразования и
    * компенсации второго порядка.
    */
    void calculate();

    uint8_t crc4(uint16_t n_prom[]);

private:
    QTimer timerPS;
    uint8_t _i2cHubAddr;
    quint8 sensorChoice = 1; // 1 - 1 sensor, 2 - 2 sensor
    // В классе MS5837 добавить отдельный fd для хаба
    int hubFd = -1;
public slots:
    void timeoutSlot();
signals:
    void started();
public slots:
    virtual void start();
};

#endif // MS5837_H
