#ifndef I2C_HUB_H
#define I2C_HUB_H

#include <QObject>

class I2C_HUB : public QObject
{
    Q_OBJECT
public:
    explicit I2C_HUB(QObject *parent = nullptr);

signals:

};

#endif // I2C_HUB_H
