#include "vma_controller.h"
#include <QSerialPort>
#include <QIODevice>
#include <QDebug>

VMA_controller::VMA_controller(const QString& portName,
                             const qint32 baud,
                             const QSerialPort::DataBits data,
                             const QSerialPort::StopBits stop,
                             const QSerialPort::Parity parity,
                             const QSerialPort::FlowControl flow,
                             QObject* parent) :
    mPortName(portName),
    mBaud(baud),
    mDataBits(data),
    mStopBits(stop),
    mParity(parity),
    mFlow(flow),
    mTimer(this)
{
    connect(&mTimer, &QTimer::timeout, this, &VMA_controller::tick);
    X[414][0] = X[415][0] = X[414][1] = X[415][1] = 150;
}

void VMA_controller::start() {
    QMutexLocker lock(&mGuard);
    if (mSerialPort->isOpen()) {
        qDebug() << "Duplicate open request";
        return;
    }
    mSerialPort->setPortName(mPortName);
    mSerialPort->setBaudRate(mBaud);
    mSerialPort->setDataBits(mDataBits);
    mSerialPort->setStopBits(mStopBits);
    mSerialPort->setParity(mParity);
    mSerialPort->setFlowControl(mFlow);
    if (!mSerialPort->open(QIODevice::ReadWrite)) {
        qDebug() << "Error connecting to MCU" << mSerialPort->errorString();
        return;
    }
    mTimer.start(REQUEST_TIME);
    qDebug() << "Port opened";
    isConnected = true;
    emit started();
    return;
}

void VMA_controller::tick() {
    sendData();
}

bool VMA_controller::sendData() {
//    int_otladka++;
//    qDebug() << "int_otladka " << int_otladka;
//    if (int_otladka < 50) {
//    vmaVector[0] =  150;
//    vmaVector[1] =  150;
//    vmaVector[2] =  150;
//    vmaVector[3] =  150;
//    vmaVector[4] =  150;
//    vmaVector[5] =  150;
//    vmaVector[6] =  150;
//    vmaVector[7] =  150;
//    vmaVector[8] =  150;
//    vmaVector[9] =  150;
//    vmaVector[10] = 150;
//    vmaVector[11] = 150;
//    } else {
//        vmaVector[0] =  200;
//        vmaVector[1] =  200;
//        vmaVector[2] =  200;
//        vmaVector[3] =  200;
//        vmaVector[4] =  200;
//        vmaVector[5] =  200;
//        vmaVector[6] =  200;
//        vmaVector[7] =  200;
//        vmaVector[8] =  200;
//        vmaVector[9] =  200;
//        vmaVector[10] = 200;
//        vmaVector[11] = 200;
//    }
//    qDebug() << "отладочный вывод, отправляю на стм" << vmaVector[0];

//    qDebug() << "отправка совершена";
//    qDebug() << "отправка совершена";
    bool ret  = true;
    QByteArray packet;

    packet.append(0xff);
    packet.append(0xfd);
    for (int i = 0; i < 12; ++i) {
        packet.append((const char*)(vmaVector + i), sizeof(qint16));
    }
    qint16  crc = calculate_crc(packet);
    qint8 crc_low = crc & 0xff;
    qint8 crc_high = (crc >> 8);
//    packet.append(crc_low);
//    packet.append(crc_high);
    qint64 bytesWritten = mSerialPort->write(packet);
    if ((bytesWritten != packet.size())) {
        qDebug() << mSerialPort->errorString();
        ret = false;
    }
    packet.clear();//    vmaVector[0] =  K[0];
    //    vmaVector[1] =  K[1];
    //    vmaVector[2] =  K[2];
    //    vmaVector[3] =  K[3];
    //    vmaVector[4] =  K[4];
    //    vmaVector[5] =  K[5];
    //    vmaVector[6] =  K[6];
    //    vmaVector[7] =  K[7];
    //    vmaVector[8] =  K[8];
    //    vmaVector[9] =  K[9];
    //    vmaVector[10] = K[10];
    //    vmaVector[11] = K[11];
    return ret;
}

quint16 VMA_controller::calculate_crc(QByteArray array) {

    int len = array.size();
    quint16 wcrc = 0xFFFF; // preset 16 position crc register , The initial values are all 1
    quint8 temp;// Define intermediate variables
    int i = 0, j = 0; // Define count
    for (i = 0; i < len; i++) { // Cycle through each data

        temp = array.at(i);
        wcrc ^= temp;
        for (j = 0; j < 8; j++) {

            // Judge whether what is moved to the right is 1, If it is 1 XOR with polynomials .
            if (wcrc & 0x0001) {

                wcrc >>= 1; // First move the data one bit to the right
                wcrc ^= 0xA001; // XOR with the polynomial above
            } else // If not 1, Then directly remove
                wcrc >>= 1; // Direct removal
        }
    }
    temp = wcrc; //crc Value
    return wcrc;
}

void VMA_controller::integrate(double &input, double &output, double &prevOutput, double dt) {
    if (input>150)
    output = prevOutput + dt*input;
    if (input<150)
    output = prevOutput - dt*input;
    prevOutput = output;
    if (output>= 200) output = 200;
    else if (output <=100) output = 100;
}

void VMA_controller::setValues(const float Upl,const float Upp,const float Ucl,const float Ucp, const float Uzlv,
                               const float Uzpv, const float Uzln, const float Uzpn, const float Ulagz, const float Ulagp,
                               const float Udrop, const float Ugrab)
{
    double scale=2;
    if (K[99]!=0) scale = K[99];
    X[80][0]=vmaVector[0] = round(Upl/scale)+K[98];
    X[81][0]=vmaVector[1] = round(Upp/scale)+K[98];
    X[82][0]=vmaVector[2] = round(Ucl/scale)+K[98];
    X[83][0]=vmaVector[3] = round(Ucp/scale)+K[98];
    X[84][0]=vmaVector[4] = round(Uzlv/scale)+K[98];
    X[85][0]=vmaVector[5] = round(Uzpv/scale)+K[98];
    X[86][0]=vmaVector[6] = round(Uzln/scale)+K[98];
    X[87][0]=vmaVector[7] = round(Uzpn/scale)+K[98];
    X[88][0]=vmaVector[8] = round(Ulagz/scale)+K[98];
    X[89][0]=vmaVector[9] = round(Ulagp/scale)+K[98];
//    vmaVector[10] = 150;
//    vmaVector[11] = 150;
    vmaVector[10] = X[94][0]=round(Udrop/scale)+K[98]; //UservoDrop
    vmaVector[11] = X[95][0]=round(Ugrab/scale)+K[98]; //UservoGrab

//    X[94][0]=round(Udrop/scale)+K[98]; //UservoDrop
//    X[95][0]=round(Ugrab/scale)+K[98]; //UservoGrab
//    integrate(X[94][0], X[414][0], X[414][1], 0.001);
//    integrate(X[95][0], X[415][0], X[415][1], 0.001);
//    vmaVector[10] = X[414][0];
//    vmaVector[11] = X[415][0];

//    qDebug() << "UservoDrop X[94][0]: "  << X[94][0];
//    qDebug() << "UservoGrab X[95][0]: "  << X[95][0];
//    qDebug() << "UservoDrop vmaVector[10]: "  << vmaVector[10];
//    qDebug() << "UservoGrab vmaVector[11]: "  << vmaVector[11];
//    qDebug() << "X[84][0]: "  << X[84][0];
//    qDebug() << "X[85][0]: "  << X[85][0];
//    qDebug() << "X[86][0]: "  << X[86][0];
//    qDebug() << "X[87][0]: "  << X[87][0];
//    vmaVector[0] = vmaVector[1] = vmaVector[2] = vmaVector[3] = vmaVector[4] = vmaVector[5] = vmaVector[6] = vmaVector[7] = vmaVector[8] = vmaVector[9] = vmaVector[10] = vmaVector[11] = 150;
}

void VMA_controller::stop() {
    QMutexLocker lock(&mGuard);
    if (mSerialPort && mSerialPort->isOpen())
        mSerialPort->close();
    emit finished();
}

VMA_controller::~VMA_controller() {
    stop();
}
