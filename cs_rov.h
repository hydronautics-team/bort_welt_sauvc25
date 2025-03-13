#ifndef CS_ROV_H
#define CS_ROV_H

#include "vectorNav/vectorNav.h"
#include <QThread>
#include <QSettings>
#include "VMA/vma_controller.h"
#include "math.h"
#include <qmath.h>
#include <QTime>
#include <QDebug>
#include "protocol_bort_AUV/pc_protocol.h"
#include "uWaveSingapore/statemachine.h"
#include "PS_MS5837/ms5837.h"
#include "hydroacoustic_peleng/hydroPeleng.h"

const QString ConfigFile = "protocols.conf";
//const QString ConfigFile = "kx_protocol.conf";
const QString XI = "xi";
const QString KI = "ki";

class CS_ROV : public QObject
{
    Q_OBJECT
public:
    CS_ROV(QObject * parent = nullptr);

    void parseJsonFile(QString filePath);
    void start(int dt){
        timer.start(dt);
    }

public slots:
    void tick();
    void resetValues();

public:
    double limit (double value, double limit){
        if(fabs(value)>limit) return (limit*sgn(value));
        else return value;
    }
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
    float saturation(float input,  float max, float min);
    float saturationPS(float input,  float max, float min);
    double yawErrorCalculation(float yawDesiredDeg, float yawCurrentDeg);
    int sign(double input);

protected:
    StateMachine* stuWave = nullptr;
    MS5837 *i2c = nullptr;
    void processDesiredValuesAutomatiz(double inputFromRUD, double &output, double &prev_output, double scaleK,
                                       bool flagLimit = false, double maxValue=180, double dt=0.01);
    void integrate(double &input, double &output, double &prevOutput, double dt);
    void exchangeJetson();
    void closeExchangeJetson();
    void readDataFromHighLevel();
    void readDataFromSensors();
    void reset_vectorNav();
    void regulators();
    void alternative_yaw_calculation(float dt);
    void controlYaw(double dt);
    void controlRoll(double dt);
    void controlPitch(double dt);
    void controlDepth(double dt);
    void resetYawChannel();
    void resetRollChannel();
    void resetPitchChannel();
    void resetDepthChannel();
    void BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux, double Uy, double Uz);
    void writeDataToVMA();
    void writeDataToHighLevel();
    void depth(double dt);
    void aperiodicFilter(double &input, double &output, double &prevOutput, double K, double T, double dt);
    VectorNavProtocol *vn100Proto = nullptr;
    PelengProtocol *hydroPeleng = nullptr;
    VMA_controller* vmaProtocol = nullptr;
    VMA_controller* vmaProtocol1 = nullptr;
    VMA_controller* vmaProtocol2 = nullptr;
    VMA_controller* vmaProtocol3 = nullptr;
    VMA_controller* vmaProtocol4 = nullptr;
    //обмен с пультом
    ControlSystem::PC_Protocol *auvProtocol = nullptr;
    QTimer timer;
    QTimer timerRound;
    QTimer timerVMA;
    QTimer timerReceived;
    QThread vmaThread;
    QThread psThread;
    QThread uWaveThread;
    quint8 contour_closure_yaw = 0;
    quint8 contour_closure_pitch = 0;
    quint8 contour_closure_roll = 0;
    quint8 contour_closure_depth = 0;
    QTime timeRegulator;
    QTime timeYaw;

    double UservoDrop = 0;
    double UservoGrab = 0;

    char stolb1 = 0;
    char stolb2 = 0;
    char stolb3 = 0;

    double drewYaw = 0;
    double drewYawAuto = 0;
    // РАСКОММЕНТИРОВАТЬ, когда флаг выставления в 0 имушки будет приходить от Джетсона и закомментить следующую строку
//    bool flagYawInit = false;
    bool flagYawInit = true;
    bool flagYawAuto = false;
    quint8 flag_of_mode = 100;

    //для альтернативного метода расчета угла курса
    double A[3][3];  //матрица перехода
    double I[3];   //Ix, Iy, Iz
};

#endif // CS_ROV_H
