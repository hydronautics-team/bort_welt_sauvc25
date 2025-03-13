#include "cs_rov.h"
#include <QDebug>
#include "uWaveSingapore/statemachine.h"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <vector>
#include <algorithm>
#include <deque>

double X[2000][2];

CS_ROV::CS_ROV(QObject *parent)
{
    parseJsonFile("settings.json"); //чтение джейсона и создание объектов под все обмены с датчиками

    auvProtocol = new ControlSystem::PC_Protocol(ConfigFile,"AUV");
    qDebug() << "---start exchange";
    auvProtocol->startExchange();

    //РАСКОММЕНТИТЬ В РЕАЛЬНЫХ ТЕСТАХ С ДЖЕТСОНОМ
//    connect(&timerReceived, &QTimer::timeout, this, &CS_ROV::closeExchangeJetson);
//    timerReceived.start(1500);
//    connect(auvProtocol, &ControlSystem::PC_Protocol::dataReceived, this, &CS_ROV::exchangeJetson);

    connect(&timer, &QTimer::timeout, this, &CS_ROV::tick);
    connect(&timerVMA, &QTimer::timeout, this, &CS_ROV::writeDataToVMA);
    timer.start(10);
    timeRegulator.start();

//    X[91][0]=X[91][1]=0; //нулевые НУ для интегрирования угловой скорости и нахождения угла курса
    X[17][0] = X[17][1] = 0; // коэфф, дающий возможность привести курс к 0 на бортике
    X[77][0] = X[77][1] = 0;
}

void CS_ROV::parseJsonFile(QString filePath)
{
    QFile file(filePath);
    if (!file.exists()) {
        qDebug() << "No config file : " << filePath;
    } else {
        qDebug() << "Config file opened";
        file.open(QIODevice::ReadOnly);
        QJsonDocument doc = QJsonDocument().fromJson(file.readAll());
        QJsonObject confObject = doc.object();

        // Обмен с ВМА
        QJsonObject vmaObj = confObject.value("VMA").toObject();
        vmaProtocol = new VMA_controller(vmaObj.value("device").toString(), 115200);
        qDebug() << "vmaObj.value" << vmaObj.value("device");
        vmaProtocol->moveToThread(&vmaThread);
        QObject::connect(&vmaThread, &QThread::started, vmaProtocol, &VMA_controller::start);
        vmaThread.start();

        // Обмен с векторнавом
        QJsonObject bsoObj = confObject.value("vectorNav").toObject();
        vn100Proto = new VectorNavProtocol(bsoObj.value("device").toString());

        QJsonObject pelengObj = confObject.value("hydroPeleng").toObject();
        hydroPeleng = new PelengProtocol(pelengObj.value("device").toString(), 115200);

        // Обмен с датчиком давления
        i2c = new MS5837();
        i2c->moveToThread(&psThread);
        QObject::connect(&psThread, &QThread::started, i2c, &MS5837::start);
        psThread.start();

        // Обмен с гидроакустическим модемом uWave
        stuWave = new StateMachine(nullptr);
        stuWave->moveToThread(&uWaveThread);
        uWaveThread.start();
    }
    qDebug() << "end parseJsonFile";
}

void CS_ROV::tick()
{
    readDataFromHighLevel();
    readDataFromSensors();
    reset_vectorNav();
    regulators();
    BFS_DRK(X[101][0], X[102][0], X[103][0] , X[104][0], X[105][0], X[106][0]);
    writeDataToVMA();
    writeDataToHighLevel();
}

void CS_ROV::integrate(double &input, double &output, double &prevOutput, double dt) {
    output = prevOutput + dt*input;
    prevOutput = output;
}

void CS_ROV::exchangeJetson()
{
    timerReceived.start(1500);
    if (!timer.isActive())
    {
        timer.start(10);
    }
}

void CS_ROV::closeExchangeJetson()
{
    timer.stop();
    resetValues();
}

void CS_ROV::resetValues()
{
    vmaProtocol->setValues(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

float CS_ROV::saturation(float input, float max, float min)
{
    if (input>= max) return max;
    else if (input <=min) return min;
    else return input;
}

float CS_ROV::saturationPS(float input, float prev_input, float step)
{
    if ((prev_input !=0) and (input !=K[209]) and (input !=K[206]))
    {
        float delta = abs(input-prev_input);
        if (delta>= step) return prev_input;
        else
        {
            prev_input=input;
            return input;
        }
    }
    else if ((input !=K[209]) and (input !=K[206])) prev_input=input;
    else prev_input = 0;
    return input;

}

void CS_ROV::processDesiredValuesAutomatiz(double inputFromRUD, double &output, double &prev_output,
                                           double scaleK, bool flagLimit, double maxValue, double dt) {
    double inputScaled = inputFromRUD*scaleK;
    integrate(inputScaled,output,prev_output,dt);
    if (flagLimit){
        saturation(output,maxValue,-maxValue);
    }
}

double CS_ROV::yawErrorCalculation(float yawDesiredDeg, float yawCurrentDeg)
{
    double l0 =0, l2 =0;
    double Krad = M_PI/180.0;
    double Kdeg = 180/M_PI;
    double desiredPsi = yawDesiredDeg*Krad;
    double currentPsi = yawCurrentDeg*Krad;
    l0=cos(desiredPsi/2)*cos(currentPsi/2)+sin(desiredPsi/2)*sin(currentPsi/2);
    l2=cos(desiredPsi/2)*sin(currentPsi/2)-cos(currentPsi/2)*sin(desiredPsi/2);
    if (fabs(l0)>1) l0=sign(l0)*1;
    if (l0<0) l0*=-1;
    else l2 *=-1;
    double temp = 2*acos(l0);
    double temp_deg = 2*acos(l0)*Kdeg;
    double temp_deg_sign = 2*acos(l0)*sign(l2)*Kdeg;
    return temp_deg_sign;
}

int CS_ROV::sign(double input)
{
    if (input>=0) return 1;
    else return -1;
}

void CS_ROV::readDataFromHighLevel()
{
    X[51][0] = K[171];
    X[52][0] = K[172];
    X[53][0] = K[173];
    X[54][0] = K[174];
    X[55][0] = K[175];
    X[56][0] = K[176];

    //ДЛЯ РЕАЛЬНОГО АППАРАТА раскооментить при обмене с Джетсоном
//    X[51][0] = auvProtocol->rec_data.controlData.yaw;
//    X[52][0] = auvProtocol->rec_data.controlData.pitch;
//    X[53][0] = auvProtocol->rec_data.controlData.roll;
//    X[54][0] = auvProtocol->rec_data.controlData.surge;
//    X[55][0] = auvProtocol->rec_data.controlData.sway;
//    X[56][0] = auvProtocol->rec_data.controlData.depth;

    // РАСКОММЕНТИРОВАТЬ, когда флаг выставления в 0 имушки будет приходить от Джетсона
//    flagYawInit = auvProtocol->rec_data.reset_imu;

//    UservoDrop = K[7];
//    UservoGrab = K[8];

     UservoDrop = auvProtocol->rec_data.servoMotors.dropper;
//    UservoGrab = auvProtocol->rec_data.servoMotors.grabber;
}

void CS_ROV::readDataFromSensors()
{
    X[18][0] = vn100Proto->data.yaw;

    X[61][0] = vn100Proto->data.yaw + X[17][0]; //для сброса в 0 на бортике
    X[62][0] = -vn100Proto->data.pitch;
    X[63][0] = vn100Proto->data.roll;

    X[64][0] = vn100Proto->data.X_accel;
    X[65][0] = vn100Proto->data.Y_accel;
    X[66][0] = vn100Proto->data.Z_accel;

    X[68][0] = vn100Proto->data.X_rate; //wx крен
    X[67][0] = -vn100Proto->data.Y_rate; //wy дифферент
    X[69][0] = vn100Proto->data.Z_rate; //wz

    X[57][0] = hydroPeleng->data;

    X[276][0] = stuWave->hydro->uwave.puwvj.dataPacket; //порядок столбиков.
    X[277][0] = stuWave->hydro->uwave.puwv3.propTime; //время распространения сигнала в одну или две стороны не помню

    switch (stuWave->hydro->uwave.puwvj.dataPacket) {
    case 1100:
        stolb1 = 82; //Red
        stolb2 = 89; //Yellow
        stolb3 = 66; //Blue
        break;
    case 1200:
        stolb1 = 82; //Red
        stolb2 = 66; //Blue
        stolb3 = 89; //Yellow
        break;
    case 1300:
        stolb1 = 66; //Blue
        stolb2 = 82; //Red
        stolb3 = 89; //Yellow
        break;
    case 1400:
        stolb1 = 66; //Blue
        stolb2 = 89; //Yellow
        stolb3 = 82; //Red
        break;
    case 1500:
        stolb1 = 89; //Yellow
        stolb2 = 66; //Blue
        stolb3 = 82; //Red
        break;
    case 1600:
        stolb1 = 89; //Yellow
        stolb2 = 82; //Red
        stolb3 = 66; //Blue
        break;
    case 2000: //хардкод под всплытие в конце
        stolb1 = 72; //High
        stolb2 = 72; //High
        stolb3 = 72; //High
        break;
    default:
        stolb1 = 0;
        stolb2 = 0;
        stolb3 = 0;
        break;
    }
}

void CS_ROV::reset_vectorNav()
{
    if (flagYawInit == 1) {
        flagYawInit = 0;
        //Обнулить курс в момент старта на бортике
        X[17][0] = -X[18][0];
        auvProtocol->send_data.reset_imu = 1;
    }
}

void CS_ROV::alternative_yaw_calculation(float dt)
{
    X[170][0] = X[70][0] + K[70]; //Mx с учетом коррекции
    X[171][0] = X[71][0] + K[71]; //My с учетом коррекции
    X[172][0] = X[72][0] + sin(0.5*X[63][0]/57.3)*K[72]; //Mz с учетом коррекции

    double teta = X[62][0]*M_PI/180;
    double gamma = X[63][0]*M_PI/180;
    X[176][0] = teta;
    X[177][0] = gamma;
    A[0][0] = cos(teta); A[0][1] = sin(teta)*sin(gamma); A[0][2] = -sin(teta)*cos(gamma);
    A[1][0] = 0; A[1][1] = cos(gamma); A[1][2] = sin(gamma);
    A[2][0] = sin(teta); A[2][1] = -sin(gamma)*cos(teta); A[2][2] = cos(teta)*cos(gamma);

    X[300][0] = I[0] = A[0][0]*X[170][0] + A[0][1]*X[171][0] + A[0][2]*X[172][0];
    X[400][0] = I[1] = A[1][0]*X[170][0] + A[1][1]*X[171][0] + A[1][2]*X[172][0];
    X[500][0] = I[2] = A[2][0]*X[170][0] + A[2][1]*X[171][0] + A[2][2]*X[172][0];

    X[174][0] = I[0];
    X[175][0] = I[1];
    X[178][0] = atan2(-I[1],-I[0])*57.3;

    X[79][0] = -1/cos(X[62][0]/57.3)*(-X[69][0]*cos(X[63][0]/57.3)-X[68][0]*sin(X[63][0]/57.3));
    integrate(X[79][0],X[91][0],X[91][1],dt); //интегрируем показание Z_rate для нахождения текущего угла курса
}

void CS_ROV::regulators()
{
    float dt = timeRegulator.elapsed()*0.001;//реальный временной шаг цикла
    timeRegulator.start();
    depth(dt);
//    alternative_yaw_calculation(dt);
    X[104][0] = K[184]*X[54][0]; //Ux
    X[105][0] = K[185]*X[55][0]; //Uy
    //qDebug() << "rec_data.controlContoursFlags.stab_yaw: " << auvProtocol->rec_data.controlContoursFlags.stab_yaw;
//    if (auvProtocol->rec_data.controlContoursFlags.stab_yaw == 0) { //контур курса разомкнут
  if (K[1] == 0) { //контур курса разомкнут ДЛЯ ОТЛАДКИ БЕЗ ДЖЕТСОНА
        X[101][0] = K[181]*X[51][0]; //Upsi
        contour_closure_yaw = 0;
        resetYawChannel();
    } else { //контур курса замкнут
       contour_closure_yaw = 1;
       controlYaw(dt);
       }
    //qDebug() << "rec_data.controlContoursFlags.stab_roll: " << auvProtocol->rec_data.controlContoursFlags.stab_roll;

//    if (auvProtocol->rec_data.controlContoursFlags.stab_roll == 0) { //контур крена разомкнут
    if (K[3] == 0) { //контур крена разомкнут ДЛЯ ОТЛАДКИ БЕЗ ДЖЕТСОНА
        X[103][0] = K[183]*X[53][0]; //Uroll
        contour_closure_roll = 0;
        resetRollChannel();
    } else {
        contour_closure_roll = 1;
        controlRoll(dt);
    }
    //qDebug() << "rec_data.controlContoursFlags.stab_pitch: " << auvProtocol->rec_data.controlContoursFlags.stab_pitch;

//   if (auvProtocol->rec_data.controlContoursFlags.stab_pitch == 0) {
    if (K[2] == 0) { //контур дифферента разомкнут ДЛЯ ОТЛАДКИ БЕЗ ДЖЕТСОНА
        X[102][0] = K[182]*X[52][0]; //Upith
        contour_closure_pitch = 0;
        resetPitchChannel();
    } else {
        contour_closure_pitch = 1;
        controlPitch(dt);
    }
   //qDebug() << "rec_data.controlContoursFlags.stab_depth: " << auvProtocol->rec_data.controlContoursFlags.stab_depth;

//   if (auvProtocol->rec_data.controlContoursFlags.stab_depth == 0) {
    if (K[4] == 0) { //контур глубины разомкнут ДЛЯ ОТЛАДКИ БЕЗ ДЖЕТСОНА
        X[106][0] = K[186]*X[56][0]; //Uz
        contour_closure_depth = 0;
        resetDepthChannel();
    } else {
        contour_closure_depth = 1;
        controlDepth(dt);
    }
}

void CS_ROV::controlYaw(double dt)
{
    X[111][0] = yawErrorCalculation(X[51][0],X[61][0]); //учет предела работы датчика, пересчет кратчайшего пути
    X[112][0] = X[111][0]*K[211];
    X[113][0] = X[112][0]*K[212];

    X[114][0] = X[114][1] + 0.5*(X[113][0] + X[113][1])*dt; //выходное значение интегратора без полок
    if (K[213] != 0){//значит заданы полки
        X[114][0] = saturation(X[114][0],K[213],K[214]); //выходное значение интегратора с полками
    }
    X[114][1] = X[114][0];
    X[113][1] = X[113][0];

    X[116][0] = X[114][0] + X[112][0];
    aperiodicFilter(X[69][0],X[122][0],X[122][1],K[217],K[218],dt);
    X[121][0] = X[122][0]*K[210];
    X[119][0] = X[51][0]*K[219];
    X[117][0] = X[116][0] - X[121][0] + X[119][0];
    X[118][0] = saturation(X[117][0],K[216],-K[216]);
    X[101][0] = X[118][0]*K[201];
}

void CS_ROV::controlRoll(double dt)
{
    X[801][0] = X[53][0] - X[63][0];
    X[802][0] = X[801][0] * K[801];
    X[803][0] = X[802][0] * K[802];
    integrate(X[803][0],X[804][0],X[804][1],dt);
    if (K[803] != 0) {
        X[804][1] = saturation(X[804][0],K[804],-K[804]);
        X[804][0] = saturation(X[804][0],K[804],-K[804]);
    }
    X[814][0] = K[814] + sin((X[53][0]-K[816])/57.3) * K[815];

    X[806][0] = X[802][0] + X[804][0];
    X[805][0] = X[53][0] * K[805];
    X[807][0] = X[806][0] + X[805][0]+ X[814][0];

    //speed loop
    aperiodicFilter(X[68][0],X[808][0],X[808][1],1,K[808],dt);
    X[809][0] = X[808][0] * K[809];
    X[810][0] = X[807][0] - X[809][0] + K[817]*X[56][0];
    X[811][0] = saturation(X[810][0],K[810],-K[810]);
    X[103][0] = K[811] * X[811][0];
}

void CS_ROV::controlPitch(double dt)
{
   X[311][0] = X[52][0] - X[62][0];
   X[312][0] = X[311][0]*K[311];
   X[313][0] = X[312][0]*K[312];
   X[314][0] = X[314][1] + 0.5*(X[313][0] + X[313][1])*dt; //выходное значение интегратора без полок
   if (K[313] != 0){//значит заданы полки
       X[314][0] = saturation(X[314][0],K[313],K[314]); //выходное значение интегратора с полками
   }
   X[314][1] = X[314][0];
   X[313][1] = X[313][0];

   X[316][0] = X[314][0] + X[312][0];
   aperiodicFilter(X[67][0],X[320][0],X[320][1],1,K[320],dt);
   X[321][0] = X[320][0]*K[318];
   X[319][0] = X[52][0]*K[319];
   X[317][0] = X[316][0] - X[321][0] + X[319][0];
   X[318][0] = saturation(X[317][0],K[316],-K[316]);
   X[102][0] = X[318][0]*K[300];
}

void CS_ROV::controlDepth(double dt)
{
   X[601][0] = X[56][0] - X[279][0];
   X[602][0] = X[601][0]*K[600]; //Усиление ошибки по положению
   X[603][0] = X[602][0]*K[601];
   X[604][0] = X[604][1] + 0.5*(X[603][0] + X[603][1])*dt; //выходное значение интегратора без полок
   if (K[603] != 0){//значит заданы полки
        X[604][0] = saturation(X[604][0],K[602],K[603]); //выходное значение интегратора с полками
   }
   X[604][1] = X[604][0];
   qDebug() << "X[604][0]" <<X[604][0];
   X[603][1] = X[603][0];
   X[605][0] = X[602][0] + X[604][0];
   X[610][0] = (X[279][0]-X[279][1])/dt; // Дифференцирование
   X[611][0] = X[610][0]*K[606]; // Усиление обратной связи по скорости
   X[613][0] = X[56][0]*K[605]; // Усиление возмущающего воздействия по скорости
   X[606][0] = X[605][0] - X[611][0] + X[613][0]; // Получение ошибки по скорости
   X[607][0] = saturation(X[606][0],K[604],-K[604]); // Насыщение в контуре скорости
   X[106][0] = X[607][0]*K[607]; // Усиление ошибки по скорости
   X[279][1] = X[279][0];
}

void CS_ROV::resetYawChannel()
{
    X[114][1] = X[114][0] =0;
}

void CS_ROV::resetRollChannel()
{
    X[804][0] = X[804][1] = 0;
}

void CS_ROV::resetPitchChannel()
{
    X[314][0] = X[314][1] = 0;
}

void CS_ROV::resetDepthChannel()
{
    qDebug() << "resetDepthChannel";
    X[604][0] = X[604][1] = 0;
}

void CS_ROV::BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux, double Uy, double Uz)
{
    X[110][0] = (K[10]*Ux + K[11]*Uy + K[12]*Uz + K[13]*Ugamma + K[14]*Uteta + K[15]*Upsi)*K[16];//U1
    X[120][0] = (K[20]*Ux + K[21]*Uy + K[22]*Uz + K[23]*Ugamma + K[24]*Uteta + K[25]*Upsi)*K[26];//U2
    X[130][0] = (K[30]*Ux + K[31]*Uy + K[32]*Uz + K[33]*Ugamma + K[34]*Uteta + K[35]*Upsi)*K[36];//U3
    X[140][0] = (K[40]*Ux + K[41]*Uy + K[42]*Uz + K[43]*Ugamma + K[44]*Uteta + K[45]*Upsi)*K[46];//U4
    X[150][0] = (K[50]*Ux + K[51]*Uy + K[52]*Uz + K[53]*Ugamma + K[54]*Uteta + K[55]*Upsi)*K[56];//U5
    X[160][0] = (K[60]*Ux + K[61]*Uy + K[62]*Uz + K[63]*Ugamma + K[64]*Uteta + K[65]*Upsi)*K[66];//U6
    X[210][0] = (K[110]*Ux + K[111]*Uy + K[112]*Uz + K[113]*Ugamma + K[114]*Uteta + K[115]*Upsi)*K[116];//U7
    X[220][0] = (K[120]*Ux + K[121]*Uy + K[122]*Uz + K[123]*Ugamma + K[124]*Uteta + K[125]*Upsi)*K[126];//U8
    X[230][0] = (K[130]*Ux + K[131]*Uy + K[132]*Uz + K[133]*Ugamma + K[134]*Uteta + K[135]*Upsi)*K[136];//U9
    X[240][0] = (K[140]*Ux + K[141]*Uy + K[142]*Uz + K[143]*Ugamma + K[144]*Uteta + K[145]*Upsi)*K[146];//U10
    if (UservoDrop > 0) {
        X[250][0] = K[0];
    }
    if (UservoGrab > 0) {
        X[260][0] = K[9];
    }

    X[201][0] = limit(X[110][0],K[200]);
    X[211][0] = limit(X[120][0],K[200]);
    X[221][0] = limit(X[130][0],K[200]);
    X[231][0] = limit(X[140][0],K[200]);
    X[241][0] = limit(X[150][0],K[200]);
    X[251][0] = limit(X[160][0],K[200]);
    X[261][0] = limit(X[210][0],K[200]);
    X[271][0] = limit(X[220][0],K[200]);
    X[281][0] = limit(X[230][0],K[200]);
    X[291][0] = limit(X[240][0],K[200]);
    X[401][0] = limit(X[260][0],K[205]);
    X[411][0] = limit(X[250][0],K[205]);
}

void CS_ROV::writeDataToHighLevel()
{

    auvProtocol->send_data.controlContoursFlags.stab_yaw = contour_closure_yaw;
    auvProtocol->send_data.controlContoursFlags.stab_pitch = contour_closure_pitch;
    auvProtocol->send_data.controlContoursFlags.stab_roll = contour_closure_roll;
    auvProtocol->send_data.controlContoursFlags.stab_depth = contour_closure_depth;

    auvProtocol->send_data.controlData.yaw = X[61][0]; //61 - это с датчика
    auvProtocol->send_data.controlData.pitch = X[62][0];
    auvProtocol->send_data.controlData.roll = X[63][0];
    auvProtocol->send_data.controlData.surge = X[104][0];
    auvProtocol->send_data.controlData.sway = X[105][0];
    auvProtocol->send_data.controlData.depth = X[279][0];

//    qDebug() << "auvProtocol->send_data.controlData.yaw=" << auvProtocol->send_data.controlData.yaw;

    auvProtocol->send_data.accel_X = X[64][0];
    auvProtocol->send_data.accel_Y = X[65][0];

//    qDebug() << "auvProtocol->send_data.accel_X=" << auvProtocol->send_data.accel_X;
//    qDebug() << "auvProtocol->send_data.accel_Y=" << auvProtocol->send_data.accel_Y;

    auvProtocol->send_data.servoMotors.dropper = UservoDrop; //исправить потом!!
    //qDebug() << "UservoDrop=" << UservoGrab;
    auvProtocol->send_data.servoMotors.grabber = UservoGrab;

    auvProtocol->send_data.reset_imu = flagYawInit;

    auvProtocol->send_data.stolb_first =  stolb1;
    auvProtocol->send_data.stolb_second = stolb2;
    auvProtocol->send_data.stolb_third =  stolb3;

    X[651][0] = stolb1;
    X[652][0] = stolb2;
    X[653][0] = stolb3;
//    qDebug() << "auvProtocol->send_data.stolb_first =" << auvProtocol->send_data.stolb_first;
//    qDebug() << "auvProtocol->send_data.stolb_second =" << auvProtocol->send_data.stolb_second;
//    qDebug() << "auvProtocol->send_data.stolb_third =" << auvProtocol->send_data.stolb_third;

    auvProtocol->send_data.distance_hydroacustic = stuWave->hydro->uwave.puwv3.propTime*1500;
    auvProtocol->send_data.peleng_hydroacustic = hydroPeleng->data;
}

void CS_ROV::depth(double dt)
{
    X[73][0] = i2c->pressure();
    if (X[73][0] < 8000)
    {return;}
    if (X[73][0] > 15000)
    {return;}
    X[280][0] = X[73][0];
    X[77][0] = X[280][0];
    X[90][0] = saturationPS(X[77][0], X[77][1], K[73]); // K==800 по расчетам, возможно можно убрать
    aperiodicFilter(X[90][0],X[78][0],X[78][1],1,K[207],dt); //K[207] = 0.1
    X[278][0] = (X[78][0]-10240)/(88.6*9.80665); //88.6 плотность
    static std::deque<double> window;
    if (window.size() == 30) {
        window.pop_front();
    }
    window.push_back(X[278][0]);
    std::deque<double> tmp_window(window.begin(), window.end());
    std::sort(tmp_window.begin(), tmp_window.end());
    X[279][0] = tmp_window[tmp_window.size() / 2];
}

void CS_ROV::aperiodicFilter(double &input, double &output, double &prevOutput, double K, double T, double dt)
{
    if (T !=0) output = prevOutput + dt*(1/T)*(input*K-prevOutput);
    else output = K*input;
    prevOutput = output;
}

void CS_ROV::writeDataToVMA()
{
      vmaProtocol->setValues(X[201][0], X[211][0], X[221][0], X[231][0], X[241][0], X[251][0], X[261][0], X[271][0], X[281][0], X[291][0], X[401][0], X[411][0]);
}
