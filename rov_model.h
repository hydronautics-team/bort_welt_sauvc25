#ifndef ROV_MODEL_H
#define ROV_MODEL_H

#include <QObject>
#include <QVector>
#include <QTimer>

#define ANPA_MOD_CNT 24

extern double X[2000][2];
extern QVector<double> K;

struct InitData {
    double m;
    double Fa;
    double Farx[3];
    double cv1[4];
    double cv2[4];
    double cw1[4];
    double cw2[4];
    double lambda[6][6];
    double Ta[6][10];
    double J[3];
    double kd;
    double h[4]; //радиус-вектор координат центра водоизмещения НПА в связанной СК, [1]-x [2]-y [3]-z
    double Td;
    double depth_limit;
    double max_depth;
}; //struct InitData

class ROV_Model : public QObject {
    Q_OBJECT
public:
    explicit ROV_Model(QObject *parent = 0);
    virtual ~ROV_Model();
signals:

public slots:
private:
    void start();

public:
    void model(const float Upl,const float Upp,const float Ucl,const float Ucp, const float Uzlv,
               const float Uzpv, const float Uzln, const float Uzpn, const float Ulagz, const float Ulagp);
    void runge(const float Upl,const float Upp,const float Ucl,const float Ucp, const float Uzlv, const float Uzpv,
               const float Uzln, const float Uzpn, const float Ulagz, const float Ulagp,const float Ttimer,const float dt=0.01);

    double a[ANPA_MOD_CNT];
    double da[ANPA_MOD_CNT];
    //константы
    double m;
    double Fa;
    double Farx[3];
    double g;
    double G;
    double cv1[4];
    double cv2[4];
    double cw1[4];
    double cw2[4];
    double lambda[7][7];
    double Ta[7][9];
    double C[7][7];
    double Vt[7]; //влиянние течения
    double Wv[7]; //вектор силы моментов, вызванных внешними возмущениями
    double J[4];
    double kd;
    double h[4]; //радиус-вектор координат центра водоизмещения НПА в связанной СК [1]-x [2]-y [3]-z
    double Td;
    double depth_limit;
    double max_depth;
    //переменные
    double sumX, sumZ;
    double cur_depth, Wx, Wy, Wz;
    double Psi_g, Gamma_g, Tetta_g;

    double Psi_gi, W_Psi_g, W_Gamma_g, W_Tetta_g;
    int N;
    double deltaSx, deltaSz;

    double Ppl, Ppp, Psl, Psp, Pzlv, Pzpv, Pzln, Pzpn, Plagz, Plagp;
    double Ppl_x, Ppp_x, Psl_x, Psp_x, Pzlv_x, Pzpv_x, Pzln_x, Pzpn_x, Plagz_x, Plagp_x;
    double Ppl_y, Ppp_y, Psl_y, Psp_y, Pzlv_y, Pzpv_y, Pzln_y, Pzpn_y, Plagz_y, Plagp_y;
    double Ppl_z, Ppp_z, Psl_z, Psp_z, Pzlv_z, Pzpv_z, Pzln_z, Pzpn_z, Plagz_z, Plagp_z;
    double Upl, Upp, Usl, Usp, Uzlv, Uzpv, Uzln, Uzpn, Ulagz, Ulagp; //напряжения движителей
    double FloatageX, FloatageY, FloatageZ, Fdx, Fdy, Fdz, Fgx, Fgy, Fgz, Fcx, Fcy, Fcz;
    double Mdx, Mdy, Mdz, Mgx, Mgy, Mgz, Mcx, Mcy, Mcz;
    double Mpl_x, Mpp_x, Msl_x, Msp_x, Mzlv_x, Mzpv_x, Mzln_x, Mzpn_x, Mlagz_x, Mlagp_x;
    double Mpl_y, Mpp_y, Msl_y, Msp_y, Mzlv_y, Mzpv_y, Mzln_y, Mzpn_y, Mlagz_y, Mlagp_y;
    double Mpl_z, Mpp_z, Msl_z, Msp_z, Mzlv_z, Mzpv_z, Mzln_z, Mzpn_z, Mlagz_z, Mlagp_z;
    double Max,May,Maz; // моменты от силы Архимеда

    double x_global, y_global, z_global;
    double vx_local,  vy_local, vz_local;  //lineinye skorosti SPA v svyazannyh osyah
    double vx_global, vy_global, vz_global;

public:
    void resetModel();
    void tick(const float Upl,const float Upp,const float Ucl,const float Ucp, const float Uzlv, const float Uzpv,
              const float Uzln, const float Uzpn, const float Ulagz, const float Ulagp, const float Ttimer);
    float Fx,Fy,Fz; //total forces for XYZ-axis
    float Mx,My,Mz; //total moments for XYZ-axis
protected:
    QTimer timer;
};

#endif // ROV_MODEL_H
