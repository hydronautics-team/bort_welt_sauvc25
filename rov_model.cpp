#include "qdebug.h"
#include <cmath>
#include <iostream>

ROV_Model::ROV_Model(QObject *parent) : QObject(parent)
{
    resetModel();

    m = 7;
    cv1[1] = 7.4; cv1[2] = 14.533; cv1[3] = 14.533;
    cv2[1] = 0.03; cv2[2] = 0.067; cv2[3] = 0.067;
    cw1[1] = 0.018; cw1[2] = 1.298; cw1[3] = 1.298; //
    cw2[1] = 0.000094; cw2[2] = 0.002; cw2[3] = 0.002;
    //Vt[1] = 1; Vt[2] = 1; Vt[3] = 1; Vt[4] = 0; Vt[5] = 0; Vt[6] = 0; // скорость течения
    //Wv[1] = 0; Wv[2] = 0; Wv[3] = 0; Wv[4] = 0; Wv[5] = 0; Wv[6] = 0; //внешние возмущения, лин. скорости([1]-[3], угловые скорости - [4]-[6])
    //h[1]= ; h[2]= ; h[3]= ; // радиус-вектор координат центра водоизмещения
    lambda[1][1] = 0.66; lambda[2][2] = 3.77; lambda[3][3] = 3.77;
    lambda[4][4] = 0; lambda[5][5] = 0; lambda[6][6] = 0.0185;
    Ta[1][1] = 0.866; Ta[1][2] = 0.866; Ta[1][3] = 0; Ta[1][4] = 0; Ta[1][5] = 0.866; Ta[1][6] = 0.866; Ta[1][7] = 0.866; Ta[1][8] = 0.866; Ta[1][9] = 0.866; Ta[1][10] = 0.866;
    Ta[2][1] = 0.5; Ta[2][2] = -0.5; Ta[2][3] = 0; Ta[2][4] = 0; Ta[2][5] = -0.5; Ta[2][6] = 0.5; Ta[2][7] = 0.866; Ta[2][8] = 0.866; Ta[2][9] = 0.866; Ta[2][10] = 0.866;
    Ta[3][1] = 0; Ta[3][2] = 0; Ta[3][3] = 1; Ta[3][4] = 1; Ta[3][5] = 0; Ta[3][6] = 0; Ta[3][7] = 0; Ta[3][8] = 0; Ta[3][9] = 0; Ta[3][10] = 0;
    Ta[4][1] = 0; Ta[4][2] = 0; Ta[4][3] = -110.9; Ta[4][4] = 110.9; Ta[4][5] = 0; Ta[4][6] = 0; Ta[4][7] = 0; Ta[4][8] = 0; Ta[4][9] = 0; Ta[4][10] = 0;
    Ta[5][1] = 0; Ta[5][2] = 0; Ta[5][3] = 0; Ta[5][4] = 0; Ta[5][5] = 0; Ta[5][6] = 0; Ta[5][7] = 0; Ta[5][8] = 0; Ta[5][9] = 0; Ta[5][10] = 0;
    Ta[6][1] = 184.93; Ta[6][2] = -184.93; Ta[6][3] = 0; Ta[6][4] = 0; Ta[6][5] = 213.96; Ta[6][6] = -213.96; Ta[6][7] = -213.96; Ta[6][8] = -213.96; Ta[6][9] = -213.96; Ta[6][10] = -213.96;
    //матрица сил и моментов инерции
    C[1][1] = 0; C[1][2] = (m+lambda[2][2])*a[20]; C[1][3] = -(m + lambda[3][3])*a[19]; C[1][4] = 0; C[1][5] = 0; C[1][6] = 0;
    C[2][1] = -(m + lambda[1][1])*a[20]; C[2][2] = 0; C[2][3] = (m + lambda[3][3])*a[18]; C[2][4] = 0; C[2][5] = 0; C[2][6] = 0;
    C[3][1] = (m + lambda[1][1])*a[19]; C[3][2] = -(m+lambda[2][2])*a[18]; C[3][3] = 0; C[3][4] = 0; C[3][5] = 0; C[3][6] = 0;
    C[4][1] = 0; C[4][2] = 0; C[4][3] = 0; C[4][4] = 0; C[4][5] = -(J[3]+lambda[6][6])*a[20]; C[4][6] = (J[2]+lambda[5][5])*a[19];
    C[5][1] = 0; C[5][2] = 0; C[5][3] = 0; C[5][4] = (J[3]+lambda[6][6])*a[20]; C[5][5] = 0; C[5][6] = -(J[1]+lambda[4][4])*a[18];
    C[6][1] = 0; C[6][2] = 0; C[6][3] = 0; C[6][4] = -(J[2]+lambda[5][5])*a[19]; C[6][5] = (J[1]+lambda[4][4])*a[18]; C[6][6] = 0;
    J[1] = 0.03; J[2] = 0.13; J[3] = 0.14; //моменты инерции вдоль соотв-х осей
    kd = 2; //коэффициент усиления движителей
    Td = 0.15; //постоянная времени движителей
    depth_limit=50;
    max_depth=50;
}

void ROV_Model::model(const float Upl,const float Upp,const float Ucl,const float Ucp, const float Uzlv,
                      const float Uzpv, const float Uzln, const float Uzpn, const float Ulagz, const float Ulagp) {
    int limit1, limit2;
    double G;

    //модули упоров движителей
    Ppl = a[7];  // передний левый(1)
    Ppp = a[8];  // передний правый(2)
    Psl = a[9];  // средний левый(3)
    Psp = a[10];  // средний правый(4)
    Pzlv = a[11];  // задний левый верхний(5)
    Pzpv = a[12];  // задний правый верхний(6)
    Pzln = a[13];  // задний левый нижний(7)
    Pzpn = a[14];  // задний правый нижний(8)
    Plagz = a[24]; //лаговый задний(9)
    Plagp = a[25]; //лаговый передний(10)

    //проекции упоров движителей на продольную ось апарата X
    Ppl_x = Ppl*Ta[1][1];
    Ppp_x = Ppp*Ta[1][2];
    Psl_x = Psl*Ta[1][3];
    Psp_x = Psp*Ta[1][4];
    Pzlv_x = Pzlv*Ta[1][5];
    Pzpv_x = Pzpv*Ta[1][6];
    Pzln_x = Pzln*Ta[1][7];
    Pzpn_x = Pzln*Ta[1][8];
    Plagz_x = Plagz*Ta[1][9];
    Plagp_x = Plagz*Ta[1][10];

    //проекции упоров движителей на продольную ось апарата Y
    Ppl_y = Ppl*Ta[2][1];
    Ppp_y = Ppp*Ta[2][2];
    Psl_y = Psl*Ta[2][3];
    Psp_y = Psp*Ta[2][4];
    Pzlv_y = Pzlv*Ta[2][5];
    Pzpv_y = Pzpv*Ta[2][6];
    Pzln_y = Pzln*Ta[2][7];
    Pzpn_y = Pzln*Ta[2][8];
    Plagz_y = Plagz*Ta[2][9];
    Plagp_y = Plagz*Ta[2][10];

    //проекции упоров движителей на продольную ось апарата Z
    Ppl_z = Ppl*Ta[3][1];
    Ppp_z = Ppp*Ta[3][2];
    Psl_z = Psl*Ta[3][3];
    Psp_z = Psp*Ta[3][4];
    Pzlv_z = Pzlv*Ta[3][5];
    Pzpv_z = Pzpv*Ta[3][6];
    Pzln_z = Pzln*Ta[3][7];
    Pzpn_z = Pzln*Ta[3][8];
    Plagz_z = Plagz*Ta[3][9];
    Plagp_z = Plagz*Ta[3][10];

    //момент создаваемый движетельным комплексом вокруг оси X
    Mpl_x = Ppl*Ta[4][1];
    Mpp_x = Ppp*Ta[4][2];
    Msl_x = Psl*Ta[4][3];
    Msp_x = Psp*Ta[4][4];
    Mzlv_x = Pzlv*Ta[4][5];
    Mzpv_x = Pzpv*Ta[4][6];
    Mzln_x = Pzln*Ta[4][7];
    Mzpn_x = Pzln*Ta[4][8];
    Mlagz_x = Plagz*Ta[4][9];
    Mlagp_x = Plagz*Ta[4][10];

    //момент создаваемый движетельным комплексом вокруг оси Y
    Mpl_y = Ppl*Ta[5][1];
    Mpp_y = Ppp*Ta[5][2];
    Msl_y = Psl*Ta[5][3];
    Msp_y = Psp*Ta[5][4];
    Mzlv_y = Pzlv*Ta[5][5];
    Mzpv_y = Pzpv*Ta[5][6];
    Mzln_y = Pzln*Ta[5][7];
    Mzpn_y = Pzln*Ta[5][8];
    Mlagz_y = Plagz*Ta[5][9];
    Mlagp_y = Plagz*Ta[5][10];

    //момент создаваемый движетельным комплексом вокруг оси Z
    Mpl_z = Ppl*Ta[6][1];
    Mpp_z = Ppp*Ta[6][2];
    Msl_z = Psl*Ta[6][3];
    Msp_z = Psp*Ta[6][4];
    Mzlv_z = Pzlv*Ta[6][5];
    Mzpv_z = Pzpv*Ta[6][6];
    Mzln_z = Pzln*Ta[6][7];
    Mzpn_z = Pzln*Ta[6][8];
    Mlagz_z = Plagz*Ta[6][9];
    Mlagp_z = Plagz*Ta[6][10];

    double g = 9.81;
    G = m*g; //вес аппарата
    Fa = G;
    Farx[0] = 0; Farx[1] = 0; Farx[2] = -Fa;

    Fdx = Ppl_x + Ppp_x + Psl_x + Psp_x + Pzlv_x + Pzpv_x + Pzln_x + Pzpn_x + Plagz_x + Plagp_x; // вектор сил и моментов, создаваемых движительным комплексом
    Fgx = -cv1[1] * a[1] * fabs(a[1]) - cv2[1] * a[1]; //произведение D1*Vx
    FloatageX = -sin(a[5]) * (G + Farx[2]);
    Fcx = C[1][1]*a[1] + C[1][2]*a[2]+C[1][3]*a[3]+C[1][4]*a[18]+C[1][5]*a[19] + C[1][6]*a[20];
    FloatageX = 0; //обнуление плавучести
    da[1] = (1/(m + lambda[1][1])) * (Fdx + Fgx + Fcx + FloatageX + Wv[1]); //vx'

    Fdy = Ppl_y + Ppp_y + Psl_y + Psp_y + Pzlv_y + Pzpv_y + Pzln_y + Pzpn_y + Plagz_y + Plagp_y; // вектор сил и моментов, создаваемых движительным комплексом
    Fgy = -cv1[2] * a[2] * fabs(a[2]) - cv2[2] * a[2]; //произведение D1*Vy
    FloatageY = cos(a[5]) * sin(a[4]) * (G + Farx[2]);
    Fcy = C[2][1]*a[1] + C[2][2]*a[2]+C[2][3]*a[3]+C[2][4]*a[18]+C[2][5]*a[19] + C[2][6]*a[20];
    FloatageY = 0; //обнуление плавучести
    da[2] = (1/(m + lambda[2][2])) * (Fdy + Fgy + Fcy + FloatageY + Wv[2]); //vy'

    Fdz = Ppl_z + Ppp_z + Psl_z + Psp_z + Pzlv_z + Pzpv_z + Pzln_z + Pzpn_z + Plagz_z + Plagp_z; // вектор сил и моментов, создаваемых движительным комплексом
    Fgz = -cv1[3] * a[3] * fabs(a[3]) - cv2[3] * a[3]; //произведение D1*Vz
    FloatageZ = cos(a[4]) * cos(a[5]) * (G + Farx[2]);
    Fcz = C[3][1]*a[1] + C[3][2]*a[2]+C[3][3]*a[3]+C[3][4]*a[18]+C[3][5]*a[19] + C[3][6]*a[20];
    FloatageZ = 0; //обнуление плавучести
    da[3] = (1/(m + lambda[3][3])) * (Fdz + Fgz + Fcz + FloatageZ + Wv[3]); //vz'

    da[4] = a[18] + (1/cos(a[5]) * ((a[19]) * sin(a[4]) * sin(a[5])  + sin(a[5]) * cos(a[4]) * a[20])) + Vt[4];  //производная крена

    da[5] = a[19] * cos(a[4]) - sin(a[4]) * a[20] + Vt[5];  //производная дифферента

    da[6] = (1/cos(a[5])) * (a[19] * sin(a[4]) + cos(a[4]) * (a[20])) + Vt[6]; //производная крена
 // Из матмодели имеем
 //K_двi - усредненный коэффициент усиления i-го движителя; T_двi=J_i/K_v1i  – наибольшее значение постоянной времени i-го ВМА
    da[7] = (1/Td) * (kd * (double)Upl - Ppl);  // передний левый(1)
    da[8] = (1/Td) * (kd * (double)Upp - Ppp);  // передний правый(2)
    da[9] = (1/Td) * (kd * (double)Usl - Psl);  // средний левый(3)
    da[10] = (1/Td) * (kd * (double)Usp - Psp); //средний правый(4)
    da[11] = (1/Td) * (kd * (double)Uzlv - Pzlv); // задний левый верхний(5)
    da[12] = (1/Td) * (kd * (double)Uzpv- Pzpv); // задний правый верхний(6)
    da[13] = (1/Td) * (kd * (double)Uzln - Pzln); // задний левый нижний(7)
    da[14] = (1/Td) * (kd * (double)Uzpn- Pzpn); // задний правый нижний(8)
    da[24] = (1/Td) * (kd * (double)Ulagz- Plagz); // лаговый задний(9)
    da[25] = (1/Td) * (kd * (double)Ulagp- Plagp); // лаговый передний(10)

    double alfa[4][4]; //матрица перевода из связанной СК в глобальную СК
    alfa[1][1] = cos(a[5])*cos(a[6]);
    alfa[2][1] = sin(a[6])*cos(a[5]);
    alfa[3][1] = -sin(a[5]);
    alfa[1][2] = cos(a[6])*sin(a[5])*sin(a[4])-cos(a[4])*sin(a[6]);
    alfa[2][2] = cos(a[6])*cos(a[4])+sin(a[4])*sin(a[5])*sin(a[6]);
    alfa[3][2] = sin(a[4])*cos(a[5]);
    alfa[1][3] = sin(a[6])*sin(a[4])+cos(a[6])*cos(a[4])*sin(a[5]);
    alfa[2][3] = sin(a[5])*sin(a[6])*cos(a[4])-cos(a[6])*sin(a[4]);
    alfa[3][3] = cos(a[5])*cos(a[4]);

    da[15] = alfa[1][1] * a[1] + alfa[1][2] * a[2] + alfa[1][3] * a[3] + Vt[1];
    //dx_global

    da[16] = alfa[2][1] * a[1] + alfa[2][2] * a[2] + alfa[2][3] * a[3] + Vt[2];
    //dy_global

    da[17] = alfa[3][1] * a[1] + alfa[3][2] * a[2] + alfa[3][3] * a[3] + Vt[3];
    //dz_global

    double Fax = -sin(a[5])*Fa;
    double Fay = sin(a[4])*cos(a[5])*Fa;
    double Faz = cos(a[5])*cos(a[4])*Fa;

    Mdx = Mpl_x + Mpp_x + Msl_x + Msp_x + Mzlv_x + Mzpv_x + Mzln_x + Mzpn_x + Mlagz_x + Mlagp_x;
    Mgx = -cw1[1] * a[18] * fabs(a[18]) - cw2[1] * a[18];
    Max = -h[2]*Faz + h[3]*Fay;
    //Max = 0; //обнуление момента от силы архимеды
    Mcx = C[4][1]*a[1] + C[4][2]*a[2]+C[4][3]*a[3]+C[4][4]*a[18]+C[4][5]*a[19] + C[4][6]*a[20];
    da[18] = (1/(J[1] + lambda[4][4])) * (Mdx + Mcx + Mgx + Max + Wv[4]);

    Mdy = Mpl_y + Mpp_y + Msl_y + Msp_y + Mzlv_y + Mzpv_y + Mzln_y + Mzpn_y + Mlagz_y + Mlagp_y;
    Mgy = -cw1[2] * a[19] * fabs(a[19]) - cw2[2] * a[19];
    May = -Faz*h[1] + Fax*h[3];
    //May = 0; //обнуление момента от силы архимеды
    Mcy = C[5][1]*a[1] + C[5][2]*a[2]+C[5][3]*a[3]+C[5][4]*a[18]+C[5][5]*a[19] + C[5][6]*a[20];
    da[19] = (1/(J[2] + lambda[5][5])) * (Mdy + Mcy + Mgy + May + Wv[5]);

    Mdz = Mpl_z + Mpp_z + Msl_z + Msp_z + Mzlv_z + Mzpv_z + Mzln_z + Mzpn_z + Mlagz_z + Mlagp_z;
    Mgz = -cw1[3] * a[20] * fabs(a[20]) - cw2[3] * a[20];
    Maz = -h[1]*Fay + h[2]*Fax;
    //Maz = 0; //обнуление момента от силы архимеды
    Mcz = C[6][1]*a[1] + C[6][2]*a[2]+C[6][3]*a[3]+C[6][4]*a[18]+C[6][5]*a[19] + C[6][6]*a[20];
    da[20] = (1/(J[3] + lambda[6][6])) * (Mdz + Mcz + Mgz + Maz + Wv[6]);

    da[21] = a[1];
    da[22] = a[2];
    da[23] = a[3];

}

void ROV_Model::resetModel(){
    for (int i=0;i<ANPA_MOD_CNT;i++) {a[i] = 0.0f; da[i]=0.0f;}
    for (int i=0; i<7;i++){
        Wv[i]=0;
        Vt[i]=0;
        h[i]=0;
    }
}

void ROV_Model::tick(const float Upl,const float Upp,const float Ucl,const float Ucp, const float Uzlv, const float Uzpv,
                     const float Uzln, const float Uzpn, const float Ulagz, const float Ulagp,const float Ttimer){

    runge(Upl, Upp, Ucl, Ucp, Uzlv, Uzpv, Uzln, Uzpn, Ulagz, Ulagp, Ttimer,Ttimer);
}

ROV_Model::~ROV_Model(){

}

void ROV_Model::runge(const float Upl,const float Upp,const float Ucl,const float Ucp, const float Uzlv, const float Uzpv,
                      const float Uzln, const float Uzpn, const float Ulagz, const float Ulagp, const float Ttimer, const float dt) {
    const double Kc = 180/M_PI;
    double a1[24], y[24];
    int i;
    const double H1 = dt;
    const int n = ANPA_MOD_CNT;
    model(Upl, Upp, Ucl, Ucp, Uzlv, Uzpv, Uzln, Uzpn, Ulagz, Ulagp);
    for (i = 1; i < n; i++) {
      a1[i] = a[i];
      y[i] = da[i];
      a[i] = a1[i] + 0.5 * H1 * da[i];
    }

    model(Upl, Upp, Ucl, Ucp, Uzlv, Uzpv, Uzln, Uzpn, Ulagz, Ulagp);
    for (i = 1; i < n; i++)
    {
      y[i] = y[i]+ 2 * da[i];
      a[i] = a1[i] + 0.5 * H1 * da[i];
    }

    model(Upl, Upp, Ucl, Ucp, Uzlv, Uzpv, Uzln, Uzpn, Ulagz, Ulagp);
    for (i = 1; i < n; i++) {
      y[i] = y[i] + 2 * da[i];
      a[i] = a1[i] + H1 * da[i];
    }

    model(Upl, Upp, Ucl, Ucp, Uzlv, Uzpv, Uzln, Uzpn, Ulagz, Ulagp);
    for (i = 1; i < n; i++) {
      a[i] = a1[i] + (H1 / 6) * (y[i] + da[i]);
    }

    //данные в СУ (с преобразованием координат)

    x_global = a[15]; //координата х в глобальной СК
    y_global = a[16];  //координата у в глобальной СК
    z_global = a[17]; //отстояние от дна относительно реперной точки, расположенной на дне
    cur_depth = max_depth + z_global;  //текущая глубина
    Wx = a[18] * Kc; //угловые скорости в связанной СК в градусах/секунду
    Wy = a[19] * Kc;
    Wz = a[20] * Kc;

    vx_local = a[1]; vy_local = a[2]; vz_local = a[3];  //линейные скорости в связанной СК
    vx_global = da[15]; vy_global = da[16]; vz_global = da[17];  // линейные скорости в глобальной СК

    Gamma_g = a[4] * Kc; // угол крена (градусы)
    Tetta_g = a[5] * Kc; // угол дифферента
    Psi_g = a[6] * Kc; // угол курса

    W_Gamma_g = da[4] * Kc; // производная угла крена
    W_Tetta_g = da[5] * Kc; // производная угла диффеннента
    W_Psi_g = da[6] * Kc; // производная угла курса

    X[10][0]=Wx;
    X[11][0]=Wy;
    X[12][0]=Wz;

    X[13][0]=vx_local;
    X[14][0]=vy_local;
    X[15][0]=vz_local;

    X[16][0]=Gamma_g; //крен
    X[17][0]=Tetta_g; //дифферент
    X[18][0]=Psi_g; //курс

    X[19][0]=x_global;
    X[20][0]=y_global;
    X[21][0]=z_global;

    X[22][0]=Ppl;
    X[23][0]=Ppp;
    X[24][0]=Psl;
    X[25][0]=Psp;
    X[26][0]=Pzlv;
    X[27][0]=Pzpv;
    X[28][0]=Pzln;
    X[29][0]=Pzpn;
    X[30][0]=Plagz;
    X[31][0]=Plagp;

    X[32][0]=vx_global;
    X[33][0]=vy_global;
    X[34][0]=vz_global;

}


