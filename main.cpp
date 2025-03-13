#include <QCoreApplication>
#include "kx_pult/kx_protocol.h"
#include "kx_pult/qkx_coeffs.h"
#include "cs_rov.h"
#include <QTimer>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    Qkx_coeffs* kProtocol = new Qkx_coeffs(ConfigFile, KI);
    x_protocol* xProtocol = new x_protocol(ConfigFile, XI, X);
    CS_ROV cs_rov;
    return a.exec();
}
