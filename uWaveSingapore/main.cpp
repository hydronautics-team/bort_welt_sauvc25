#include <QCoreApplication>
#include "statemachine.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    StateMachine st;

    return a.exec();
}
