#include "statemachine.h"

StateMachine::StateMachine(QObject *parent) : QObject(parent), timerCmd1(this),
    timerCmd2(this), timerSheerState(this),timerNavigation(this)
  , timerDirect(this)
{
    Json_parser js;
//    hydro = new Hydroacoustics(js.set.comIMU, 9600, nullptr); //возможно стоит вынести, что пытаемся пока не подключимся

    hydro = new Hydroacoustics("ttyAMA1", 9600, nullptr);
     connect(&timerCmd1,&QTimer::timeout,hydro,&Hydroacoustics::onPaket);
      connect(&timerCmd2,&QTimer::timeout,hydro,&Hydroacoustics::waitStolbik); ///для аппарата
//     connect(&timerCmd2,&QTimer::timeout,hydro,&Hydroacoustics::sendCmd2); // для берега
    connect(&timerDirect,&QTimer::timeout, hydro, &Hydroacoustics::modeDirect);

    SheerState = new QState();
    Navigation = new QState();
    EndState   = new QState();

    SendCmd1 = new QState(SheerState); //состояние внутри SheerState
    SendCmd2 = new QState(SheerState);


    SendCmd1->addTransition(hydro, &Hydroacoustics::initCmd1Done, SendCmd2);
    SendCmd2->addTransition(hydro, &Hydroacoustics::initCmd2Done, Navigation); // для берега
    Navigation->addTransition(this, &StateMachine::finishNavigation, EndState);

    SheerState->assignProperty(this,"m_state",statesMap.value(State::SheerState));
    Navigation->assignProperty(this,"m_state", "Navigation");
    EndState->assignProperty(this,"m_state","EndState");

    SendCmd1->assignProperty(this,"m_subState","SendCmd1");
    SendCmd2->assignProperty(this,"m_subState","SendCmd2");


    connect(SendCmd1,&QState::entered,this, [this](){ //входим в состояние SendCmd1 и запускаем таймер
        timerCmd1.start(2000);
    });
    connect(SendCmd1, &QState::exited,this, [this](){ //выходим из состояния SendCmd1 и выключаем таймер
        timerCmd1.stop();
        hydro->stopCounter();
    });
    connect(SendCmd2,&QState::entered,this, [this](){ //входим в состояние SendCmd1 и запускаем таймер
        timerCmd2.start(2000);
    });
    connect(SendCmd2, &QState::exited,this, [this](){ //выходим из состояния SendCmd1 и выключаем таймер
        timerCmd2.stop();
        hydro->stopCounter();
    });

    // connect(SheerState,&QState::entered,this,&StateMachine::initSheerStateSlot);
    connect(Navigation,&QState::entered,this,&StateMachine::initNavigationSlot);
    connect(EndState,&QState::entered,this,&StateMachine::initEndState);
    // connect(SheerState,&QState::exited,this,&StateMachine::finishSheerStateSlot);
    connect(Navigation,&QState::exited,this,&StateMachine::finishNavigationSlot);
    connect(EndState,&QState::exited,this,&StateMachine::finishEndState);




    machine.addState(SheerState);
    machine.addState(Navigation);
    machine.addState(EndState);
    machine.setInitialState(SheerState);
    SheerState->setInitialState(SendCmd2); // Назначаем начальное состояние SendCmd1 внутри SheerState

    machine.start(); //старт работы StateMachine

    // connect(hydro, &Hydroacoustics::updateData, this, &StateMachine::update);
}
void StateMachine::initSheerStateSlot()
{
//вызывается при вхождении в SheerState один раз

    // hydro->transferPaket(0,1,1001);
}

void StateMachine::initNavigationSlot()
{
//вызывается при вхождении в Navigation один раз
    int timeQuest = 500;
    qDebug() << "timeQuest: " << timeQuest;
    timerDirect.start(timeQuest);

}

void StateMachine::initEndState()
{

}

void StateMachine::finishEndState()
{

}

void StateMachine::finishSheerStateSlot()
{
//вызывается при выходе из SheerState один раз
}

void StateMachine::finishNavigationSlot()
{
    //вызывается при выходе из Navigation один раз
    hydro->stopCounter();
    timerDirect.stop();
}



void StateMachine::update(uWave uwave)
{
    // qDebug() << uwave.
}
