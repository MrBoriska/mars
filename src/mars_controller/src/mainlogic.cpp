#include "mars_controller/mainlogic.h"

MainLogic::MainLogic(QObject *parent) : QObject(parent)
{
    modelThread = 0;
    modelWorker = 0;

    modelConfig = ModelConfig::Instance();
    cs_service = new ControlSysService();

    connect(cs_service, SIGNAL(cs_start()),
            this,       SLOT(cs_start()));
    connect(cs_service, SIGNAL(cs_pause()),
            this,       SLOT(cs_pause()));
    connect(cs_service, SIGNAL(cs_stop()),
            this,       SLOT(cs_stop()));

    connect(cs_service, SIGNAL(cs_set_track_path(QPainterPath)),
            this,       SLOT(cs_set_track_path(QPainterPath)));
    connect(cs_service, SIGNAL(cs_set_start_pos(GroupPos)),
            this,       SLOT(cs_set_start_pos(GroupPos)));
    connect(cs_service, SIGNAL(cs_get_pos_request()),
            this,       SLOT(cs_get_pos_request()));

    cs_service->init();
}

void MainLogic::cs_simulateStarted(){
    cs_service->send_started_event();
}

void MainLogic::cs_simulateStopped()
{
    //modelThread = 0;
    //modelWorker = 0;
}

void MainLogic::cs_start() {
    //modelThread = new QThread;

    if (modelWorker == 0) {
        modelWorker = new ModelWorker(modelConfig);
        //modelWorker->moveToThread(modelThread);

        //connect(modelThread, SIGNAL(finished()), modelWorker, SLOT(deleteLater()));
        //connect(modelThread, SIGNAL(finished()), modelThread, SLOT(deleteLater()));
        //connect(modelWorker, SIGNAL(simulateStopped()), modelThread, SLOT(quit()));
        connect(modelWorker, SIGNAL(simulateStopped()), this, SLOT(cs_simulateStopped()));
        //QObject::connect(modelWorker, SIGNAL(simulateFinished()), this, SLOT(showFinishSimulation()));
        //connect(modelThread, SIGNAL(started()), modelWorker, SLOT(simulate()));

        connect(modelWorker, SIGNAL(simulateStarted()), this, SLOT(cs_simulateStarted()));
        connect(cs_service, SIGNAL(cs_pause()),
                modelWorker,SLOT(pause_trigger()));

        connect(cs_service, SIGNAL(cs_stop()),
                modelWorker,SLOT(stop_simulate()));
    }

    //modelThread->start();
    modelWorker->simulate();
}
void MainLogic::cs_pause(){
    //modelWorker->pause_trigger();
    cs_service->send_paused_event();
}
void MainLogic::cs_stop(){
    cs_service->send_stopped_event();
}
void MainLogic::cs_set_track_path(QPainterPath tpath) {
    modelConfig->setTrackPath(tpath);
}
void MainLogic::cs_set_start_pos(GroupPos gpos) {
    modelConfig->setStartPosition(gpos);
}
void MainLogic::cs_get_pos_request() {

    if (modelWorker != 0) {
        cs_service->send_positions(
            modelWorker->getCurrentPos(),
            modelWorker->getCurrentTime()
        );
    } else {
        cs_service->send_positions(
            modelConfig->getStartPosition(),
            0
        );
    }
}