#include "mars_controller/mainlogic.h"

MainLogic::MainLogic(int argc, char** argv, QObject *parent)
: QObject(parent), qnode(argc,argv)
{
    modelThread = nullptr;
    modelWorker = nullptr;

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

    connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(cs_stop()));

    cs_service->init();
}

void MainLogic::cs_simulateStarted(){
    cs_service->send_started_event();
}

void MainLogic::cs_simulateStopped() {}

void MainLogic::cs_start() {

    // Create or update model worker process
    if (modelWorker == nullptr) {
        // init communication to ROS
        (&qnode)->init(modelConfig->getRobotsNum());

        // create 
        modelWorker = new ModelWorker(modelConfig, &qnode);

        connect(modelWorker, SIGNAL(simulateStopped()),
                this, SLOT(cs_simulateStopped()));
        connect(modelWorker, SIGNAL(simulateStarted()),
                this, SLOT(cs_simulateStarted()));
        connect(cs_service, SIGNAL(cs_pause()),
                modelWorker,SLOT(pause_trigger()));
        connect(cs_service, SIGNAL(cs_stop()),
                modelWorker,SLOT(stop_simulate()));
    } else {
        // update number of robots and e.t.c.
        (&qnode)->update(modelConfig);
    }

    // Start predict control commands
    modelWorker->simulate();
}

void MainLogic::cs_pause(){
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

    if (modelWorker != nullptr) {
        GroupPos grpos = modelWorker->getCurrentPos();
        long int currtime = modelWorker->getCurrentTime();
        
        // send commands to robots
        (&qnode)->setRealGroupPos(&grpos);
        
        // send to MARS GUI
        cs_service->send_positions(
            grpos,
            currtime
        );
    } else {
        // send to MARS GUI
        cs_service->send_positions(
            modelConfig->getStartPosition(),
            0
        );
    }
}
