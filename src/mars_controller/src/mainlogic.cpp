#include "mars_controller/mainlogic.h"

MainLogic::MainLogic(int argc, char** argv, QObject *parent)
: QObject(parent)
{
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

    // Запуск узла ROS
    qnode = new QNode(argc,argv);
    // связываем завершение QApplication потока с завершением потока узла ROS
    connect(qnode, SIGNAL(rosShutdown()), parent, SLOT(quit()));

    // Запуск сервера для связи с GUI
    cs_service->init();
}

// Обработка внутреннего сигнала о том, что модель запущена
void MainLogic::cs_simulateStarted(){
    // отправка сообщения пользователю
    cs_service->send_started_event();
}

// Обработка внутреннего сигнала о том, что модель остановлена
void MainLogic::cs_simulateStopped() {
    // отправка сообщения пользователю
    cs_service->send_stopped_event();
}

// Обработчик запроса на запуск
void MainLogic::cs_start() {
    qDebug() << "MainLogic recieved start event";
    // Create or update model worker process
    if (modelWorker == nullptr) {
        // init communication to ROS
        qnode->init();

        // create Model handler
        modelWorker = new ModelWorker();

        connect(modelWorker, SIGNAL(simulateStopped()),
                this, SLOT(cs_simulateStopped()));
        connect(modelWorker, SIGNAL(simulateStarted()),
                this, SLOT(cs_simulateStarted()));
        connect(modelWorker, SIGNAL(newRobotGoal(int, double, double, QVector3D, long int)),
                qnode, SLOT(sendGoal(int, double, double, QVector3D, long int)));
        connect(cs_service, SIGNAL(cs_pause()),
                modelWorker,SLOT(pause_trigger()));
        connect(cs_service, SIGNAL(cs_stop()),
                modelWorker,SLOT(stop_simulate()));
        connect(qnode, SIGNAL(rosShutdown()),
                modelWorker, SLOT(stop_simulate()));
    } else {
        // update number of robots and e.t.c.
        qnode->init();
    }

    // Start predict control commands
    modelWorker->simulate();
}

// Обработчик запроса на паузу
void MainLogic::cs_pause() {
    qDebug() << "MainLogic recieved pause event";
    // Отправляем сообщение что приняли сообщение о паузе.
    // Приостановка модели происходит в другом классе
    cs_service->send_paused_event();
}

// Обработчик запроса на остановку
void MainLogic::cs_stop() {
    qDebug() << "MainLogic recieved stop event";
    // Отправляем сообщение что приняли сообщение о остановке.
    // Остановка модели происходит в другом классе
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
        qnode->setRealGroupPos(&grpos);
        
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
