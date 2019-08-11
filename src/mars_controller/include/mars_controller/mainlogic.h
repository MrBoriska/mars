#ifndef MAINLOGIC_H
#define MAINLOGIC_H

#include <QObject>

#include "modelconfig.h"
#include "modelworker.h"
#include "controlsysservice.h"

class MainLogic : public QObject
{
    Q_OBJECT
public:
    explicit MainLogic(QObject *parent = 0);

    QThread *modelThread;
    ModelWorker *modelWorker;
    ModelConfig *modelConfig;
    ControlSysService *cs_service;

private slots:
    void cs_simulateStarted();
    void cs_simulateStopped();

public slots:
    void cs_start();
    void cs_pause();
    void cs_stop();
    void cs_set_track_path(QPainterPath tpath);
    void cs_set_start_pos(GroupPos gpos);
    void cs_get_pos_request();

};

#endif // MAINLOGIC_H
