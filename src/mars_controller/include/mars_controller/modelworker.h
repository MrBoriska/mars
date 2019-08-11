#ifndef MODELWORKER_H
#define MODELWORKER_H

#include <QTimer>
#include <QtCore>
#include <QTime>

#include "modelconfig.h"

class ModelWorker: public QObject
{
    Q_OBJECT

public:
    ModelWorker(ModelConfig *config, QObject *parent = 0);
    ~ModelWorker();

    GroupPos getCurrentPos();
    qint64 getCurrentTime();
    bool started;


public slots:
    void pause_trigger();
    void stop_simulate();
    void simulate();
private slots:
    void simulateStep();
    void deleteLater();


signals:
    void simulateStarted();
    void simulateFinished();
    void simulateStopped();

private:
    QTimer *timer;
    ModelConfig *config;
    QMutex mutex;
    GroupPos groupPos;
    long int current_time;

    QList<GroupPos> trackMap;
    long int current_index;

    void genTrackPoints();
    QPointF catmullrom(long double t, double u, QPointF P0, QPointF P1, QPointF P2, QPointF P3);
    QVector2D catmullrom_speed(long double t, double u, QPointF P0, QPointF P1, QPointF P2, QPointF P3);
    QVector2D catmullrom_acc(long double t, double u, QPointF P0, QPointF P1, QPointF P2, QPointF P3);

    QVector2D rotate(QVector2D v, qreal a);
    double genRmax(QList<RobotState> robots, ItemPos object_pos);
    double genVmax(QList<RobotState> robots, ItemPos object_pos, double Rmax);
    QList<RobotState> getRobotsStates(GroupPos grSt, double Vmax, double Rmax);
};

#endif // MODELWORKER_H
