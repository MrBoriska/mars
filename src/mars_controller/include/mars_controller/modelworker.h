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
    ModelWorker(QObject *parent = nullptr);
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
    void newRobotGoal(int robot_id, double vx, double w, QVector3D goal_pos, long int rel_time);

private:
    QTimer *timer;
    
    // All configuration data for robots group
    ModelConfig *config;

    // tool for managing acces for threads
    QMutex mutex;

    // Current robots state (changing in timer with fixed time interval)
    GroupPos groupPos;
    long int current_index;
    long int current_time;

    // Main trajectory with all control information
    QList<GroupPos> trackMap;

    // Main procedure for all calculations (result in trackMap)
    void genTrackPoints();

    // Catmull-Rom splines calc
    QPointF catmullrom(long double t, double u, QPointF P0, QPointF P1, QPointF P2, QPointF P3);
    QVector2D catmullrom_speed(long double t, double u, QPointF P0, QPointF P1, QPointF P2, QPointF P3);
    QVector2D catmullrom_acc(long double t, double u, QPointF P0, QPointF P1, QPointF P2, QPointF P3);

    // Searching robot with maximum radius relative ICP
    double genRmax(QList<RobotState> robots, ItemPos object_pos);
    // Selecting maxim velositi for "Rmax robot" with restrictions by surface
    double genVmax(QList<RobotState> robots, ItemPos object_pos, double Rmax);
    // Calculation status vector (pos, vels, acc) for all robots
    QList<RobotState> getRobotsStates(GroupPos grSt, double Vmax, double Rmax);

    // Small tool for rotation 2d vector
    QVector2D rotate(QVector2D v, qreal a);
};

#endif // MODELWORKER_H
