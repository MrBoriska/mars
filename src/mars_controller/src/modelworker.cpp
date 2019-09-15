#include "mars_controller/modelworker.h"

#include <QDebug>
#include <QThread>
#include <QVector2D>

#include <cmath>

#include <QGraphicsItem>

ModelWorker::ModelWorker(ModelConfig *config_, QNode *qnode_, QObject *parent) : QObject(parent)
{
    timer = 0;
    config = config_;
    qnode = qnode_;

    started = false;
    current_time = 0;

    groupPos.robots_pos.clear();
    groupPos.object_pos.pos.x = 0.0;
    groupPos.object_pos.pos.y = 0.0;
    groupPos.object_pos.pos.alfa = 0.0;
    groupPos.object_pos.pos.k = 0.0;
    groupPos.object_pos.vel.x = 0.0;
    groupPos.object_pos.vel.w = 0.0;

}

ModelWorker::~ModelWorker()
{
    if (timer->isActive()) timer->stop();
    delete timer;
}

void ModelWorker::simulate()
{
    qDebug() << "simulate()";

    current_time = 0;
    this->groupPos = config->getStartPosition();

    // send to ROS robots control system
    int units_count = groupPos.robots_pos.size();
    for (int robot_id = 0; robot_id < units_count; robot_id++) {
        qnode->sendGoal(
            robot_id,
            // velocity
            0.0,
            0.0,
            // target position (for next step)
            QVector3D(
                groupPos.robots_pos[robot_id].pos.x,
                groupPos.robots_pos[robot_id].pos.y,
                groupPos.robots_pos[robot_id].pos.alfa
            ),
            // current time (relative to the start time)
            current_time
        );
    }
    
    genTrackPoints();

    if (timer == 0) {
        // TODO: need synchronize by ROS simulation time
        timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(simulateStep()));
    }
    timer->setInterval(config->interval/config->target_realtime_factor);
    timer->start();

    emit simulateStarted();
}

void ModelWorker::pause_trigger()
{
    // Блокируем доступ к памяти для других потоков
    //mutex.lock();

    if (timer->isActive()) {
        timer->stop();
    } else {
        //timer = new QTimer(this);
        //connect(timer, SIGNAL(timeout()), this, SLOT(simulateStep()));
        timer->setInterval(config->interval/config->target_realtime_factor);
        timer->start();
    }

    // Возвращаем доступ к памяти для других потоков
    //mutex.unlock();
}

void ModelWorker::stop_simulate()
{
    if (this->started && timer->isActive()) {
        timer->stop();
        mutex.lock();
        this->started = false;
        mutex.unlock();
    }

    // send to ROS robots control system
    int units_count = groupPos.robots_pos.size();
    for (int robot_id = 0; robot_id < units_count; robot_id++) {
        qnode->sendGoal(
            robot_id,
            0.0,
            0.0,
            QVector3D(
                groupPos.robots_pos[robot_id].pos.x,
                groupPos.robots_pos[robot_id].pos.y,
                groupPos.robots_pos[robot_id].pos.alfa
            ),
            current_time
        );
    }

    emit simulateStopped();
    qDebug() << "simulateStopped()";
}



void ModelWorker::simulateStep()
{
    // Завершение работы модели
    if (current_index > trackMap.count()-1) {
        timer->stop();
        mutex.lock();
        this->started = false;
        mutex.unlock();


        // send to ROS robots control system
        int units_count = groupPos.robots_pos.size();
        for (int robot_id = 0; robot_id < units_count; robot_id++) {
            qnode->sendGoal(
                robot_id,
                0.0,
                0.0,
                QVector3D(
                    groupPos.robots_pos[robot_id].pos.x,
                    groupPos.robots_pos[robot_id].pos.y,
                    groupPos.robots_pos[robot_id].pos.alfa
                ),
                current_time
            );
        }


        emit simulateFinished();
        qDebug() << "simulateFinished()";
        return;
    }

    // индикатор запуска модели
    if (!started) started = true;

    // Блокируем доступ к памяти для других потоков
    mutex.lock();
    
    GroupPos groupPos_start = config->getStartPosition();
    groupPos = trackMap.at(current_index);

    // send to ROS robots control system
    int units_count = groupPos.robots_pos.size();
    for (int robot_id = 0; robot_id < units_count; robot_id++) {
        qnode->sendGoal(
            robot_id,
            // velocity
            groupPos.robots_pos[robot_id].vel.x,
            groupPos.robots_pos[robot_id].vel.w,
            // target position (for next step)
            QVector3D(
                groupPos.robots_pos[robot_id].pos.x,
                groupPos.robots_pos[robot_id].pos.y,
                groupPos.robots_pos[robot_id].pos.alfa
            ) - QVector3D(
                groupPos_start.object_pos.pos.x,
                groupPos_start.object_pos.pos.y,
                0
            ),
            // current time (relative to the start time)
            current_time
        );
    }

    current_index++;
    current_time += config->interval/config->target_realtime_factor;

    // Возвращаем доступ к памяти для других потоков
    mutex.unlock();
}

QVector2D ModelWorker::rotate(QVector2D v, qreal a) {
    return QVector2D(
        v.x()*cos(a)-v.y()*sin(a),
        v.x()*sin(a)+v.y()*cos(a)
    );
}

QPointF ModelWorker::catmullrom(long double t, double u, QPointF P0, QPointF P1, QPointF P2, QPointF P3) {

    double t2 = t*t;
    double t3 = t2*t;

    double A = -u*t3+2.0*u*t2-u*t;
    double B = (2.0-u)*t3+(u-3.0)*t2+1.0;
    double C = (u-2.0)*t3+(3.0-2.0*u)*t2+u*t;
    double D = u*t3-u*t2;

    return A*P0+B*P1+C*P2+D*P3;
}

QVector2D ModelWorker::catmullrom_speed(long double t, double u, QPointF P0, QPointF P1, QPointF P2, QPointF P3) {
    double t2 = t*t;

    double A = -3.0*u*t2+4.0*u*t-u;
    double B = 2.0*t*(u-3.0)-3.0*t2*(u-2.0);
    double C = u-2*t*(2*u-3)+3.0*t2*(u-2.0);
    double D = 3.0*u*t2-2.0*u*t;

    return QVector2D(A*P0+B*P1+C*P2+D*P3);
}

QVector2D ModelWorker::catmullrom_acc(long double t, double u, QPointF P0, QPointF P1, QPointF P2, QPointF P3) {

    double A = 4.0*u-6.0*u*t;
    double B = -(6.0*t*(u-2.0)-2.0*u+6.0);
    double C = 6.0*t*(u-2.0)-4.0*u+6.0;
    double D = -(2.0*u-6.0*u*t);

    return QVector2D(A*P0+B*P1+C*P2+D*P3);
}

double ModelWorker::genRmax(QList<RobotState> robots, ItemPos object_pos)
{
    double Rmax = -1.0;
    if (object_pos.k != 0.0) {
        int units_count = robots.size();

        double R0 = -1.0/object_pos.k; // K имеет знак
        QVector2D R0_(
            0,
            R0
        );
        for (int k = 0; k < units_count; k++) {
            // Положение робота относительно обьекта управления(в с.к. обьекта)
            QVector2D OA = QVector2D(
                robots[k].pos.x-object_pos.x,
                robots[k].pos.y-object_pos.y
            );

            // Радиус-вектор поворота робота
            QVector2D R_ = R0_+OA;
            double R = R_.length();
            if (R > Rmax) {
                Rmax = R;
            }
        }
    } else {
        Rmax = INFINITY;
    }
    return Rmax;
}


/**
 * @brief ModelWorker::genVmax
 * @param robots
 * @param object_pos
 * @param Rmax - (cм) радиус до максимально удаленного от центра робота
 * @return Vmax (м/c)
 */
double ModelWorker::genVmax(QList<RobotState> robots, ItemPos object_pos, double Rmax)
{
    qreal dT = (double)(config->interval)/1000; // мс -> с
    Rmax /= 100; // cм -> м
    GroupPos grSt_start = config->getStartPosition();

    // Максимальная заданная скорость
    double V = config->vel_max;
    double Vmax = V;

    int rc = robots.count();

    for(int i=0;i<rc;i++) {

        // Find material object under robot
        QGraphicsPolygonItem* item = (QGraphicsPolygonItem *)(config->sceneObject->itemAt(
                                          robots[i].pos.x,
                                          robots[i].pos.y, QTransform()
                                      ));
        // todo!! this is BUG!
        if (!item) {
            return robots[i].vel.x;
        }
        
        // Get material information
        ItemMaterial mat = config->getItemMaterialByColor(item->brush().color());

        // qDebug() << mat.title << "for " << i;

        // По окружности
        if (object_pos.k != 0.0) {
            // Радиус-вектор поворота робота
            double R = (
                QVector2D(
                    0.0,
                    -1.0/object_pos.k
                )+QVector2D(
                    grSt_start.robots_pos[i].pos.x-grSt_start.object_pos.pos.x,
                    grSt_start.robots_pos[i].pos.y-grSt_start.object_pos.pos.y
                )
            ).length();

            R /= 100; // cм -> м

            // Исходя из ограничений для текущего робота, вычисляем максимальную скорость для Rmax робота
            // Учет ограничения на максимальное нормальное ускорение
            V = sqrt(mat.accn_max*R)*Rmax/R;
            if (V < Vmax) Vmax = V;
            // Учет ограничения на максимальное тангенциальное ускорение разгона
            V = double(robots[i].vel.x + mat.acct_max * dT)*Rmax/R;
            if (V < Vmax) Vmax = V;
            // торможения
            V = double(robots[i].vel.x - mat.acct_max * dT)*Rmax/R;
            if (V > Vmax) Vmax = V;
        
        // Прямолинейное
        } else {
            // Учет ограничения на максимальное тангенциальное ускорение разгона
            V = double(robots[i].vel.x + mat.acct_max * dT);
            if (V < Vmax) Vmax = V;
            // торможения
            V = double(robots[i].vel.x - mat.acct_max * dT);
            if (V > Vmax) Vmax = V;
        }
    }



    return Vmax;
}

QList<RobotState> ModelWorker::getRobotsStates(GroupPos grSt, double Vmax, double Rmax) {

    GroupPos grSt_start = config->getStartPosition();

    // Абсолютная позиция обьекта управления
    /*
     * /--------------> x
     * |
     * |    |-----|
     * |    |--O--|
     * |    |-----|
     * |
     *\ / y
     */
    QVector2D O(
        grSt.object_pos.pos.x,
        grSt.object_pos.pos.y
    );

    double K = grSt.object_pos.pos.k;
    double alfa = grSt.object_pos.pos.alfa;

    int units_count = grSt.robots_pos.size();
    // Поиск положений и скоростей роботов(для следующего шага)
    for (int k = 0; k < units_count; k++) {
        // Обьект для хранения абсолютного положения и ориентации робота
        ItemPos rpos;
        ItemVel rvel;
        RobotState rstate;

        // Положение робота относительно обьекта управления(в с.к. обьекта)
        QVector2D OA = QVector2D(
            grSt_start.robots_pos[k].pos.x-grSt_start.object_pos.pos.x,
            grSt_start.robots_pos[k].pos.y-grSt_start.object_pos.pos.y
        );


        // Криволинейное движение
        if (K != 0.0) {

            double R0 = -1.0/K;
            // Радиус-вектор поворота обьекта в с.к. обьекта
            /*
             * |-----|
             * |--O--|------------> x
             * |--|--|
             *    |
             *    |
             *    |
             *   \ / y
             */
            QVector2D R0_(
                0,
                R0
            );

            // Радиус-вектор поворота робота
            QVector2D R_ = R0_+OA;

            // Ориентация вектора скорости робота (в с.к. обьекта)
            QVector2D V = rotate(R_,(fabs(K)/K)*M_PI/2).normalized();
            // Ориентация робота соответственно (в с.к. обьекта)
            rpos.alfa = atan2(V.y(),V.x());

            // Кривизна траектории движения робота
            rpos.k = (fabs(K)/K)/R_.length();

            // Ориентация робота в с.к. карты
            rpos.alfa += alfa;

            rvel.x = Vmax*R_.length()/Rmax; // only > 0
            rvel.w = -(fabs(rpos.k)/rpos.k)*rvel.x/(R_.length()/100);

        } else {
            // Ориентация робота при прямолинейном движении неизменна
            rpos.alfa = grSt.robots_pos[k].pos.alfa;
            rpos.k = 0.0;

            rvel.x = Vmax;
            rvel.w = 0.0;
        }

        // Положение робота относительно обьекта управления(в с.к. карты)
        OA = rotate(OA, alfa);

        // Абсолютное положение робота
        QVector2D A = O + OA;

        rpos.x = A.x();
        rpos.y = A.y();
        rstate.pos = rpos;
        rstate.vel = rvel;
        grSt.robots_pos[k] = rstate;
    }

    return grSt.robots_pos;
}

// Generate object trajectory points
void ModelWorker::genTrackPoints()
{
    qDebug() << "genTrackPoints() // start";
    trackMap.clear();

    GroupPos grSt_start = config->getStartPosition();
    GroupPos grSt = grSt_start;

    QPainterPath track_path = config->getTrackPath();
    int length = track_path.elementCount();

    double Tension = 0.5;

    QPointF P0;
    QPointF P1(grSt.object_pos.pos.x, grSt.object_pos.pos.y);

    for(int i=1;i<length;i++) {
        // Вектор перехода текущий и следующий
        QVector2D vector = QVector2D(track_path.elementAt(i)-track_path.elementAt(i-1));
        QVector2D vector_next = vector;
        if (i+1 < length) {
            vector_next = QVector2D(track_path.elementAt(i+1)-track_path.elementAt(i));
        }

        if (i == 1) {
            P0 = P1-vector.toPointF();
        }
        QPointF P2 = P1+vector.toPointF();
        QPointF P3 = P2+vector_next.toPointF();


        double dist = vector.length();
        qint64 steps = qint64(dist/config->step);

        if (steps > 0) {
            int j = 0;
            while (j < steps) {
                long double p = (long double)j/steps;

                // Поиск Rmax
                double Rmax = genRmax(grSt.robots_pos, grSt.object_pos.pos);
                // Поиск скорости для Rmax робота(самого быстрого робота)
                double Vmax = genVmax(grSt.robots_pos, grSt.object_pos.pos, Rmax);

                // Кривизна траектории обьекта управления на текущем  шаге
                double K = grSt.object_pos.pos.k;
                double R0 = INFINITY;

                // Определение типа движения
                bool lin_mv = true;
                if (K != 0.0) {
                    lin_mv = false;
                    R0 = -1.0/K;
                }

                // Скорость обьекта управления на текущем шаге
                if (lin_mv) {
                    ItemVel vel;
                    vel.x = Vmax;
                    vel.w = 0.0;
                    grSt.object_pos.vel = vel;
                } else {
                    ItemVel vel;
                    vel.x = fabs(Vmax*R0/Rmax);
                    vel.w = vel.x/(R0/100);
                    grSt.object_pos.vel = vel;
                }

                // Расчет следующей позиции обьекта управления,
                // исходя из скорости движения и заданного интервала времени
                QPointF pos;
                QPointF dpos;
                double ds = 0.0;
                while (grSt.object_pos.vel.x && ds < grSt.object_pos.vel.x*config->interval/1000) {
                    p = (long double)j/steps;
                    if (p > 1) {
                        j--;
                        break;
                    }
                    pos = this->catmullrom(p,Tension,P0,P1,P2,P3);
                    dpos = pos - QPointF(grSt.object_pos.pos.x,grSt.object_pos.pos.y);
                    ds = QVector2D(dpos).length()/100;

                    j++;
                }

                if (p > 1) {
                    break;
                }

                //qDebug() << "K:" << K << "alfa0:" << alfa;


                // Расчет векторов состояний роботов
                grSt.robots_pos = getRobotsStates(grSt, Vmax, Rmax);


                // Позиция обьекта управления для следующего шага
                grSt.object_pos.pos.y = pos.y();
                grSt.object_pos.pos.x = pos.x();
                QVector2D sp = this->catmullrom_speed(p,Tension,P0,P1,P2,P3);
                QVector2D ac = this->catmullrom_acc(p,Tension,P0,P1,P2,P3);
                grSt.object_pos.pos.alfa = atan2(sp.y(),sp.x());
                grSt.object_pos.pos.k = (sp.x()*ac.y()-sp.y()*ac.x())/pow(sp.length(),3);

                trackMap.append(grSt);
            }
        }

        // Переход к следующему шагу интерполяции
        P0 = P1;
        P1 = P2;
    }

    current_index = 0;

    qDebug() << "genTrackPoints() // end";
}

void ModelWorker::deleteLater()
{
    qDebug() << "deleteLater()";
    QObject::deleteLater();
}


GroupPos ModelWorker::getCurrentPos() {
    return groupPos;
}

qint64 ModelWorker::getCurrentTime() {
    return current_time;
}
