#ifndef MODELCONFIG_H
#define MODELCONFIG_H

#include <QObject>
#include <QtCore>
#include <QPainter>

#include <QtGui>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsPolygonItem>

struct ItemPos {
  qreal x;
  qreal y;
  qreal alfa;
  qreal k; // коэфф. кривизны траектории
};

// Скорости робота
struct ItemVel {
    qreal x;
    qreal w;
};

// Вектор состояния обьекта переноса или робота
struct RobotState {
    ItemPos pos;
    ItemVel vel;
    ItemPos pos_real;
    ItemVel vel_real;
};

struct ItemMaterial {
    QColor color;
    QString title;
    qreal accn_max;
    qreal acct_max;
};

struct GroupPos {
    RobotState object_pos;
    QList<RobotState> robots_pos;
};

/**
 * @brief The ModelConfig class use Singleton pattern for saving config data
 */
class ModelConfig : public QObject
{
    Q_OBJECT
public:
    explicit ModelConfig(QObject *parent = 0);
    static ModelConfig* Instance();

    /**
     * @brief величина шага за один такт отработки модели
     */
    double step;

    /**
     * @brief временной интервал между тактами отработки модели(мс)
     */
    int interval;
    double target_realtime_factor;

    /**
     * @brief максимальная скорость робота(которую он стремится достичь)
     */
    double vel_max;

    /**
     * @brief системные ограничения на параметры
     */
    double step_max;
    double step_min;
    int interval_max;
    int interval_min;


    QGraphicsScene* sceneObject;

    void reset();

    void setStartPosition(GroupPos s_pos);
    GroupPos getStartPosition();
    int getRobotsNum();

    QList<ItemMaterial> materials;
    ItemMaterial defaultMaterial;

    QSize getSceneSize();
    void setSceneSize(QSize size);
    double getSceneBorderWidth();

    ItemMaterial getItemMaterialByColor(QColor color);

    void setTrackPath(QPainterPath tpath);
    QPainterPath getTrackPath();
private:
    static ModelConfig* _instance;

    QPainterPath track_path;
    GroupPos startPos;
    QSize sceneSize;
    double sceneBorderWidth;
};

#endif // MODELCONFIG_H
