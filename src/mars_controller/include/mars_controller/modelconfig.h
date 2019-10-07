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
    explicit ModelConfig(QObject *parent = nullptr);
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
     * @brief Настройки регулятора движения по траектории
     */
    double trajectory_P;
    double trajectory_vP;
    double trajectory_w_thres_offset;
    double trajectory_w_thres;
    double trajectory_wI;

    /**
     * @brief Объект сцены (карты)
     */
    QGraphicsScene* sceneObject;
    void setSceneSize(QSize size);
    QSize getSceneSize();
    double getSceneBorderWidth();

    /**
     * @brief Возвращает количество роботов
     */
    int getRobotsNum();

    /**
     * @brief Устанавливает начальную позицию роботов
     */
    void setStartPosition(GroupPos s_pos);
    /**
     * @brief Возвращает начальную позицию роботов
     */
    GroupPos getStartPosition();


    /**
     * @brief Устанавливает начальный вид траектории движения
     */
    void setTrackPath(QPainterPath tpath);
    /**
     * @brief Возвращает начальный вид траектории движения
     */
    QPainterPath getTrackPath();

    
    /**
     * @brief Хранилище используемых материалов и их параметров
     */
    QList<ItemMaterial> materials;
    ItemMaterial defaultMaterial;
    
    /**
     * @brief Получает обьект с полной информацией о материле по его цвету на карте
     */
    ItemMaterial getItemMaterialByColor(QColor color);

private:
    static ModelConfig* _instance;

    QPainterPath track_path;
    GroupPos startPos;
    QSize sceneSize;
    double sceneBorderWidth;
};

#endif // MODELCONFIG_H
