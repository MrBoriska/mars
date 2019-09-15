#include "mars_controller/modelconfig.h"

ModelConfig::ModelConfig(QObject *parent) : QObject(parent)
{
    step = 0.001;
    interval = 10;
    target_realtime_factor = 0.9;
    vel_max = 0.5; // m/s

    step_max = 3;
    step_min = 0.1;
    interval_max = 10;
    interval_min = 4;

    sceneSize = QSize(1000,700); // см х см
    sceneBorderWidth = 15;
    sceneObject = new QGraphicsScene();

    ItemMaterial mat;

    mat.color = QColor("#b5e3ea");
    mat.title = "Лёд";
    materials.append(mat);

    mat.color = QColor("#5a3d32");
    mat.title = "Земля";
    materials.append(mat);

    mat.color = QColor("#707f89");
    mat.title = "Асфальт";
    materials.append(mat);
    defaultMaterial = mat;

    mat.color = QColor("#f2e27f");
    mat.title = "Песок";
    materials.append(mat);
}

ModelConfig* ModelConfig::_instance = 0;
ModelConfig* ModelConfig::Instance() {
    if(_instance == 0) {
        _instance = new ModelConfig;
    }
    return _instance;
}

void ModelConfig::setTrackPath(QPainterPath tpath)
{
    track_path = tpath;
}

QPainterPath ModelConfig::getTrackPath()
{
    return track_path;
}

void ModelConfig::setStartPosition(GroupPos s_pos)
{
    startPos = s_pos;
}

GroupPos ModelConfig::getStartPosition()
{
    return startPos;
}

void ModelConfig::setSceneSize(QSize size)
{
    sceneSize = size;
}

double ModelConfig::getSceneBorderWidth()
{
    return sceneBorderWidth;
}

QSize ModelConfig::getSceneSize()
{
    return sceneSize;
}

int ModelConfig::getRobotsNum()
{
    return this->startPos.robots_pos.size();
}

ItemMaterial ModelConfig::getItemMaterialByColor(QColor color)
{
    foreach (ItemMaterial mat, materials) {
        if (mat.color == color) {
            return mat;
        }
    }
    return defaultMaterial;
}

void ModelConfig::reset()
{

}
