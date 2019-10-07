/*
 *  controlsysservice.h
 */

#ifndef CONTROLSYSSERVICE_H
#define CONTROLSYSSERVICE_H

#include <QObject>
#include <QtWebSockets/qwebsocketserver.h>
#include <QtWebSockets/qwebsocket.h>
#include <QAbstractSocket>
#include <QDebug>

#include "modelconfig.h"

class ControlSysService : public QObject
{
    Q_OBJECT
public:
    explicit ControlSysService(QObject *parent = nullptr);
    ~ControlSysService();

    void init();

    // Методы для отправки информации
    void send_started_event();
    void send_paused_event();
    void send_stopped_event();
    void send_ready_event();
    void send_unready_event();
    void send_positions(GroupPos gpos, qint64 time);

signals:
    void cs_start();
    void cs_pause();
    void cs_stop();
    void cs_set_track_path(QPainterPath);
    void cs_set_start_pos(GroupPos);
    void cs_set_config_data(ModelConfig*);
    void cs_get_pos_request();


public slots:
    void newConnection();
    void readyRead(QString strReply);
    void client_disconnected();

private:
    QWebSocketServer *server;
    QWebSocket *cs_socket;

    // Обработчики входящих сообщений
    void handler_start_request();
    void handler_pause_request();
    void handler_stop_request();
    void handler_set_start_pos(QJsonObject msg);
    void handler_set_track_path(QJsonObject msg);
    void handler_get_pos();
    void handler_set_config_data(QJsonObject msg);

    // Функции для конвертации внутренник типов данных в JSON и обратно
    QJsonArray tpath_to_jsonArray(QPainterPath tpath);
    QPainterPath jsonArray_to_tpath(QJsonArray ja);
    QJsonObject gpos_to_jsonObject(GroupPos gpos);
    QJsonObject state_to_jsonObject(RobotState state);
    QJsonObject pos_to_jsonObject(ItemPos pos);
    QJsonObject vel_to_jsonObject(ItemVel vel);
    GroupPos jsonObject_to_gpos(QJsonObject jo);
    RobotState jsonObject_to_state(QJsonObject jo);
    ItemPos jsonObject_to_pos(QJsonObject jo);
    ItemVel jsonObject_to_vel(QJsonObject jo);
    QJsonObject material_to_jsonObject(ItemMaterial material);
    QJsonObject point_to_jsonObject(QPointF point);
    ItemMaterial jsonObject_to_material(QJsonObject jo);
    QPointF jsonObject_to_point(QJsonObject jpoint);
};

#endif // CONTROLSYSSERVICE_H
