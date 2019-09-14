#include "mars_controller/controlsysservice.h"

#include <QJsonDocument>
#include <QJsonObject>


ControlSysService::ControlSysService(QObject *parent) : QObject(parent)
{
    cs_socket = 0;
    server = 0;
}
ControlSysService::~ControlSysService()
{
    server->close();
}
void ControlSysService::init()
{
    server = new QWebSocketServer(QStringLiteral("Model demo server"),
                                  QWebSocketServer::NonSecureMode, this);

    if(!server->listen(QHostAddress::LocalHost, 9999))
    {
        qDebug() << "Server could not start";
    }
    else
    {
        qDebug() << "Server started!";

        // whenever a user connects, it will emit signal
        connect(server, SIGNAL(newConnection()),
                this,   SLOT(newConnection()));

        //connect(server, SIGNAL(closed()),
        //        this,   SLOT(closed()));
    }
}



void ControlSysService::newConnection()
{
    // Защита от второго подключения
    if (cs_socket != 0) {
        QWebSocket *socket = server->nextPendingConnection();
        socket->sendTextMessage("Client already connected\r\n");

        socket->close();

        return;
    }

    qDebug() << "connected";
    cs_socket = server->nextPendingConnection();
    //cs_socket = qobject_cast<QWebSocket *>(sender());
    connect(cs_socket, SIGNAL(disconnected()),
            this, SLOT(client_disconnected()));
    connect(cs_socket, SIGNAL(textMessageReceived(QString)),
             this,          SLOT(readyRead(QString)));

    //cs_socket->waitForReadyRead();
    QJsonObject json;
    json.insert("type","ping");
    cs_socket->sendTextMessage(QJsonDocument(json).toJson());
}

void ControlSysService::client_disconnected()
{
    qDebug() << "client disconnected";
    if (cs_socket) cs_socket->deleteLater();
    cs_socket = 0;
}

void ControlSysService::readyRead(QString strReply)
{
    //qDebug("readyRead: ");

    QString msg_type;

    // Read new message
    //QString strReply = (QString)(cs_socket->readAll());

    // Parse JSON
    QJsonParseError parse_error;
    QJsonDocument jsonResponse = QJsonDocument::fromJson(strReply.toUtf8(), &parse_error);

    // Validate and send validation error
    if (jsonResponse.isNull()) {
        QJsonObject error_msg;
        error_msg.insert("type","error");
        error_msg.insert("info",parse_error.errorString());

        cs_socket->sendTextMessage(QJsonDocument(error_msg).toJson());
        return;
    }

    // Using JSON
    QJsonObject jsonObject = jsonResponse.object();

    // Get type of message
    msg_type = jsonObject.value("type").toString();

    //qDebug() << jsonObject;

    // Call handlers
    if (msg_type == "ping")
    {
        qDebug() << "Pong!";
        QJsonObject pong_msg;
        pong_msg.insert("type","pong");
        cs_socket->sendTextMessage(QJsonDocument(pong_msg).toJson());
    }
    else if (msg_type == "pong") {}
    else if (msg_type == "error") {
        // todo
    }
    else if (msg_type == "start_request") handler_start_request();
    else if (msg_type == "pause_request") handler_pause_request();
    else if (msg_type == "stop_request") handler_stop_request();
    else if (msg_type == "set_start_pos") handler_set_start_pos(jsonObject);
    else if (msg_type == "set_track_path") handler_set_track_path(jsonObject);
    else if (msg_type == "set_config_data") handler_set_config_data(jsonObject);
    else if (msg_type == "get_pos") handler_get_pos();
    //else if (msg_type == "ready_event") handler_ready_event();
    //else if (msg_type == "unready_event") handler_unready_event();

    // Unknown message type
    else {
        QJsonObject error_msg;
        error_msg.insert("type","error");
        error_msg.insert("info","Unknown type (" + msg_type + ")");

        cs_socket->sendTextMessage(QJsonDocument(error_msg).toJson());
    }
}

/* ---------------------------------------------------------------
 * ЗАПУСК
 * ---------------------------------------------------------------
 */
void ControlSysService::send_started_event()
{
    QJsonObject msg;
    msg.insert("type","started_event");
    cs_socket->sendTextMessage(QJsonDocument(msg).toJson());
}
void ControlSysService::handler_start_request()
{
    emit cs_start();
    //emit cs_started_error("Unknown error");
}

/* ---------------------------------------------------------------
 * ПАУЗА
 * ---------------------------------------------------------------
 */
void ControlSysService::send_paused_event()
{
    QJsonObject msg;
    msg.insert("type","paused_event");
    cs_socket->sendTextMessage(QJsonDocument(msg).toJson());
}

void ControlSysService::handler_pause_request()
{
    emit cs_pause();
}

/* ---------------------------------------------------------------
 * ОСТАНОВКА
 * ---------------------------------------------------------------
 */
void ControlSysService::send_stopped_event()
{
    QJsonObject msg;
    msg.insert("type","stopped_event");
    cs_socket->sendTextMessage(QJsonDocument(msg).toJson());
}

void ControlSysService::handler_stop_request()
{
    emit cs_stop();
}



/* ---------------------------------------------------------------
 * Прием входных данных системы управления
 * ---------------------------------------------------------------
 */
void ControlSysService::handler_set_start_pos(QJsonObject msg)
{
    GroupPos start_pos = jsonObject_to_gpos(msg);
    emit cs_set_start_pos(start_pos);
}

void ControlSysService::handler_set_track_path(QJsonObject msg)
{
    QPainterPath tpath = jsonArray_to_tpath(msg.value("path").toArray());
    emit cs_set_track_path(tpath);
}

void ControlSysService::handler_set_config_data(QJsonObject msg)
{
    ModelConfig* config = ModelConfig::Instance();
    config->interval = msg.value("interval").toInt();
    config->step = msg.value("step").toDouble();
    config->vel_max = msg.value("vel_max").toDouble();

    QSize size;
    size.setWidth(msg.value("scene_size").toObject().value("width").toInt());
    size.setHeight(msg.value("scene_size").toObject().value("width").toInt());
    config->setSceneSize(size);
    config->sceneObject->setSceneRect(config->getSceneBorderWidth(),
                                       config->getSceneBorderWidth(),
                                       config->getSceneSize().width(),
                                       config->getSceneSize().height());
    config->materials.clear();
    QJsonArray materials = msg.value("materials").toArray();
    int length = materials.size();
    for(int i=0;i<length;i++) {
        QJsonObject jo = materials[i].toObject();
        config->materials.append(jsonObject_to_material(jo));
    }

    config->sceneObject->clear();
    QJsonArray map = msg.value("map").toArray();
    foreach(QJsonValue value, map) {
        QJsonObject jo = value.toObject();
        if (jo.value("type").toString() == "ground_polygon") {
            QPolygonF polygon;
            QJsonArray points = jo.value("points").toArray();
            int count = points.count();
            for(int i=0;i<count;i++) {
                polygon << jsonObject_to_point(points[i].toObject());
            }
            QGraphicsPolygonItem* polygon_item = new QGraphicsPolygonItem();
            polygon_item->setPolygon(polygon);
            polygon_item->setPos(jsonObject_to_point(jo.value("pos").toObject()));
            polygon_item->setBrush(QBrush(QColor(jo.value("material").toString())));

            config->sceneObject->addItem(polygon_item);
        }
    }

    emit cs_set_config_data(config);
}

// Вызывается в момент, когда СУ имеет достаточное количество входных данных, чтобы запуститься
void ControlSysService::send_ready_event()
{
    QJsonObject msg;
    msg.insert("type","ready_event");
    cs_socket->sendTextMessage(QJsonDocument(msg).toJson());
}
void ControlSysService::send_unready_event()
{
    QJsonObject msg;
    msg.insert("type","unready_event");
    cs_socket->sendTextMessage(QJsonDocument(msg).toJson());
}

/* ---------------------------------------------------------------
 * Функции для обработки запроса текущего положения роботов и груза
 * ---------------------------------------------------------------
 */
void ControlSysService::handler_get_pos()
{
    emit cs_get_pos_request();
}
void ControlSysService::send_positions(GroupPos gpos, qint64 time)
{
    QJsonObject msg = gpos_to_jsonObject(gpos);
    msg.insert("type","positions");
    msg.insert("time", time);
    cs_socket->sendTextMessage(QJsonDocument(msg).toJson());
}


/* ---------------------------------------------------------------
 * Функции для преобразования из внутренних типов в json и обратно
 * ---------------------------------------------------------------
 */
QJsonArray ControlSysService::tpath_to_jsonArray(QPainterPath tpath)
{
    int length = tpath.elementCount();
    QJsonArray ja;
    for(int i=0;i<length;i++) {
        QJsonObject jo;
        QPainterPath::Element el = tpath.elementAt(i);

        if (el.isLineTo())
            jo.insert("type", "LineTo");
        else {
            qDebug() << "Error! Unsupported element of track(QPainterPath)";
            //return QJsonArray();
        }

        QPointF point = el;
        QJsonObject jpoint;
        jpoint.insert("x", point.x());
        jpoint.insert("y", point.y());
        jo.insert("point", jpoint);

        ja.append(jo);
    }

    return ja;
}
QPainterPath ControlSysService::jsonArray_to_tpath(QJsonArray ja)
{
    QPainterPath tpath;
    int length = ja.size();
    for(int i=0;i<length;i++) {
        QJsonObject jo = ja[i].toObject();

        if (jo.value("type").toString() == "LineTo")
            tpath.lineTo(
                jo.value("point").toObject().value("x").toDouble(),
                jo.value("point").toObject().value("y").toDouble()
            );
        else {
            qDebug() << "Error! Unsupported element of track(QJsonArray)";
            // return QPainterPath();
        }
    }
    return tpath;
}

QJsonObject ControlSysService::gpos_to_jsonObject(GroupPos gpos)
{
    QJsonObject jo;
    jo.insert("object_pos", state_to_jsonObject(gpos.object_pos));
    QJsonArray ja;
    foreach(RobotState pos, gpos.robots_pos) {
        ja.append(state_to_jsonObject(pos));
    }
    jo.insert("robots_pos", ja);

    return jo;
}

QJsonObject ControlSysService::state_to_jsonObject(RobotState state)
{
    QJsonObject jo;
    jo.insert("pos", pos_to_jsonObject(state.pos));
    jo.insert("vel", vel_to_jsonObject(state.vel));
    jo.insert("pos_real", pos_to_jsonObject(state.pos_real));
    jo.insert("vel_real", vel_to_jsonObject(state.vel_real));
    return jo;
}

QJsonObject ControlSysService::material_to_jsonObject(ItemMaterial material)
{
    QJsonObject jo;
    jo.insert("title", material.title);
    jo.insert("color", material.color.name());

    jo.insert("accn_max", material.accn_max);
    jo.insert("acct_max", material.acct_max);

    return jo;
}

QJsonObject ControlSysService::point_to_jsonObject(QPointF point)
{
    QJsonObject jpoint;
    jpoint.insert("x", point.x());
    jpoint.insert("y", point.y());
    return jpoint;
}

QJsonObject ControlSysService::pos_to_jsonObject(ItemPos pos)
{
    QJsonObject jo;
    jo.insert("alfa", pos.alfa);
    jo.insert("x", pos.x);
    jo.insert("y", pos.y);
    return jo;
}

QJsonObject ControlSysService::vel_to_jsonObject(ItemVel vel)
{
    QJsonObject jo;
    jo.insert("x",vel.x);
    jo.insert("w",vel.w);
    return jo;
}

GroupPos ControlSysService::jsonObject_to_gpos(QJsonObject jo)
{
    GroupPos gpos;
    gpos.object_pos = jsonObject_to_state(jo.value("object_pos").toObject());
    gpos.robots_pos = QList<RobotState>();
    foreach(QJsonValue value, jo.value("robots_pos").toArray()) {
        gpos.robots_pos.append(jsonObject_to_state(value.toObject()));
    }
    return gpos;
}

RobotState ControlSysService::jsonObject_to_state(QJsonObject jo)
{
    RobotState state;
    state.pos = jsonObject_to_pos(jo.value("pos").toObject());
    state.vel = jsonObject_to_vel(jo.value("vel").toObject());
    state.pos_real = jsonObject_to_pos(jo.value("pos_real").toObject());
    state.vel_real = jsonObject_to_vel(jo.value("vel_real").toObject());
    return state;
}

ItemMaterial ControlSysService::jsonObject_to_material(QJsonObject jo)
{
    ItemMaterial material;
    material.title = jo.value("title").toString();
    material.color = QColor(jo.value("color").toString());

    material.accn_max = jo.value("accn_max").toDouble();
    material.acct_max = jo.value("acct_max").toDouble();

    return material;
}

QPointF ControlSysService::jsonObject_to_point(QJsonObject jpoint)
{
    return QPointF(
        jpoint.value("x").toDouble(),
        jpoint.value("y").toDouble()
    );
}

ItemPos ControlSysService::jsonObject_to_pos(QJsonObject jo)
{
    ItemPos pos;
    pos.alfa = jo.value("alfa").toDouble();
    pos.x = jo.value("x").toDouble();
    pos.y = jo.value("y").toDouble();
    pos.k = 0;
    return pos;
}

ItemVel ControlSysService::jsonObject_to_vel(QJsonObject jo)
{
    ItemVel vel;
    vel.x = jo.value("x").toDouble();
    vel.w = jo.value("w").toDouble();
    return vel;
}
