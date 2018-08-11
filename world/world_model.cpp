#include "world_model.h"
#include "simulator.h"

#include <QPainter>
#include <QtMath>

#include <QDebug>

template<class T>
static QList<QVariant> toVariantList(const QVector<T> &vec) {
    QList<QVariant> res;
    for(auto &i : vec) res.append(i);
    return res;
}

Pose Pose::toDegrees() const {
    return Pose(x, y, qRadiansToDegrees(th));
}

//=============================================================================

WorldObject::WorldObject(Type type, const QString &id)
    : _id(id), _type(type), _shape(Box),
      _motion(type == Robot ? Trajectory : Static),
      _drive(type == Robot ? Diff : Omni),
      _speedLin(0.5), _speedAng(90.0),
      _brush(type == Robot ? QColor(135, 206, 250) : Qt::blue)
{
    _waypoints.append({0, 0, 0});
}

void WorldObject::save(QSettings &settings) const {
    settings.setValue("id", _id);
    settings.setValue("type", _type);
    settings.setValue("motion", _motion);
    settings.setValue("drive", _drive);
    settings.setValue("speedLinear", _speedLin);
    settings.setValue("speedAngular", _speedAng);
//    settings.setValue("pose", pose());
    settings.beginWriteArray("waypoints", _waypoints.size());
    int i = 0;
    for(auto wp : _waypoints) {
        settings.setArrayIndex(i++);
        settings.setValue("pose", wp.toVector3D());
    }
    settings.endArray();
    settings.beginWriteArray("paths", _paths.size());
//    i = 0;
//    for(auto &p : _paths) {
//        settings.setArrayIndex(i++);
//        settings.setValue("path", toVariantList(p));
//    }
    settings.endArray();
}

double WorldObject::angularSpeedRad() const {
    return qDegreesToRadians(_speedAng);
}

void WorldObject::setPose(const Pose &pose) {
    _waypoints[0] = pose;
}

int WorldObject::addWaypoint(const Pose &pose) {
    _waypoints.append(pose);
    return _waypoints.size() - 1;
}

void WorldObject::removeWaypoint(int i) {
    _waypoints.removeAt(i);

    if(i < _paths.size()) _paths.removeAt(i); // from current to next
    if(i > 0) _paths.removeAt(i - 1); //from prev to current
    if(i > 0 && i < _waypoints.size()) {
        _paths.insert(i - 1, Path());
    }
}

QVector<QVector3D> WorldObject::path() const {
    QVector<QVector3D> path;
    for(auto &p : _paths) path.append(p.toVector3D());
    return path;
}

void WorldObject::setPath(int i, const Path &path) {
    if(i == _paths.size()) _paths.append(path);
    else _paths[i] = path;
}

//=============================================================================

RobotObject::RobotObject(const QString &id)
    : WorldObject(Robot, id),
      _odomNoise(true), _odomNoiseLin(0.03), _odomNoiseAng(0.05)
{
    _speedLin = 1.5;
    _size = {0.35, 0.35};
    _origin = {0.0, 0.0};
}

void RobotObject::save(QSettings &settings) const {
    WorldObject::save(settings);
    settings.setValue("odomNoise", _odomNoise);
    settings.setValue("odomNoiseLinear", _odomNoiseLin);
    settings.setValue("odomNoiseAng", _odomNoiseAng);
}

//=============================================================================

WorldModel::WorldModel(QObject *parent)
    : QObject(parent), _worldScale(1.0), _robotCount(0), _obstacleCount(0)
{
    _planner = new PathPlanner(this);
}

WorldModel::~WorldModel() {
    clear();
}

QList<WorldObject*> WorldModel::objects() const {
    return _objects.values();
}

QList<WorldObject*> WorldModel::robots() const {
    QList<WorldObject*> rlist;
    for(auto *obj : _objects) {
        if(obj->type() == WorldObject::Robot) rlist.append(obj);
    }
    return rlist;
}

void WorldModel::clear() {
    _robotCount = _obstacleCount = 0;
    for(auto *obj : _objects) delete obj;
    _objects.clear();
}

void WorldModel::setData(WorldObject *object, DataRole role, const QVariant &value) {
    auto obj = _objects.find(object->id());
    if(obj == _objects.end()) return;

#define SET_DATA(member, value) { \
    auto val = value; \
    if(object->member == val) return; \
    object->member = val; \
    break; \
}

#define SET_ROBOT_DATA(member, value) { \
    if(object->type() != WorldObject::Robot) return; \
    auto robot = static_cast<RobotObject*>(object); \
    auto val = value; \
    if(robot->member == val) return; \
    robot->member = val; \
    break; \
}

    switch(role) {
    case SizeRole:   SET_DATA(_size, value.toSizeF());
    case OriginRole: SET_DATA(_origin, value.toPointF());
    case MotionRole: SET_DATA(_motion, static_cast<WorldObject::Motion>(value.toInt()));
    case DriveRole:  SET_DATA(_drive, static_cast<WorldObject::Drive>(value.toInt()));
    case ShapeRole:  SET_DATA(_shape, static_cast<WorldObject::Shape>(value.toInt()));
    case ShapePixmapRole:
        object->_shapePix = value.value<QPixmap>();
        break;
    case SpeedLinRole: SET_DATA(_speedLin, value.toDouble());
    case SpeedAngRole: SET_DATA(_speedAng, value.toDouble());
    case BrushRole:    SET_DATA(_brush, value.value<QBrush>());

    case OdomNoiseRole:    SET_ROBOT_DATA(_odomNoise, value.toBool());
    case OdomNoiseLinRole: SET_ROBOT_DATA(_odomNoiseLin, value.toDouble());
    case OdomNoiseAngRole: SET_ROBOT_DATA(_odomNoiseAng, value.toDouble());

    default: return;
    }

#undef SET_ROBOT_DATA
#undef SET_DATA

    emit dataChanged(object, role, value);
}

bool WorldModel::setMap(const QString &fileName) {
    _map = QPixmap(fileName);
    if(_map.isNull()) {
        _mapFileName.clear();
        return false;
    }
    _mapFileName = fileName;
    _planner->setMap(_map);
    emit mapChanged(_map);
    return true;
}

void WorldModel::setWorldScale(double val) {
    auto factor = val / _worldScale;
    for(auto *obj : _objects) {
        obj->setWorldSize(obj->worldSize() * factor);
    }
    _worldScale = val;
    updateAllTrajectories();
}

quint64 WorldModel::robotCount() const {
    return robots().size();
}

quint64 WorldModel::obstacleCount() const {
    return _objects.size() - robotCount();
}

void WorldModel::addObject(WorldObject::Type type, const QPointF &pos) {
    QString id;
    WorldObject *obj;

    switch(type) {
    case WorldObject::Robot:
        id = QString("robot_%1").arg(_robotCount++);
        obj = new RobotObject(id);
        break;
    case WorldObject::Obstacle:
        id = QString("obstacle_%1").arg(_obstacleCount++);
        obj = new WorldObject(type, id);
        obj->_size = {2.0, 2.0};
        obj->_origin = { obj->_size.width() / _worldScale / 2.0,
                         obj->_size.height() / _worldScale / 2.0 };
        break;
    default:
        return;
    }

    obj->setPose({pos.x(), pos.y(), 0});
    _objects[obj->id()] = obj;

    emit objectCreated(obj);
}

void WorldModel::addWaypoint(WorldObject *object, const Pose &pose) {
    int i = object->addWaypoint(pose);
    emit waypointCreated(object, i);

    updateWaypoint(object, i);
}

void WorldModel::setWaypoint(WorldObject *object, int i, const Pose &pose) {
    object->_waypoints[i] = pose;
}

void WorldModel::removeWaypoint(WorldObject *object, int i) {
    object->removeWaypoint(i);
    emit waypointRemoved(object, i);

    if(object->waypointCount() == 0) {
        removeObject(object);
    } else if(i > 0 && i < object->waypointCount()) {
        updatePath(object, i - 1, i);
    }
}

void WorldModel::updateWaypoint(WorldObject *object, int wpi) {
    if(wpi < 0 || wpi >= object->waypointCount()) return;
    if(wpi > 0) updatePath(object, wpi - 1, wpi);
    if(wpi < object->waypointCount() - 1) updatePath(object, wpi, wpi + 1);
}

void WorldModel::updateTrajectory(WorldObject *object) {
    if(object->motionType() != WorldObject::Trajectory) return;
    for(int i = 0; i < object->waypointCount() - 1; ++i) {
        updatePath(object, i, i + 1);
    }
}

void WorldModel::updateAllTrajectories() {
    for(auto *obj : _objects) updateTrajectory(obj);
}

void WorldModel::updatePath(WorldObject *wo, int p1, int p2) {
    auto ps = wo->waypoint(p1).toVector3D();//waypointPose(wo->waypoint(p1));
    auto pg = wo->waypoint(p2).toVector3D();//waypointPose(wo->waypoint(p2));

    _planner->setRobotSize(wo->worldSize());
    auto plan = _planner->makePlan(ps, pg);
    bool ok = !plan.isEmpty();
    wo->setPath(p1, Path(plan, ok));

    if(!ok) {
        qDebug() << wo->id() << ": unable to find path from" << ps << "to" << pg;
    }

    emit pathChanged(wo, p1);
}

void WorldModel::removeObject(WorldObject *object) {
    auto obj = _objects.find(object->id());
    if(obj == _objects.end()) return;

    _objects.erase(obj);
    delete object;

    emit objectRemoved(object);
}

WorldObject *WorldModel::object(const QString &id) {
    auto obj = _objects.find(id);
    return obj == _objects.end() ? 0 : obj.value();
}

const WorldObject *WorldModel::object(const QString &id) const {
    auto obj = _objects.find(id);
    return obj == _objects.end() ? 0 : obj.value();
}

QPixmap WorldModel::pixmap(const WorldObject *object) const {
    auto shape = object->type() == WorldObject::Robot ? WorldObject::Box
                                                      : object->shapeType();

    QRect rect = QRectF(QPointF(0, 0), object->worldSize() / _worldScale).toRect();

    QPixmap pix(rect.size());
    pix.fill(Qt::white);

    QPainter p(&pix);
    p.setBrush(object->brush());
    p.setPen(Qt::NoPen);
    switch(shape) {
    case WorldObject::Box:
        p.drawRect(rect);
        break;
    case WorldObject::Ellipse:
        p.drawEllipse(rect);
        break;
    case WorldObject::CustomShape:
        p.drawPixmap(rect, object->shapePixmap(), object->shapePixmap().rect());
        break;
    default:
        return pix;
    }

    p.end();
    return pix;
}
