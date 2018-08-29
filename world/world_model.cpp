#include "world_model.h"
#include "path_planner.h"

#include <QPainter>
#include <QtMath>

template<class T>
static QList<QVariant> toVariantList(const QVector<T> &vec) {
    QList<QVariant> res;
    for(auto &i : vec) res.append(i);
    return res;
}

template<class T>
static QList<QVariant> toVariantList(const QList<T> &list) {
    return toVariantList<T>(list.toVector());
}

//=============================================================================

WorldObject::WorldObject(Type type, const QString &id)
    : _id(id), _type(type), _shape(Box),
      _motion(type == Robot ? Trajectory : Static),
      _drive(type == Robot ? Diff : Omni),
      _speedLin(0.5), _speedAng(90.0),
      _brush(type == Robot ? QColor(135, 206, 250) : Qt::blue)
{
    _waypoints.append({0, 0});
}

double WorldObject::angularSpeedRad() const {
    return qDegreesToRadians(_speedAng);
}

void WorldObject::setPose(const Pose &pose) {
    _waypoints[0] = pose;
}

int WorldObject::addWaypoint(const Pose &pose, int i) {
    if(i < 0) {
        _waypoints.append(pose);
        _paths.append(Path());
        return _waypoints.size() - 1;
    }

    _waypoints.insert(i, pose);
    _paths.insert(i, Path());
    return i;
}

void WorldObject::removeWaypoint(int i) {
    _waypoints.removeAt(i);

    if(i < _paths.size()) _paths.removeAt(i); // from current to next
    if(i > 0) _paths.removeAt(i - 1); //from prev to current
    if(i > 0 && i < _waypoints.size()) {
        _paths.insert(i - 1, Path());
    }
}

Path WorldObject::path() const {
    Path path;
    path.ok = true;
    for(auto &p : _paths) {
        path.append(p);
        path.ok &= p.ok;
    }
    return path;
}

void WorldObject::setPath(int i, const Path &path) {
    _paths[i] = path;
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

//=============================================================================

WorldModel::WorldModel(QObject *parent)
    : QObject(parent), _worldScale(1.0), _robotCount(0), _obstacleCount(0)
{
    _planner = new PathPlanner(this);
}

WorldModel::~WorldModel() {
    clear();
}

bool WorldModel::save(ProjectFile &project) const {
    auto *settings = project.settings();
    if(!settings) return false;

    if(!_map.isNull()) {
        // TODO: mapId should be unique
        QString mapId = "map";
        project.addPixmap(mapId, _map);
        settings->setValue("Map", mapId);
    } else {
        settings->remove("Map");
    }

    settings->setValue("WorldScale", _worldScale);
    settings->beginWriteArray("Objects", _objects.size());
    int i = 0;
    for(auto *obj : _objects) {
        settings->setArrayIndex(i++);
        if(!saveObject(obj, project)) return false;
    }
    settings->endArray();

    return true;
}

bool WorldModel::load(ProjectFile &project) {
    clear();

    if(!project.settings()) return false;
    auto &settings = *project.settings();

    auto mapFile = settings.value("Map").toString();
    if(mapFile.isEmpty()) return false;
    if(!setMap(project.getPixmap(mapFile))) return false;

    _worldScale = settings.value("WorldScale", 0.1).toDouble();

    int n = settings.beginReadArray("Objects");
    for(int i = 0; i < n; ++i) {
        settings.setArrayIndex(i);
        if(!loadObject(project)) {
            clear();
            return false;
        }
    }
    settings.endArray();

    return true;
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
    _map = QPixmap();

    emit cleared();
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
    return setMap(QPixmap(fileName));
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

void WorldModel::addObject(WorldObject::Type type, const QPointF &pos, const QString &id) {
    WorldObject *obj;

    auto objId = id.isEmpty() ? generateId(type) : id;
    if(objId.isEmpty()) return;

    switch(type) {
    case WorldObject::Robot:
        obj = new RobotObject(objId);
        break;
    case WorldObject::Obstacle:
        obj = new WorldObject(type, objId);
        obj->_size = {2.0, 2.0};
        obj->_origin = { obj->_size.width() / _worldScale / 2.0,
                         obj->_size.height() / _worldScale / 2.0 };
        break;
    default:
        return;
    }

    obj->setPose({pos.x(), pos.y(), Angle()});
    _objects[objId] = obj;

    emit objectCreated(obj);
}

void WorldModel::addWaypoint(WorldObject *object, const Pose &pose) {
    insertWaypoint(object, -1, pose);
}

void WorldModel::insertWaypoint(WorldObject *object, int i, const Pose &pose) {
    i = createWaypoint(object, pose, i);
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

void WorldModel::removeTrajectory(WorldObject *object) {
    for(int i = object->waypointCount() - 1; i > 0; --i) {
        emit waypointRemoved(object, i);
    }

    auto pose = object->pose();
    object->_waypoints.clear();
    object->_waypoints.append(pose);
    object->_paths.clear();
}

void WorldModel::updateAllTrajectories() {
    for(auto *obj : _objects) updateTrajectory(obj);
}

bool WorldModel::setMap(const QPixmap &pix) {
    if(pix.isNull()) return false;
    _map = pix;
    _planner->setMap(_map);
    emit mapChanged(_map);
    return true;
}

QString WorldModel::generateId(WorldObject::Type type) {
    while(true) {
        QString id;
        switch(type) {
        case WorldObject::Robot: id = QString("robot_%1").arg(_robotCount++); break;
        case WorldObject::Obstacle: id = QString("obstacle_%1").arg(_obstacleCount++); break;
        default: return QString();
        }

        if(!_objects.contains(id)) return id;
    }

    return QString();
}

int WorldModel::createWaypoint(WorldObject *object, const Pose &pose, int i) {
    i = object->addWaypoint(pose, i);
    emit waypointCreated(object, i);
    return i;
}

void WorldModel::updatePath(WorldObject *wo, int p1, int p2) {
    auto ps = wo->waypoint(p1);
    auto pg = wo->waypoint(p2);

    _planner->setRobotSize(wo->worldSize());
    auto plan = _planner->makePlan(ps, pg);
    bool ok = !plan.isEmpty();
    wo->setPath(p1, Path(plan, ok));

    if(!ok) {
        qDebug() << wo->id() << ": unable to find path from" << ps << "to" << pg;
    }

    emit pathChanged(wo, p1);
}

bool WorldModel::saveObject(const WorldObject *object, ProjectFile &project) const {
    auto &settings = *project.settings();

    settings.setValue("Id",           object->_id);
    settings.setValue("Type",         object->_type);
    settings.setValue("Motion",       object->_motion);
    settings.setValue("Drive",        object->_drive);
    settings.setValue("SpeedLinear",  object->_speedLin);
    settings.setValue("SpeedAngular", object->_speedAng);
    settings.setValue("Size",         object->_size);
    settings.setValue("Pose",         object->pose());
    settings.setValue("Origin",       object->_origin);
    settings.setValue("Brush",        object->_brush);
    settings.setValue("Shape",        object->_shape);
    settings.setValue("Waypoints",    toVariantList(object->_waypoints));

    if(object->_shape == WorldObject::CustomShape) {
        if(!project.addPixmap(object->_id, object->_shapePix)) return false;
        settings.setValue("ShapePix", object->_id);
    } else {
        settings.remove("ShapePix");
    }

    settings.beginWriteArray("Paths", object->_paths.size());
    int i = 0;
    for(auto &p : object->_paths) {
        settings.setArrayIndex(i++);
        settings.setValue("Path", toVariantList(p));
        settings.setValue("Ok", p.ok);
    }
    settings.endArray();

    switch(object->_type) {
    case WorldObject::Robot: {
        auto *robot = static_cast<const RobotObject*>(object);
        settings.setValue("OdomNoise", robot->_odomNoise);
        settings.setValue("OdomNoiseLinear", robot->_odomNoiseLin);
        settings.setValue("OdomNoiseAng", robot->_odomNoiseAng);
        break;
    }
    default:
        break;
    }

    return true;
}

bool WorldModel::loadObject(ProjectFile &project) {
    auto &settings = *project.settings();

    auto id = settings.value("Id").toString();
    addObject(static_cast<WorldObject::Type>(settings.value("Type", -1).toInt()),
              settings.value("Pose").value<Pose>().toPointF(), id);

    auto *object = _objects[id];
    if(!object) return false;

    setData(object, MotionRole,   settings.value("Motion"));
    setData(object, DriveRole,    settings.value("Drive"));
    setData(object, SpeedLinRole, settings.value("SpeedLinear"));
    setData(object, SpeedAngRole, settings.value("SpeedAngular"));
    setData(object, SizeRole,     settings.value("Size"));
    setData(object, OriginRole,   settings.value("Origin"));
    setData(object, BrushRole,    settings.value("Brush"));
    setData(object, ShapeRole,    settings.value("Shape"));

    if(object->shapeType() == WorldObject::CustomShape) {
        QPixmap pix = project.getPixmap(settings.value("ShapePix").toString());
        if(pix.isNull()) return false;
        setData(object, ShapePixmapRole, pix);
    }

    switch(object->type()) {
    case WorldObject::Robot: {
        setData(object, OdomNoiseRole,    settings.value("OdomNoise"));
        setData(object, OdomNoiseLinRole, settings.value("OdomNoiseLinear"));
        setData(object, OdomNoiseAngRole, settings.value("OdomNoiseAng"));
        break;
    }
    default:
        break;
    }

    auto wps = settings.value("Waypoints").toList();
    for(auto wp = wps.begin() + 1; wp != wps.end(); ++wp) {
        createWaypoint(object, wp->value<Pose>());
    }

    int pc = settings.beginReadArray("Paths");
    for(int i = 0; i < pc; ++i) {
        settings.setArrayIndex(i);
        Path path;
        path.ok = settings.value("Ok").toBool();
        for(auto &pi : settings.value("Path").toList()) {
            path.append(pi.value<Pose>());
        }

        object->setPath(i, path);
        emit pathChanged(object, i);
    }
    settings.endArray();

    return true;
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
