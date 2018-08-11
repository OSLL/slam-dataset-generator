#ifndef WORLD_MODEL_H
#define WORLD_MODEL_H

#include <QGraphicsObject>
#include <QVector3D>
#include <QBrush>
#include <QPen>
#include <QHash>
#include <QSettings>

// TODO:
//   - WorldObject* should be replaced with QModelIndex in functions like
//     addWaypoint, setWaypoint, setData, etc
//   - maybe WorldModel should inherit QAbstractItemModel
//   - all coordinates should be in a world scale, now only an object size
//     is stored in a world scale (for performance reasons)
//   - should object pixmaps be retrieved from WorldObjectItem?

class PathPlanner;

struct Pose {
    Pose() : x(0), y(0), th(0) {}
    Pose(double x, double y, double th) : x(x), y(y), th(th) {}

    double x, y, th;

    QPointF toPointF() const { return QPointF(x, y); }
    QVector3D toVector3D() const { return QVector3D(x, y, th); }

    Pose toDegrees() const;

    Pose &operator +=(const QPointF &point) {
        x += point.x();
        y += point.y();
        return *this;
    }
};

inline Pose operator+(const Pose &pose, const QPointF &point) {
    return Pose(pose.x + point.x(), pose.y + point.y(), pose.th);
}

struct Path : public QVector<Pose> {
    Path() : QVector<Pose>(), ok(false) {}
    Path(const QVector<Pose> &other, bool ok) : QVector<Pose>(other), ok(ok) {}
    Path(const QVector<QVector3D> &other, bool ok) : QVector<Pose>(other.size()), ok(ok) {
        int i = 0;
        for(auto &v : other) (*this)[i++] = {v.x(), v.y(), v.z()};
    }

    QVector<QVector3D> toVector3D() const {
        QVector<QVector3D> res;
        for(auto &p : *this) res.append(p.toVector3D());
        return res;
    }

    bool ok;
};

//=============================================================================

class WorldObject {
    friend class WorldModel;
public:
    enum Type   { Obstacle, Robot };
    enum Shape  { Box, Ellipse, CustomShape };
    enum Motion { Static, RandomSpeed, RandomPos, Trajectory };
    enum Drive  { Diff, Omni };

    WorldObject(Type type, const QString &id);
    virtual ~WorldObject() {}

    virtual void save(QSettings &settings) const;

    QString id() const { return _id; }
    Type    type() const { return _type; }
    Motion  motionType() const { return _motion; }
    Drive   driveType() const { return _drive; }
    Shape   shapeType() const { return _shape; }

    Pose    pose() const { return _waypoints.first(); }
    QPointF origin() const { return _origin; }
    QSizeF  worldSize() const { return _size; }

    const QPixmap &shapePixmap() const { return _shapePix; }
    bool hasShapePixmap() const { return !_shapePix.isNull(); }

    double linearSpeed() const { return _speedLin; }
    double angularSpeed() const { return _speedAng; }
    double angularSpeedRad() const;

    const QBrush &brush() const { return _brush; }

    QVector<QVector3D> path() const;
    const Path &path(int i) const { return _paths[i]; }
    bool hasPath() const { return !_paths.isEmpty(); }
    const QList<Path> &paths() const { return _paths; }

    Pose waypoint(int i) const { return _waypoints[i]; }
    const QList<Pose> &waypoints() const { return _waypoints; }
    int waypointCount() const { return _waypoints.size(); }

private:
    void setPose(const Pose &pose);
    int addWaypoint(const Pose &pose);
    void removeWaypoint(int i);

    void setWorldSize(const QSizeF &size) { _size = size; }
    void setPath(int i, const Path &path);

protected:
    QString _id;
    Type    _type;
    Shape   _shape;
    Motion  _motion;
    Drive   _drive;
    QSizeF  _size;
    QPointF _origin;
    double  _speedLin, _speedAng;
    QBrush  _brush;
    QPixmap _shapePix;

    QList<Pose> _waypoints;
    QList<Path> _paths;
};

//=============================================================================

class RobotObject : public WorldObject {
    friend class WorldModel;
public:
    RobotObject(const QString &id);

    void save(QSettings &settings) const;

    bool isOdomNoiseEnabled() const { return _odomNoise; }
    double odomNoiseLinear() const { return _odomNoiseLin; }
    double odomNoiseAngular() const { return _odomNoiseAng; }

private:
    bool _odomNoise;
    double _odomNoiseLin, _odomNoiseAng;
};

//=============================================================================

class WorldModel : public QObject {
    Q_OBJECT

public:
    enum DataRole { PoseRole, OriginRole, SizeRole, BrushRole,
                    MotionRole, DriveRole, ShapeRole, ShapePixmapRole,
                    SpeedAngRole, SpeedLinRole,
                    OdomNoiseRole, OdomNoiseLinRole, OdomNoiseAngRole };

    WorldModel(QObject *parent = 0);
    ~WorldModel();

    QList<WorldObject*> objects() const;
    QList<WorldObject*> robots() const;

    void clear();

    void setData(WorldObject *object, DataRole role, const QVariant &value);
//    QVariant data(WorldObject *object, DataRole role) const;

    bool setMap(const QString &fileName);
    QString mapFileName()  const { return _mapFileName; }
    QSizeF  mapWorldSize() const { return _map.size() * _worldScale; }
    QSize   mapSizePix()   const { return _map.size(); }

    double worldScale() const { return _worldScale; }
    void setWorldScale(double val);

    quint64 robotCount() const;
    quint64 obstacleCount() const;

    void addObject(WorldObject::Type type, const QPointF &pos);
    void removeObject(WorldObject *object);

    void addWaypoint(WorldObject *object, const Pose &pose);
    void setWaypoint(WorldObject *object, int i, const Pose &pose);
    void removeWaypoint(WorldObject *object, int i);
    void updateWaypoint(WorldObject *object, int wpi);

    void updateTrajectory(WorldObject *object);
    void updateAllTrajectories();

    WorldObject *object(const QString &id);
    const WorldObject *object(const QString &id) const;

    QPixmap pixmap(const WorldObject *object) const;

signals:
    void mapChanged(const QPixmap &map);
    void objectCreated(WorldObject *object);
    void objectRemoved(WorldObject *object);
    void waypointCreated(WorldObject *object, int i);
    void waypointRemoved(WorldObject *object, int i);
    void pathChanged(WorldObject *object, int i);
    void dataChanged(WorldObject *object, int role, const QVariant &value);

private:
    void updatePath(WorldObject *wo, int p1, int p2);

    double _worldScale;
    quint64 _robotCount, _obstacleCount;
    QHash<QString, WorldObject*> _objects;

    QPixmap _map;
    QString _mapFileName;
    PathPlanner *_planner;
};

#endif // WORLD_MODEL_H
