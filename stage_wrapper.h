#ifndef STAGE_WRAPPER_H
#define STAGE_WRAPPER_H

#include <QObject>
#include <QString>
#include <QVector>
#include <QVector3D>
#include <QHash>

namespace Stg {
class World;
class Model;
class ModelPosition;
class ModelRanger;
}

class WorldModel;

struct StageObjectState {
    QString id;
    QVector3D gtPose;       // groundtruth pose
    QVector3D estOrig;      // pose estimation origin
    QVector3D estPose;      // estimated pose
    QVector3D laserPose;    // laser pose
    QVector3D speed;
    QVector<double> laserRanges;
};

struct StageWorldState {
    double time;
    QVector<StageObjectState> objects;
};

class StageWrapper : public QObject {
    Q_OBJECT

public:
    StageWrapper(const WorldModel *model, QObject *parent = 0);
    ~StageWrapper();

    bool load(const QString &fileName);
    void moveTo(const QString &id, const QVector3D &goal);
    void moveTo(const QString &id, double x, double y, double th);
    void setSpeed(const QString &id, const QVector3D &speed, int timeMs);
    void setSpeed(const QString &id, double x, double y, double a, int timeMs);

public slots:
    void update();

signals:
    void goalReached(const QString &id);
    void objectStalled(const QString &id, const QVector3D &pose);
    void worldUpdated(const StageWorldState &state);

private:
    struct Goal {
        Goal(double x = 0, double y = 0, double th = 0, bool active = false)
            : x(x), y(y), th(th), time(0), active(active) {}

        void setGoalPos(double x, double y, double t) {
            active = true;
            time = 0;
            this->x = x;
            this->y = y;
            this->th = t;
        }

        void setGoalTime(uint64_t time) {
            active = true;
            this->time = time;
        }

        bool isTimeGoal() const {
            return time > 0;
        }

        double x, y, th;
        quint64 time;
        bool active;
    };

    struct StageObject {
        StageObject(Stg::ModelPosition *m = 0) : model(m), laser(0) {}

        Stg::ModelPosition *model;
        Stg::ModelRanger *laser;
        Goal goal;
    };

    void worldUpdate();
    void poseUpdate(const QString &id);

    bool isStaticObject(const Stg::ModelPosition *m) const;
    bool isGoalReached(const StageObject *object) const;
    StageObjectState createState(const Stg::ModelPosition *m, bool scan = false) const;
    StageObject *objectById(const QString &id);

    static int worldCb(Stg::World *world, void *data);
    static int poseCb(Stg::Model *model, void *data);

    Stg::World *_world;
    const WorldModel *_model;

    StageObject _robot;
    QHash<QString, StageObject> _obstacles;
    double _updateInterval;
};

#endif // STAGE_WRAPPER_H
