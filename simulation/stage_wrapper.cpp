#include "stage_wrapper.h"

#include "stage.hh"

static inline Pose qtPose(const Stg::Pose &p) {
    return Pose(p.x, p.y, Radians(p.a));
}

static inline Speed qtVel(const Stg::Velocity &v) {
    return Speed(v.x, v.y, Radians(v.a));
}

static inline double absMin(double a, double b) {
    return std::fabs(a) < std::fabs(b) ? std::fabs(a) : std::fabs(b);
}

StageWrapper::StageWrapper(const WorldModel *model, QObject *parent) : QObject(parent), _world(0), _model(model) {
    if(!Stg::InitDone()) {
        int argc = 0;
        Stg::Init(&argc, 0);
    }
}

StageWrapper::~StageWrapper() {
    delete _world;
}

bool StageWrapper::load(const QString &fileName) {
    if(_world) delete _world;

    _world = new Stg::World(); /* new Stg::WorldGui(1024, 768, "stage"); */
    _world->Load(fileName.toStdString());
    _world->AddUpdateCallback(worldCb, this);

    _obstacles.clear();
    for(auto m : _world->GetAllModels()) {
        if(m->TokenStr().find("obstacle") != std::string::npos) {
            auto *pm = dynamic_cast<Stg::ModelPosition*>(m);
            _obstacles[pm->Token()] = StageObject(pm);
            if(!isStaticObject(pm)) {
                pm->AddCallback(Stg::Model::CB_UPDATE, poseCb, this);
                pm->Subscribe();
            }
        } else if(m->TokenStr().find("robot") != std::string::npos) {
            auto *pm = dynamic_cast<Stg::ModelPosition*>(m);
            if(pm) {
                _robot.model = pm;
                _robot.laser = dynamic_cast<Stg::ModelRanger*>(_world->GetModel(m->TokenStr() + ".laser"));
            }
        }
    }

    if(!_robot.model || !_robot.laser) return false;
    if(_robot.laser->GetSensors().size() != 1) return false;

    _robot.model->Subscribe();
    _robot.model->AddCallback(Stg::Model::CB_UPDATE, poseCb, this);
    _robot.laser->Subscribe();

    _updateInterval = (double)_world->sim_interval / 1e6;

    return true;
}

void StageWrapper::update() {
    _world->Update();
}

void StageWrapper::moveTo(const QString &id, const Pose &goal) {
    StageObject *obj = objectById(id);
    if(!obj) return;
    qDebug() << "object" << id << "is going from" << qtPose(obj->model->GetPose()) << "to" << goal;
    obj->goal.setGoalPose(goal);
}

void StageWrapper::setPose(const QString &id, const Pose &pose) {
    auto *obj = objectById(id);
    if(!obj) return;
    obj->goal.setGoalPose(pose);
    obj->model->SetPose({pose.x, pose.y, 0.0, pose.th.rad()});
}

void StageWrapper::setSpeed(const QString &id, const Speed &speed, int timeMs) {
    setSpeed(id, speed.x, speed.y, speed.th.rad(), timeMs);
}

void StageWrapper::setSpeed(const QString &id, double x, double y, double a, int timeMs) {
    StageObject *obj = objectById(id);
    if(!obj) return;
    obj->goal.setGoalTime(_world->SimTimeNow() + 1000 * timeMs);
    obj->model->SetStall(false);
    obj->model->SetSpeed(x, y, a);
}

void StageWrapper::setVisible(const QString &id, bool on) {
    auto *obj = objectById(id);
    if(!obj) return;
    obj->model->SetObstacleReturn(on);
    obj->model->SetRangerReturn(on ? 1.0 : -1.0);
}

void StageWrapper::poseUpdate(const QString &id) {
    StageObject *object = objectById(id);
    if(!object || !object->goal.active) return;

    if(object->goal.isTimeGoal()) {
        if(object->goal.time <= _world->SimTimeNow()) {
            object->goal.active = false;
            object->model->SetSpeed(0, 0, 0);
            emit goalReached(id);
        }
        return;
    }

    Stg::Pose p = object->model->GetPose();

    if(isGoalReached(object)) {
        if(std::fabs(p.a - object->goal.th) > 0.1) {
            object->model->SetSpeed(0, 0, Stg::normalize(object->goal.th - p.a) / _updateInterval);
        } else {
            object->model->SetSpeed(0, 0, 0);
            object->goal.active = false;
            emit goalReached(id);
        }
        return;
    }

    double dy = object->goal.y - p.y, dx = object->goal.x - p.x;

    switch(_model->object(id)->driveType()) {
    case WorldObject::Omni: {
        QTransform t;
        t.rotateRadians(-p.a);
        auto d = t.map(QPointF(dx, dy)); // transform deltas to robot coordinates
        double goalDir = std::atan2(d.y(), d.x());
        double vx = absMin(d.x(), object->model->velocity_bounds[0].max) * std::cos(goalDir);
        double vy = absMin(d.y(), object->model->velocity_bounds[1].max) * std::sin(goalDir);
        object->model->SetSpeed(vx, vy, 0);
        break;
    }
    case WorldObject::Diff: {
        double goalDir = std::atan2(dy, dx);
        if(std::fabs(p.a - goalDir) < 0.1) {
            double d = std::sqrt(dx * dx + dy * dy);
            object->model->SetSpeed(d / _updateInterval, 0, 0);
        } else {
            object->model->SetSpeed(0, 0, Stg::normalize(goalDir - p.a) / _updateInterval);
        }
        break;
    }
    default:
        object->model->SetSpeed(0, 0, 0);
        break;
    }
}

void StageWrapper::worldUpdate() {
    if(_robot.model->Stalled()) {
        emit objectStalled(_robot.model->Token(), qtPose(_robot.model->GetPose()));
    }

    StageWorldState state;
    state.time = _world->SimTimeNow() / (double)1e6;
    state.objects.push_back(createState(_robot.model, true));
    for(auto i : _obstacles) {
        Stg::ModelPosition *m = i.model;
        state.objects.push_back(createState(m));
        if(m->Stalled()) {
            emit objectStalled(m->Token(), qtPose(m->GetPose()));
        }
    }

    emit worldUpdated(state);
}

bool StageWrapper::isStaticObject(const Stg::ModelPosition *m) const {
    return fabs(m->velocity_bounds[0].max) < 1e-6;
}

bool StageWrapper::isGoalReached(const StageObject *object) const {
    double dx = object->goal.x - object->model->GetPose().x;
    double dy = object->goal.y - object->model->GetPose().y;
    return (std::fabs(dx) < 1.0 / _world->Resolution() && std::fabs(dy) < 1.0 / _world->Resolution());
}

StageObjectState StageWrapper::createState(const Stg::ModelPosition *m, bool scan) const {
    StageObjectState state;
    state.id = QString(m->Token());
    state.gtPose = qtPose(m->GetPose());
    state.estOrig = qtPose(m->est_origin);
    state.estPose = qtPose(m->est_pose);
    state.laserPose = Pose();
    state.speed = qtVel(m->GetVelocity());

    if(scan) {
        Stg::ModelRanger *rm = dynamic_cast<Stg::ModelRanger*>(m->GetChild("laser"));
        if(rm) {
            state.laserPose = qtPose(rm->GetPose());
            const auto &s = rm->GetSensors()[0];
            state.laserRanges.resize(s.sample_count);
            std::copy(s.ranges.begin(), s.ranges.end(), state.laserRanges.begin());
        }
    }

    return state;
}

StageWrapper::StageObject *StageWrapper::objectById(const QString &id) {
    if(id.startsWith("robot")) {
        return &_robot;
    } else {
        auto obj = _obstacles.find(id);
        if(obj != _obstacles.end()) return &obj.value();
    }
    return 0;
}

int StageWrapper::worldCb(Stg::World *world, void *data) {
    static_cast<StageWrapper*>(data)->worldUpdate();
    return 0;
}

int StageWrapper::poseCb(Stg::Model *model, void *data) {
    static_cast<StageWrapper*>(data)->poseUpdate(QString::fromStdString(model->TokenStr()));
    return 0;
}
