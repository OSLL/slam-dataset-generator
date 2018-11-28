#include "simulator.h"

#include <QRgb>
#include <QtMath>
#include <QLineF>
#include <QFileInfo>
#include <QFileDialog>
#include <QSignalMapper>
#include <QApplication>

#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include <QDebug>

static bool exportAsPgm(const QPixmap &pix, const QString &outputFileName) {
    auto img = pix.toImage().convertToFormat(QImage::Format_ARGB32);
    if(img.isNull()) return false;

    auto data = img.constBits();
    QByteArray rstData(img.byteCount() / 4, 0);
    for(int i = 0, j = 0; i < img.byteCount(); i += 4, ++j) {
        if(data[i + 3] < 127) rstData[j] = 255;
        else rstData[j] = qGray(data[i], data[i + 1], data[i + 2]);
    }

    QFile out(outputFileName);
    if(!out.open(QFile::WriteOnly)) return false;

    auto header = QString("P5\n%1 %2\n255\n").arg(pix.width()).arg(pix.height());
    out.write(header.toLatin1());
    out.write(rstData);

    return true;
}

//=============================================================================

BagWriter::BagWriter(QObject *parent) : QObject(parent), _open(false) {
    ros::Time::init();
}

BagWriter::~BagWriter() {
    close();
}

void BagWriter::setStartTime(const QDateTime &time) {
    _time = ros::Time().fromNSec(time.toMSecsSinceEpoch() * 1e6);
}

void BagWriter::setTfPrefix(const QString &prefix) {
    _tfPrefix = prefix;
    if(!prefix.startsWith('/')) _tfPrefix.prepend('/');
    if(!prefix.endsWith('/')) _tfPrefix.append('/');
}

bool BagWriter::open(const QString &fileName) {
    close();

    try {
        _bag.open(fileName.toStdString(), rosbag::bagmode::Write);
    } catch(rosbag::BagException &e) {
        qDebug() << "Unable to open output file: " << e.what();
        return false;
    }

    _time = ros::Time::now();
    _open = true;
    return true;
}

void BagWriter::close() {
    if(!_open) return;
    _bag.close();
    _open = false;
}

void BagWriter::writeLaserScan(double time, const LaserScan &scan) {
    auto ts = _time + ros::Duration().fromSec(time);

    sensor_msgs::LaserScan msg;
    msg.header.stamp = ts;
    msg.header.frame_id = addPrefix("base_laser_link");
    msg.angle_min = scan.minAngle;
    msg.angle_max = scan.maxAngle;
    msg.angle_increment = scan.angleIncrement;
    msg.range_min = scan.minRange;
    msg.range_max = scan.maxRange - 0.001;
    msg.ranges.reserve(scan.ranges.size());
    for(auto &r : scan.ranges) msg.ranges.push_back(r);

    _bag.write(addPrefix("base_scan", true), ts, msg);

    geometry_msgs::TransformStamped tr;
    tr.header.stamp = ts;
    tr.header.frame_id = addPrefix("base_link");
    tr.child_frame_id = addPrefix("base_laser_link");
    tr.transform = createTransform(scan.pose.x, scan.pose.y, scan.pose.th.rad());

    tf::tfMessage tfMsg;
    tfMsg.transforms.push_back(tr);

    _bag.write("/tf", ts, tfMsg);
}

void BagWriter::writeRobotPose(double time, const Pose &pose) {
    auto ts = _time + ros::Duration().fromSec(time);

    // frame_id -> child_frame_id
    geometry_msgs::TransformStamped tr1;
    tr1.header.stamp = ts;
    tr1.header.frame_id = addPrefix("base_footprint");
    tr1.child_frame_id = addPrefix("base_link");
    tr1.transform = createTransform(0, 0, 0);

    geometry_msgs::TransformStamped tr2;
    tr2.header.stamp = ts;
    tr2.header.frame_id = addPrefix("odom_combined");
    tr2.child_frame_id = addPrefix("base_footprint");
    tr2.transform = createTransform(pose.x, pose.y, pose.th.rad());

    tf::tfMessage msg;
    msg.transforms.push_back(tr1);
    msg.transforms.push_back(tr2);

    _bag.write("/tf", ts, msg);
}

geometry_msgs::Transform BagWriter::createTransform(double x, double y, double th) const {
    geometry_msgs::Transform t;
    t.translation.x = x;
    t.translation.y = y;
    t.translation.z = 0.0;
    t.rotation = tf::createQuaternionMsgFromYaw(th);
    return t;
}

std::string BagWriter::addPrefix(const QString &frame, bool topic) const {
    return QString("%1%2").arg((topic && _tfPrefix.isEmpty()) ? "/" : _tfPrefix)
                          .arg(frame).toStdString();
}

//=============================================================================

GroundTruthWriter::GroundTruthWriter(QObject *parent) : QObject(parent) {}

bool GroundTruthWriter::open(const QString &fileName) {
    close();

    _file.setFileName(fileName);
    if(!_file.open(QFile::WriteOnly)) return false;
    _ts.setDevice(&_file);
    return true;
}

void GroundTruthWriter::close() {
    _ts.setDevice(0);
    _file.close();
}

void GroundTruthWriter::writeRobotPose(double time, const Pose &pose) {
    auto t = _time + ros::Duration().fromSec(time);
    auto q = tf::createQuaternionFromYaw(pose.th.rad());

    _ts << t.sec << "." << t.nsec << " "
        << pose.x << " " << pose.y << " 0 "
        << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
        << "\n";
}

//=============================================================================

SimulatorConfig::SimulatorConfig(QWidget *parent) : QWidget(parent) {
    setupUi(this);
    startTimeTypeChanged();

    connect(_outputFile, SIGNAL(textChanged(QString)),
            this, SLOT(updateGtFileName(QString)));

    connect(_rbFixedTime, SIGNAL(toggled(bool)),
            this, SLOT(startTimeTypeChanged()));

    QSignalMapper *sm = new QSignalMapper(this);
    connect(sm, SIGNAL(mapped(int)), this, SLOT(updateOptionsFlags(int)));

#define ADD_DSB_MAPPING(name, flag) \
    sm->setMapping(name, flag); \
    connect(name, SIGNAL(valueChanged(double)), sm, SLOT(map()));

    ADD_DSB_MAPPING(_mapResolution, MapResolution);

#undef ADD_DSB_MAPPING

    connect(_pbOutputFile, SIGNAL(clicked(bool)), this, SLOT(selectOutputFile()));
    connect(_pbApply, SIGNAL(clicked(bool)), this, SLOT(applySettings()));
}

void SimulatorConfig::save(QSettings &settings) const {
    settings.setValue("OutputEnabled",      isOutputEnabled());
    settings.setValue("GTEnabled",          isGroundtruthEnabled());
    settings.setValue("VisEnabled",         isVisualizationEnabled());
    settings.setValue("LaserNoiseEnabled",  isLaserNoiseEnabled());
    settings.setValue("OutputFile",         outputFileName());
    settings.setValue("VisualizationRate",  visualizationRate());
    settings.setValue("MapResolution",      mapResolution());
    settings.setValue("SimInterval",        simInterval());
    settings.setValue("MaxAvoidanceTries",  maxAvoidTries());
    settings.setValue("LaserRangeMin",      laserRangeMin());
    settings.setValue("LaserRangeMax",      laserRangeMax());
    settings.setValue("LaserFOV",           laserFOV());
    settings.setValue("LaserSamples",       laserSamples());
    settings.setValue("LaserNoiseType",     laserNoiseType());
    settings.setValue("LaserNoiseC",        laserNoiseConst());
    settings.setValue("LaserNoiseP",        laserNoiseProp());
    settings.setValue("LaserNoiseAng",      laserNoiseAngular());
    settings.setValue("StartTimeFixed",     isStartTimeFixed());
    settings.setValue("StartTime",          startTime());
    settings.setValue("TfPrefix",           tfPrefix());
}

void SimulatorConfig::load(QSettings &settings) {
    setOutputFileName(settings.value("OutputFile").toString());

    for(auto *w : findChildren<QWidget*>()) w->blockSignals(true);
    _gbOutput->setChecked(settings.value("OutputEnabled", true).toBool());
    _cbGroundtruth->setChecked(settings.value("GTEnabled", false).toBool());
    _gbVisualization->setChecked(settings.value("VisEnabled", true).toBool());
    _laserNoise->setChecked(settings.value("LaserNoiseEnabled", true).toBool());
    _startTime->setDateTime(settings.value("StartTime",
                                           QDateTime::fromMSecsSinceEpoch(0)).toDateTime());
    if(settings.value("StartTimeFixed", false).toBool()) {
        _rbFixedTime->setChecked(true);
    } else {
        _rbCurrentTime->setChecked(true);
    }
    setTfPrefix(settings.value("TfPrefix").toString());
    setVisualizationRate(settings.value("VisualizationRate", 20).toInt());
    setMapResolution(settings.value("MapResolution", 0.1).toDouble());
    setSimInterval(settings.value("SimInterval", 100).toInt());
    setMaxAvoidTries(settings.value("MaxAvoidanceTries", 10).toInt());
    setLaserRangeMin(settings.value("LaserRangeMin", 0.0).toDouble());
    setLaserRangeMax(settings.value("LaserRangeMax", 30.0).toDouble());
    setLaserFOV(settings.value("LaserFOV", 270).toDouble());
    setLaserSamples(settings.value("LaserSamples", 1081).toInt());
    setLaserNoiseType(settings.value("LaserNoiseType", 0).toInt());
    setLaserNoiseConst(settings.value("LaserNoiseC", 0.01).toDouble());
    setLaserNoiseProp(settings.value("LaserNoiseP", 0.0).toDouble());
    setLaserNoiseAngular(settings.value("LaserNoiseAng", 0.05).toDouble());
    for(auto *w : findChildren<QWidget*>()) w->blockSignals(false);
}

void SimulatorConfig::setMapFileName(const QString &fileName) {
    QFileInfo fi(fileName);
    QString of = _outputFile->text();
    int p = of.lastIndexOf('/');
    of = (p == -1 ? "" : of.left(p + 1)) + fi.baseName() + ".bag";
    _outputFile->setText(of);
}

void SimulatorConfig::selectOutputFile() {
    auto of = QFileDialog::getSaveFileName(this, "Output file", _outputFile->text(), "Bag files (*.bag)");
    if(!of.isEmpty()) _outputFile->setText(of);
}

void SimulatorConfig::updateGtFileName(const QString &ofn) {
    if(!ofn.endsWith(".bag")) {
        _gtFileName.clear();
    } else {
        _gtFileName = ofn;
        _gtFileName.replace(ofn.size() - 3, 3, "gt");
    }
}

void SimulatorConfig::updateOptionsFlags(int flag) {
    _updatedOptions.setFlag(static_cast<Option>(flag));
    _pbApply->setEnabled(true);
}

void SimulatorConfig::applySettings() {
    auto opts = _updatedOptions;
    _updatedOptions = Options(0);
    _pbApply->setEnabled(false);
    emit settingsUpdated(opts);
}

void SimulatorConfig::startTimeTypeChanged() {
    if(_rbCurrentTime->isChecked()) {
        _startTime->setDateTime(QDateTime::currentDateTime());
    }
    _startTime->setEnabled(_rbFixedTime->isChecked());
}

//=============================================================================

PathTracker::PathTracker(QObject *parent)
    : QObject(parent), _curIdx(0), _ignoreNextPoint(false), _alignPath(false), _ignoreCount(0) {}

PathTracker::PathTracker(const Path &path, bool align, QObject *parent) : PathTracker(parent) {
    init(path, align);
}

void PathTracker::init(const Path &path, bool align) {
    _path = path;
    _alignPath = align;
    _totalLength = 0.0;
    for(int i = 0; i < path.size() - 1; ++i) {
        _totalLength += distance(path[i], path[i + 1]);
    }

    findWaypoints();
    reset();
}

void PathTracker::reset() {
    _curIdx = 0;
    _curChunkLength = _completedLength = 0.0;
    _ignoreNextPoint = false;
    _ignoreCount = 0;
    _currentPoint = _path.first();
    emit progress(0);
}

void PathTracker::revert() {
    std::reverse(_path.begin(), _path.end());
    findWaypoints();
    reset();
}

bool PathTracker::hasNextPoint() const {
    return _curIdx < _path.size() - (_ignoreNextPoint ? 0 : 1);
}

Pose PathTracker::nextPoint() {
    if(_ignoreNextPoint) {
        ++_ignoreCount;
        _ignoreNextPoint = false;
        return _currentPoint;
    }

    Pose p;

    if(!hasNextPoint()) return p;
    if(_waypointIdxs.contains(_curIdx + 1) || !_alignPath) {
        p = _path[_curIdx + 1];
    } else {
        p = alignedTo(_path[_curIdx + 1],
                _curIdx < _path.size() - 2 ? _path[_curIdx + 2] : _path.last());
    }

    _completedLength += _curChunkLength;
    _curChunkLength = distance(_path[_curIdx], _path[_curIdx + 1]);
    emit progress(qRound(_completedLength / _totalLength * 100.0));

    ++_curIdx;
    _ignoreCount = 0;
    _currentPoint = p;
    return p;
}

float PathTracker::updateProgress(const Pose &pos) {
    float p = 1;
    if(_curIdx < _path.size()) {
        p = (_completedLength + _curChunkLength - distance(pos, _path[_curIdx])) / _totalLength;
    }
    emit progress(qRound(p * 100.0));
    return p;
}

double PathTracker::distance(const Pose &p1, const Pose &p2) const {
    return QLineF(p1.toPointF(), p2.toPointF()).length();
}

Pose PathTracker::alignedTo(const Pose &p1, const Pose &p2) const {
    return Pose(p1.x, p1.y, Radians(qAtan2(p2.y - p1.y, p2.x - p1.x)));
}

void PathTracker::findWaypoints() {
    _waypointIdxs.clear();
    if(_path.isEmpty()) return;

    _waypointIdxs.append(0);
    int i = 0;
    for(auto pi = _path.constBegin(); pi != _path.constEnd() - 1; ++pi, ++i) {
        if(*pi == *(pi + 1)) _waypointIdxs.append(i);
    }
    _waypointIdxs.append(_path.size() - 1);
}

//=============================================================================

Simulator::ObjectState::ObjectState(WorldObject *object, const Pose &pose, const Path &path)
    : object(object), path(path.isEmpty() ? 0 : new PathTracker(path, true)),
      pose(pose), loops(0) {}

Simulator::ObjectState::~ObjectState() {
    delete path;
}

void Simulator::ObjectState::setPath(const Path &p, bool align) {
    if(path) path->init(p, align);
    else path = new PathTracker(p, align);
}

Simulator::Simulator(const WorldModel *model, QObject *parent)
    : QObject(parent), _stg(0), _bag(0), _gt(0), _model(model), _tmpDir(0)
{
    _mapResolution = 0.1;
    _maxAvoidTries = 10;

    _stgTimer = new QTimer(this);
    _stgTimer->setInterval(100);
}

Simulator::~Simulator() {
    delete _tmpDir;
}

void Simulator::simulate(const SimulatorConfig *config) {
    _objects.clear();
    bool robotFound = false;
    for(auto obj : _model->objects()) {
        auto &state = _objects[obj->id()];
        state = ObjectState(obj, mapToStg(obj->pose()));
        if(obj->motionType() == WorldObject::Trajectory) {
            state.setPath(obj->path(), obj->driveType() != WorldObject::Omni);
            state.loops = obj->trajectoryLoops() == 0 ? -1 : obj->trajectoryLoops();
            if(obj->type() == WorldObject::Robot) {
                robotFound = obj->hasPath();
                connect(state.path, SIGNAL(progress(int)), this, SIGNAL(simulationProgress(int)));
            }
        }
    }

    if(!robotFound) {
        qDebug() << "unable to find valid robot";
        return;
    }

    delete _tmpDir;
    _tmpDir = new QTemporaryDir();
    if(!_tmpDir->isValid()) {
        qDebug() << "unable to create temporary dir";
        return;
    }

    auto worldConfig = generateWorldFile(config);
    if(worldConfig.isEmpty()) {
        qDebug() << "unable to generate world file";
        return;
    }

    auto worldFilePath = tmpFilePath("config.world");

    QFile worldFile(worldFilePath);
    if(!worldFile.open(QFile::WriteOnly)) {
        qDebug() << "unable to write world file";
        return;
    }

    worldFile.write(worldConfig.toLatin1());
    worldFile.close();

    _finished = false;
    _visual = config->isVisualizationEnabled();
    _mapResolution = config->mapResolution();
    _maxAvoidTries = config->maxAvoidTries();

    _laserScan.minAngle = qDegreesToRadians(-config->laserFOV() / 2.0);
    _laserScan.maxAngle = qDegreesToRadians(+config->laserFOV() / 2.0);
    _laserScan.angleIncrement =
            (_laserScan.maxAngle - _laserScan.minAngle) / (float)(config->laserSamples() > 1 ? config->laserSamples() - 1 : 1);
    _laserScan.minRange = config->laserRangeMin() / _mapResolution;
    _laserScan.maxRange = config->laserRangeMax() / _mapResolution;

    delete _bag;
    delete _gt;
    _bag = 0;
    _gt = 0;

    if(config->isOutputEnabled()) {
        _bag = new BagWriter(this);
        if(!_bag->open(config->outputFileName())) {
            qDebug() << "unable to open output bag file";
            return;
        }

        if(config->isStartTimeFixed()) {
            _bag->setStartTime(config->startTime());
        }

        _bag->setTfPrefix(config->tfPrefix());

        if(config->isGroundtruthEnabled()) {
            _gt = new GroundTruthWriter(this);
            _gt->setStartTime(_bag->startTime());
            if(!_gt->open(config->gtFileName())) {
                qDebug() << "unable to open output gt file";
                return;
            }
        }
    }

    delete _stg;
    _stg = new StageWrapper(_model, this);
    connect(_stg, SIGNAL(goalReached(QString)), this, SLOT(goalReached(QString)));
    connect(_stg, SIGNAL(objectStalled(QString,Pose)), this, SLOT(objectStalled(QString,Pose)));
    connect(_stg, SIGNAL(worldUpdated(StageWorldState)), this, SLOT(worldUpdated(StageWorldState)));
    connect(_stgTimer, SIGNAL(timeout()), _stg, SLOT(update()));

    if(!_stg->load(worldFilePath)) {
        qDebug() << "worldfile is invalid";
        return;
    }

    emit simulationProgress(0);

    if(_visual) {
        _stgTimer->setInterval(1000 / config->visualizationRate());
        _stgTimer->start();
        for(auto &i : _objects) goalReached(i.object->id());
    } else {
        for(auto &i : _objects) goalReached(i.object->id());
        while(!_finished) {
            _stg->update();
            qApp->processEvents();
        }
    }
}

void Simulator::stop() {
    _stgTimer->stop();
    _finished = true;
}

QPointF Simulator::mapToStg(const QPointF &p) const {
    return QPointF(p.x() * _mapResolution, -p.y() * _mapResolution);
}

Pose Simulator::mapToStg(const Pose &p) const {
    return Pose(p.x * _mapResolution, -p.y * _mapResolution, -p.th);
}

Pose Simulator::mapFromStg(const Pose &p) const {
    return Pose(p.x / _mapResolution, -p.y / _mapResolution, -p.th);
}

Simulator::ObjectState *Simulator::stateById(const QString &id) {
    auto obj = _objects.find(id);
    return obj == _objects.end() ? 0 : &obj.value();
}

void Simulator::onPathComplete(ObjectState *state) {
    bool done = false;
    if(state->object->endAction() == WorldObject::StopAtEnd || state->loops == 0) {
        done = true;
    } else {
        switch(state->object->endAction()) {
        case WorldObject::RunFromStart: state->path->reset(); break;
        case WorldObject::RunBackward: state->path->revert(); break;
        default: break;
        }
        _stg->setPose(state->object->id(),
                      mapToStg(state->path->currentPoint()));
        if(state->loops > 0) --state->loops;
    }

    if(done && state->object->hideAtEnd()) {
        _stg->setVisible(state->object->id(), false);
        emit visibleChanged(state->object->id(), false);
    }
}

Pose Simulator::randomOffset(double maxX, double maxY, double maxTh) const {
    return Pose(randomValue(maxX), randomValue(maxY), Radians(randomValue(maxTh)));
}

double Simulator::randomValue(double maxVal) const {
    // Should be replaced with QRandomGenerator if Qt 5.10 is available
    return (qrand() / (float)RAND_MAX * 2.0 - 1.0) * maxVal;
}

void Simulator::goalReached(const QString &id) {
    auto st = stateById(id);
    if(!st) {
        qDebug() << "unable to find object" << id;
        return;
    }

    switch(st->object->type()) {
    case WorldObject::Robot:
        if(st->path->hasNextPoint()) {
            _stg->moveTo(id, mapToStg(st->path->nextPoint()));
        } else {
            _finished = true;
            _stgTimer->stop();
            emit simulationFinished();
        }
        break;
    case WorldObject::Obstacle:
        switch(st->object->motionType()) {
        case WorldObject::RandomSpeed: {
            auto sp = st->speed + randomOffset(st->object->linearSpeed() / 2.0,
                                               st->object->linearSpeed() / 2.0,
                                               st->object->angularSpeedRad() / 2.0);
            _stg->setSpeed(id, sp.x, sp.y, sp.th.rad(), randomValue(1000) + 1500);
            break;
        }
        case WorldObject::RandomPos:
            _stg->moveTo(id, st->pose + randomOffset(10, 10, 2 * M_PI));
            break;
        case WorldObject::Trajectory:
            if(st->path->hasNextPoint()) {
                _stg->moveTo(id, mapToStg(st->path->nextPoint()));
            } else {
                onPathComplete(st);
            }
            break;
        default:
            return;
        }
        break;
    default:
        break;
    }
}

void Simulator::objectStalled(const QString &id, const Pose &pose) {
    auto st = stateById(id);
    switch(st->object->type()) {
    case WorldObject::Robot:
        qDebug() << id << "crashed";
        emit positionChanged(id, mapFromStg(pose));
        if(st->path->ignoreCount() >= _maxAvoidTries) {
            stop();
            emit robotCrashed(true);
        } else {
            st->path->ignoreNextPoint();
            _stg->setSpeed(id, -st->speed.x / 2.0, 0.0, 0.0, randomValue(250) + 750);
        }
        break;
    case WorldObject::Obstacle:
        //_stg->setSpeed(id, -st->speed.x / 2.0, -st->speed.y / 2.0, -st->speed.th.rad() / 2.0,
        //               randomValue(300) + 800);
        goalReached(id);
        break;
    default:
        break;
    }
}

void Simulator::worldUpdated(const StageWorldState &s) {
    int ri = 0;
    for(auto &obj : s.objects) {
        if(obj.id.startsWith("robot")) break;
        ++ri;
    }

    const StageObjectState &r = s.objects[ri];

    _laserScan.pose = r.laserPose;
    _laserScan.ranges = r.laserRanges;

    if(_bag && _bag->isOpen()) {
        _bag->writeLaserScan(s.time, _laserScan);
        _bag->writeRobotPose(s.time, r.estPose);
        if(_gt) _gt->writeRobotPose(s.time, r.gtPose - r.estOrig);
        if(_finished) {
            _bag->close();
            if(_gt) _gt->close();
        }
    }

    _objects[r.id].path->updateProgress(mapFromStg(r.gtPose));

    if(_visual && !_finished) {
        for(auto &obj : s.objects) {
            auto st = stateById(obj.id);
            if(st) {
                st->pose = obj.gtPose;
                st->speed = obj.speed;
            }
            emit positionChanged(obj.id, mapFromStg(obj.gtPose));
        }

        // transform to viewport coordinates
        _laserScan.pose = mapFromStg(r.gtPose + r.laserPose);
        _laserScan.pose.th = -_laserScan.pose.th;
        for(auto &lr : _laserScan.ranges) lr /= _mapResolution;
        emit scanChanged(_laserScan);
    }
}

QString Simulator::readTemplate(const QString &fileName) const {
    QFile templateFile(fileName);
    templateFile.open(QFile::ReadOnly | QFile::Text);
    return QString::fromLatin1(templateFile.readAll());
}

#define SET_PROPERTY(name, value) \
    tmp.replace(name, QString::number(value));

QString Simulator::generateWorldFile(const SimulatorConfig *config) const {
    auto tmp = readTemplate(":/templates/world.template");
    if(tmp.isEmpty()) return QString();

    auto mapSize = _model->mapWorldSize();
    auto mapFileName = tmpFilePath("map.pgm");
    if(!exportAsPgm(_model->map(), mapFileName)) return QString();

    tmp.replace("<map_filename>",   mapFileName);
    SET_PROPERTY("<rt_resolution>", _model->worldScale() / 2.0);
    SET_PROPERTY("<sim_interval>",  config->simInterval());
    SET_PROPERTY("<map_width>",     mapSize.width());
    SET_PROPERTY("<map_height>",    mapSize.height());
    SET_PROPERTY("<map_x>",         mapSize.width() / 2.0);
    SET_PROPERTY("<map_y>",        -mapSize.height() / 2.0);

    for(auto *obj : _model->objects()) {
        switch(obj->type()) {
        case WorldObject::Robot:
            tmp.append(generateRobot(static_cast<RobotObject*>(obj), config));
            break;
        case WorldObject::Obstacle:
            tmp.append(generateObstacle(obj));
            break;
        default:
            break;
        }
    }

    return tmp;
}

QString Simulator::generateObstacle(const WorldObject *object) const {
    auto tmp = readTemplate(":/templates/obstacle.template");

    // in Stage origin is an object center; object pose is relative to the origin
    auto center = QPointF(object->worldSize().width() / 2.0, object->worldSize().height() / 2.0);
    auto origin = mapToStg(center / _mapResolution - object->origin());
    auto pose = mapToStg(object->pose());

    SET_PROPERTY("<size_x>", object->worldSize().width());
    SET_PROPERTY("<size_y>", object->worldSize().height());
    SET_PROPERTY("<orig_x>", origin.x());
    SET_PROPERTY("<orig_y>", origin.y());
    SET_PROPERTY("<pose_x>", pose.x);
    SET_PROPERTY("<pose_y>", pose.y);
    SET_PROPERTY("<pose_t>", pose.th.deg());
    if(object->motionType() == WorldObject::Static) {
        SET_PROPERTY("<sp_lin>", 0.0);
        SET_PROPERTY("<sp_ang>", 0.0);
    } else {
        SET_PROPERTY("<sp_lin>", object->linearSpeed());
        SET_PROPERTY("<sp_ang>", object->angularSpeed());
    }
    tmp.replace("<name>",  object->id());
    tmp.replace("<drive>", driveTypeToString(object->driveType()));

    switch(object->shapeType()) {
    case WorldObject::Ellipse:
    case WorldObject::CustomShape: {
        auto fileName = tmpFilePath(object->id() + ".pgm");
        if(!exportAsPgm(_model->pixmap(object), fileName)) return QString();
        tmp.replace("<bitmap>", QString("bitmap \"%1\"").arg(fileName));
        break;
    }
    default:
        tmp.replace("<bitmap>", "");
    }

    return tmp;
}

QString Simulator::generateRobot(const RobotObject *object, const SimulatorConfig *config) const {
    QString tmp = readTemplate(":/templates/robot.template");

    auto pose = mapToStg(object->pose());

    tmp.replace("<robot_name>",     object->id());
    tmp.replace("<robot_drive>",    driveTypeToString(object->driveType()));
    SET_PROPERTY("<robot_size_x>",  object->worldSize().width());
    SET_PROPERTY("<robot_size_y>",  object->worldSize().height());
    SET_PROPERTY("<robot_x>",       pose.x);
    SET_PROPERTY("<robot_y>",       pose.y);
    SET_PROPERTY("<robot_th>",      pose.th.deg());
    SET_PROPERTY("<robot_slx>",     object->linearSpeed());
    SET_PROPERTY("<robot_sa>",      object->angularSpeed());
    SET_PROPERTY("<laser_rmin>",    config->laserRangeMin());
    SET_PROPERTY("<laser_rmax>",    config->laserRangeMax());
    SET_PROPERTY("<laser_fov>",     config->laserFOV());
    SET_PROPERTY("<laser_samples>", config->laserSamples());
    if(object->isOdomNoiseEnabled()) {
        tmp.replace("<robot_loc>",  "odom");
        SET_PROPERTY("<robot_nlx>", object->odomNoiseLinear());
        SET_PROPERTY("<robot_na>",  object->odomNoiseAngular());
    } else {
        tmp.replace("<robot_loc>",  "gps");
        SET_PROPERTY("<robot_nlx>", 0.0);
        SET_PROPERTY("<robot_na>",  0.0);
    }
    if(config->isLaserNoiseEnabled() && config->laserNoiseType() == 0) {
        SET_PROPERTY("<laser_nc>",  config->laserNoiseConst());
        SET_PROPERTY("<laser_np>",  config->laserNoiseProp());
        SET_PROPERTY("<laser_na>",  config->laserNoiseAngular());
    } else {
        SET_PROPERTY("<laser_nc>",  0.0);
        SET_PROPERTY("<laser_np>",  0.0);
        SET_PROPERTY("<laser_na>",  0.0);
    }
    if(object->driveType() == WorldObject::Omni) {
        SET_PROPERTY("<robot_sly>", object->linearSpeed());
        SET_PROPERTY("<robot_nly>", object->isOdomNoiseEnabled() ? object->odomNoiseLinear() : 0.0);
    } else {
        SET_PROPERTY("<robot_sly>", 0.0);
        SET_PROPERTY("<robot_nly>", 0.0);
    }

    return tmp;
}

#undef SET_PROPERTY

QString Simulator::driveTypeToString(WorldObject::Drive type) const {
    switch(type) {
    case WorldObject::Diff: return "diff";
    case WorldObject::Omni: return "omni";
    default: break;
    }
    return QString();
}

QString Simulator::tmpFilePath(const QString &fileName) const {
    return _tmpDir ? _tmpDir->path() + "/" + fileName : fileName;
}
