#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <QObject>
#include <QVector>
#include <QVector3D>
#include <QRectF>
#include <QSizeF>
#include <QPixmap>
#include <QImage>
#include <QTimer>
#include <QFile>
#include <QHash>
#include <QTextStream>

#include <rosbag/bag.h>
#include <geometry_msgs/TransformStamped.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include "stage_wrapper.h"
#include "world_model.h"
#include "ui_simulator_config.h"

struct LaserScan {
    QVector3D pose;
    QVector<double> ranges;
    float minRange, maxRange;
    float minAngle, maxAngle;
    float angleIncrement;
};

//=============================================================================

class PathPlanner : public QObject {
public:
    PathPlanner(QObject *parent = 0);
    ~PathPlanner();

    QVector<QVector3D> makePlan(const QVector3D &start, const QVector3D &goal);

    void setMap(const QPixmap &map);
    void setRobotSize(const QSizeF &size);

private:
    bool isStateValid(const ompl::base::State *state) const;
    bool isValidPixel(int x, int y) const;

    QRectF _robot;
    QImage _map;
};

//=============================================================================

class BagWriter : public QObject {
public:
    BagWriter(QObject *parent = 0);
    ~BagWriter();

    ros::Time startTime() const { return _time; }

    bool isOpen() const { return _open; }
    bool open(const QString &fileName);
    void close();

    void writeLaserScan(double time, const LaserScan &scan);
    void writeRobotPose(double time, const QVector3D &pose);

private:
    geometry_msgs::Transform createTransform(double x, double y, double th) const;

    bool _open;
    ros::Time _time;
    rosbag::Bag _bag;
};

//=============================================================================
// TODO make common ancestor with BagWriter
//      or include into BagWriter
class GroundTruthWriter : public QObject {
public:
    GroundTruthWriter(QObject *parent = 0);

    void setStartTime(const ros::Time &time) {
        _time = time;
    }

    bool open(const QString &fileName);
    void close();

    void writeRobotPose(double time, const QVector3D &pose);

private:
    ros::Time _time;
    QTextStream _ts;
    QFile _file;
};

//=============================================================================

#define PROPERTY(type, var, get, set) \
    type get() const { return var; } \
    void set(const type &val) { var = val; }

#define SB_PROPERTY(type, member, get, set) \
    type get() const { return member->value(); } \
    void set(type val) { member->setValue(val); }

#define LE_PROPERTY(member, get, set) \
    QString get() const { return member->text(); } \
    void set(const QString &val) { member->setText(val); }

#define CB_PROPERTY(member, get, set) \
    int get() const { return member->currentIndex(); } \
    void set(int i) const { member->setCurrentIndex(i); }

class SimulatorConfig : public QWidget, private Ui::SimulatorConfig {
    Q_OBJECT

public:
    enum Option { MapResolution = 0x1 };
    Q_DECLARE_FLAGS(Options, Option)

    SimulatorConfig(QWidget *parent = 0);

    void setMapFileName(const QString &fileName);

    QString gtFileName() const { return _gtFileName; }
    void setGtFileName(const QString &fileName) { _gtFileName = fileName; }

    bool isOutputEnabled() const { return _gbOutput->isChecked(); }
    bool isGroundtruthEnabled() const { return _cbGroundtruth->isChecked(); }
    bool isVisualizationEnabled() const { return _gbVisualization->isChecked(); }
    bool isLaserNoiseEnabled() const { return _laserNoise->isChecked(); }

    SB_PROPERTY(int, _visualizationRate, visualizationRate, setVisualizationRate)
    SB_PROPERTY(double, _mapResolution, mapResolution, setMapResolution)
    SB_PROPERTY(int, _simulationInterval, simInterval, setSimInterval)
    SB_PROPERTY(int, _maxAvoidTries, maxAvoidTries, setMaxAvoidTries)
    SB_PROPERTY(double, _laserRangeMin, laserRangeMin, setLaserRangeMin)
    SB_PROPERTY(double, _laserRangeMax, laserRangeMax, setLaserRangeMax)
    SB_PROPERTY(double, _laserFOV, laserFOV, setLaserFOV)
    SB_PROPERTY(int, _laserSamples, laserSamples, setLaserSamples)
    SB_PROPERTY(double, _laserNoiseConst, laserNoiseConst, setLaserNoiseConst)
    SB_PROPERTY(double, _laserNoiseProp, laserNoiseProp, setLaserNoiseProp)
    SB_PROPERTY(double, _laserNoiseAng, laserNoiseAngular, setLaserNoiseAngular)
    LE_PROPERTY(_outputFile, outputFileName, setOutputFileName)
    CB_PROPERTY(_laserNoiseType, laserNoiseType, setLaserNoiseType)

signals:
    void settingsUpdated(SimulatorConfig::Options options);

private slots:
    void selectOutputFile();
    void updateGtFileName(const QString &ofn);
    void updateOptionsFlags(int flag);
    void applySettings();

private:
    QString _gtFileName;
    Options _updatedOptions;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(SimulatorConfig::Options)

#undef PROPERTY
#undef SB_PROPERTY
#undef LE_PROPERTY
#undef CB_PROPERTY

//=============================================================================

class PathTracker : public QObject {
    Q_OBJECT

public:
    PathTracker(QObject *parent = 0);
    PathTracker(const QVector<QVector3D> &path, bool align, QObject *parent = 0);

    void init(const QVector<QVector3D> &path, bool align);

    bool hasNextPoint() const;
    QVector3D nextPoint();

    void ignoreNextPoint() { _ignoreNextPoint = true; }
    int ignoreCount() const { return _ignoreCount; }

    float updateProgress(const QVector3D &pos);

signals:
    void progress(int val) const;

private:
    double distance(const QVector3D &p1, const QVector3D &p2) const;
    QVector3D alignedTo(const QVector3D &p1, const QVector3D &p2) const;

    QVector<QVector3D> _path;
    QVector<int> _waypointIdxs;
    QVector3D _currentPoint;
    int _curIdx;

    bool _ignoreNextPoint, _alignPath;
    int _ignoreCount;

    double _totalLength;
    double _curChunkLength;
    double _completedLength;
};

//=============================================================================

class Simulator : public QObject {
    Q_OBJECT

public:
    Simulator(const WorldModel *model, QObject *parent = 0);
    ~Simulator();

    void simulate(const SimulatorConfig *config);
    void stop();

signals:
    void positionChanged(const QVector3D &newPos);
    void positionChanged(const QString &id, const QVector3D &newPos);
    void scanChanged(const LaserScan &scan);
    void robotCrashed(bool on);
    void simulationProgress(int value);
    void simulationFinished();

private slots:
    void goalReached(const QString &id = "robot");
    void objectStalled(const QString &id, const QVector3D &pose);
    void worldUpdated(const StageWorldState &s);

private:
    struct ObjectState {
        ObjectState(WorldObject *object = 0,
                    const QVector3D &pose = QVector3D(),
                    const QVector<QVector3D> &path = QVector<QVector3D>());
        ~ObjectState();

        void setPath(const QVector<QVector3D> &p, bool align);

        WorldObject *object;
        PathTracker *path;
        QVector3D pose, speed;
    };

    QString readTemplate(const QString &fileName) const;
    QString generateWorldFile(const SimulatorConfig *config) const;
    QString generateObstacle(const WorldObject *object) const;
    QString generateRobot(const RobotObject *object, const SimulatorConfig *config) const;
    QString driveTypeToString(WorldObject::Drive type) const;

    QPointF mapToStg(const QPointF &p) const;
    QVector3D mapToStg(const QVector3D &p) const;
    QVector3D mapToStg(const Pose &p) const;
    QVector3D mapFromStg(const QVector3D &p) const;

    ObjectState *stateById(const QString &id);
    QVector3D randomOffset(double maxX, double maxY, double maxTh = 0) const;
    double randomValue(double maxVal) const;

    StageWrapper *_stg;
    BagWriter *_bag;
    GroundTruthWriter *_gt;
    QTimer *_stgTimer;
    const WorldModel *_model;

    QHash<QString, ObjectState> _objects;

    int _maxAvoidTries;
    double _mapResolution;
    bool _finished, _visual;
    LaserScan _laserScan;
};

#endif // SIMULATOR_H