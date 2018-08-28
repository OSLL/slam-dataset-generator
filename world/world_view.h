#ifndef WORLD_VIEW_H
#define WORLD_VIEW_H

#include <QGraphicsView>
#include <QGraphicsPixmapItem>
#include <QVector>
#include <QVector3D>
#include <QTimer>
#include <QSharedPointer>

#include "simulator.h"
#include "widgets.h"
#include "world_model.h"
#include "world_items.h"

class WorldView : public QGraphicsView {
    Q_OBJECT

public:
    WorldView(WorldModel *model, QWidget *parent = 0);

    void setSimulationMode(bool on, bool online = true);

    QString defaultMessage() const;

public slots:
    void clear();
    void setInteractionMode(SimulationToolBox::Tool mode);
    void setObjectPosition(const QString &id, const QVector3D &pos);
    void setRobotCrashed(bool on);
    void setLaserScan(const LaserScan &scan);
    void removeSelectedItems();

signals:
    void zoomChanged(double val);
    void simulationAvailabilityChanged(bool on);
    void objectSelected(WorldObject *object);
    void message(NotificationsWidget::MessageType type, const QString &msg);
    void changeTool(SimulationToolBox::Tool tool);

protected:
    void mouseDoubleClickEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void enterEvent(QEvent *event);
    void leaveEvent(QEvent *event);

private slots:
    void updateSelection();
    void updateObjectSize();
    void updateObjectOrigin();
    void modifyWaypoint(int action);
    void updateWaypoint();
    void updateWaypointShape();
    void updateTrajectory();
    void updatePath();

    void setMap(const QPixmap &map);
    void createObject(WorldObject *object);
    void removeObject(WorldObject *object);
    void updateObject(WorldObject *object, int role, const QVariant &data);
    void createWaypoint(WorldObject *object, int i);
    void removeWaypoint(WorldObject *object, int i);
    void setPath(WorldObject *object, int i);

private:
    enum WaypointAction { RemoveWaypoint, UpdateWaypoint, RemoveTrajectory };

    //-------------------------------------------------------------------------

    struct ObjectView {
        ObjectView(WorldObject *object = 0, WorldObjectItem *item = 0)
            : object(object), item(item) {}

        WorldObject *object;
        WorldObjectItem *item;
        QList<WorldObjectItem*> waypoints;
        QList<WorldObjectPathItem*> paths;
    };

    //-------------------------------------------------------------------------

    class ObjectViewHash {
    public:
        using ObjectViewPtr = QSharedPointer<ObjectView>;

        ObjectViewHash() {}

        auto begin() { return _objectToView.begin(); }
        auto end()   { return _objectToView.end(); }

        void add(WorldObject *object, WorldObjectItem *item);
        void remove(WorldObject *object);
        void clear();

        ObjectView *operator [](WorldObject *object) const;
        ObjectView *operator [](WorldObjectItem *item) const;

    private:
        QHash<WorldObject*, ObjectViewPtr> _objectToView;
        QHash<WorldObjectItem*, ObjectViewPtr> _itemToView;
    };

    //-------------------------------------------------------------------------

    class ControlStack : public QObject {
    public:
        ControlStack(WorldView *view) : QObject(view), _view(view) {}

        void addControl(WorldObjectItem *item, WorldObjectControlItem::ControlType type);
        void removeControl(WorldObjectItem *item, WorldObjectControlItem::ControlType type);
        void removeAllControls(WorldObjectItem *item);
        void removeAllControls();

    private:
        WorldView *_view;
        QHash<WorldObjectItem*, QList<WorldObjectControlItem*>> _controls;
    };

    //-------------------------------------------------------------------------

    class DelayTimer : public QTimer {
    public:
        DelayTimer(int interval, QObject *parent = 0) : QTimer(parent) {
            setInterval(interval);
            setSingleShot(true);
        }

        void delay() {
            stop();
            start();
        }
    };

    //-------------------------------------------------------------------------

    void createDefaultControls();

    void checkPaths();
    void updatePath(WorldObjectItem *wp);
    void setTempPath(ObjectView *ov, int p1, int p2) const;
    WorldObjectPathItem *createEmptyPath(WorldObject *object);

    void setTrajectoryMode(ObjectView *ov, bool on) const;
    void setTrajectoryMode(WorldObjectItem *wpi, bool on) const;
    void changeTrajectoryBase(WorldObject *newBase);

    void addObject(WorldObject::Type type, const QPointF &pos);
    void removeObject(WorldObjectItem *woi);

    void addWaypoint(WorldObject *object, int i, const QPointF &scenePos);
    bool isWaypoint(WorldObjectItem *item) const;
    Pose waypointPose(WorldObjectItem *wp) const;
    void removeWaypoint(WorldObjectItem *wp);
    void removeTrajectory(WorldObjectItem *wp);

    WorldObject *baseObject(WorldObjectItem *item) const;
    WorldObjectItem *baseWaypoint(WorldObject *object) const;
    WorldObjectItem *worldObjectItemAt(const QPoint &p) const;
    ObjectView *senderItemView() const;

    QVector3D objectPose(const WorldObject *wo, int wpi = -1) const;

    QPair<int, int> findWaypoints(WorldObjectPathItem *pathItem) const;

    void snapCursor(const QMouseEvent *event, double snapR);

    // NOTE: uncomment if a model stores coordinates in the world scale
    inline QPointF mapToModel(const QPointF &p) const   { return p /* * _model->worldScale() */; }
    inline QPointF mapFromModel(const QPointF &p) const { return p /* / _model->worldScale() */; }

    inline QSizeF mapToModel(const QSizeF &s) const   { return s * _model->worldScale(); }
    inline QSizeF mapFromModel(const QSizeF &s) const { return s / _model->worldScale(); }

    QList<RobotItem*> robotList() const;
    void setScale(double value);

    WorldModel *_model;

    QGraphicsPixmapItem *_map;
    LaserScanItem *_scan;
    PointerItem *_pointer;

    WorldObject *_trajBase;
    DelayTimer *_trajUpdateTimer;
    WorldObjectPathItem *_selectedPath;
    QSet<WorldObjectItem*> _selectedItems;

    ControlStack *_itemControls;
    ObjectViewHash _objectViews;

    SimulationToolBox::Tool _mode;
    double _viewScale;
    QPoint _lastMousePos;
    bool _mousePressed;
};

#endif // WORLD_VIEW_H
