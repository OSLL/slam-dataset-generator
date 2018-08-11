#include "world_view.h"

#include <QKeyEvent>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QScrollBar>
#include <QPen>
#include <QtMath>

#include <cmath>

#include <QDebug>

static QPen cosmeticPen(const QBrush &brush) {
    QPen p(brush, 1.0, Qt::DashLine);
    p.setCosmetic(true);
    return p;
}

static double normalizedRotation(double angle) {
    angle = std::fmod(angle + 180.0, 360.0);
    angle += angle < 0 ? 360.0 : -180.0;
    return qDegreesToRadians(angle);
}

//=============================================================================

WorldView::WorldView(WorldModel *model, QWidget *parent)
    : QGraphicsView(parent), _model(model), _map(0),
      _mode(SimulationToolBox::Navigation), _viewScale(1.0), _mousePressed(false)
{
    setScene(new QGraphicsScene(this));
    setInteractive(true);
    setRenderHint(QPainter::Antialiasing);

    _itemControls = new ControlStack(this);
    _trajUpdateTimer = new DelayTimer(500, this);

    connect(_model, SIGNAL(mapChanged(QPixmap)), this, SLOT(setMap(QPixmap)));
    connect(_model, SIGNAL(objectCreated(WorldObject*)), this, SLOT(createObject(WorldObject*)));
    connect(_model, SIGNAL(objectRemoved(WorldObject*)), this, SLOT(removeObject(WorldObject*)));
    connect(_model, SIGNAL(dataChanged(WorldObject*,int,QVariant)), this, SLOT(updateObject(WorldObject*,int,QVariant)));
    connect(_model, SIGNAL(waypointCreated(WorldObject*,int)), this, SLOT(createWaypoint(WorldObject*,int)));
    connect(_model, SIGNAL(waypointRemoved(WorldObject*,int)), this, SLOT(removeWaypoint(WorldObject*,int)));
    connect(_model, SIGNAL(pathChanged(WorldObject*,int)), this, SLOT(setPath(WorldObject*,int)));

    connect(_trajUpdateTimer, SIGNAL(timeout()), this, SLOT(updateTrajectory()));
    connect(scene(), SIGNAL(selectionChanged()), this, SLOT(updateSelection()));
}

void WorldView::clear() {
    changeTrajectoryBase(0);
    _itemControls->removeAllControls();
    _objectViews.clear();
    scene()->clear();
}

void WorldView::setSimulationMode(bool on, bool online) {
    if(!_map) return;
    auto robots = robotList();

    if(on) {
        scene()->clearSelection();
        for(auto *r : robots) r->setCrashed(false);
    }

    for(auto &ov : _objectViews) {
        if(!on) ov->item->setPose(ov->waypoints[0]->pose());
        if(!ov->waypoints.isEmpty()) ov->waypoints[0]->setVisible(on);
        ov->item->blockSignals(on);
    }

    for(auto *r : robots) r->setVisible(!on || online);
    _scan->setVisible(on && online);

    if(!on && _trajBase && _mode == SimulationToolBox::SetTrajectory) {
        setTrajectoryMode(_objectViews[_trajBase], true);
    }

    setInteractive(!on);
}

QString WorldView::defaultMessage() const {
    switch(_mode) {
    case SimulationToolBox::AddRobot:
        return "<b>Left double click:</b> add robot <b>Ctrl + Left double click:</b> add obstacle <b>Right click:</b> edit object's trajectory";
    case SimulationToolBox::AddObstacle:
        return "<b>Left double click:</b> add obstacle <b>Ctrl + Left double click:</b> add robot <b>Right click:</b> edit object's trajectory";
    case SimulationToolBox::SetTrajectory:
        return "<b>Select</b> item to start trajectory editing <b>Left double click:</b> add waypoint";
    default:
        return "<b>Right drag:</b> scroll map";
    }
}

void WorldView::setInteractionMode(SimulationToolBox::Tool mode) {
    if(_mode == mode) return;

    bool controlUpdated = false;

    switch(mode) {
    case SimulationToolBox::SetTrajectory: {
        auto items = scene()->selectedItems();
        if(items.size() != 1) break;

        auto *i = dynamic_cast<WorldObjectItem*>(items.first());
        if(!i) break;

        _itemControls->removeAllControls(i);
        changeTrajectoryBase(_objectViews[i]->object);
        controlUpdated = true;
        break;
    }
    case SimulationToolBox::AddRobot:
    case SimulationToolBox::AddObstacle:
        if(_mode == SimulationToolBox::SetTrajectory && _trajBase) {
            auto *prevOV = _objectViews[_trajBase];
            changeTrajectoryBase(0);

            prevOV->item->setSelected(true);
            _itemControls->addControl(prevOV->item, WorldObjectControlItem::Shape);
            controlUpdated = true;

            emit objectSelected(prevOV->object);
        }
        break;
    case SimulationToolBox::Navigation:
        _itemControls->removeAllControls();
        changeTrajectoryBase(0);
        break;
    default:
        break;
    }

    if(!controlUpdated) scene()->clearSelection();

    _mode = mode;
    emit message(NotificationsWidget::Info, defaultMessage());
}

void WorldView::setObjectPosition(const QString &id, const QVector3D &pos) {
    auto *ov = _objectViews[_model->object(id)];
    if(!ov) return;

    auto item = ov->item;
    if(item->type() == RobotItem::Type) {
        // robot's origin is always in (0, 0) so the pos can be set directly
        item->setPos(pos.toPointF());
    } else {
        // a hack to avoid rounding errors when mapping pos to item's coordinates
        auto so = item->sceneOrigin();
        item->setRotation(0);
        auto d = so - item->sceneOrigin();
        item->setPos(pos.toPointF() - item->origin() - d);
    }

    item->setRotation(qRadiansToDegrees(pos.z()));
}

void WorldView::setRobotCrashed(bool on) {
    auto robots = robotList();
    if(robots.size() != 1) return;

    if(on) {
        emit message(NotificationsWidget::Warning, "<b>Error:</b> robot crashed");
        robots.first()->show();
    }
    robots.first()->setCrashed(on);
}

void WorldView::setLaserScan(const LaserScan &scan) {
    if(!_scan) return;
    _scan->setPos(scan.pose.toPointF());
    _scan->setLaserScan(scan);
}

void WorldView::removeSelectedItems() {
    emit objectSelected(0);
    for(auto i : scene()->selectedItems()) {
        auto obj = dynamic_cast<WorldObjectItem*>(i);
        if(!obj) continue;
        if(isWaypoint(obj)) removeWaypoint(obj);
        else removeObject(obj);
    }
}

void WorldView::mouseDoubleClickEvent(QMouseEvent *event) {
    if(!_map || !isInteractive()) return;
    auto p = mapToScene(event->pos());

    switch(_mode) {
    case SimulationToolBox::SetTrajectory:
        if(_trajBase) addWaypoint(_trajBase, p);
        break;
    case SimulationToolBox::AddRobot:
        addObject(event->modifiers() == Qt::ControlModifier ? WorldObject::Obstacle
                                                               : WorldObject::Robot, p);
        break;
    case SimulationToolBox::AddObstacle:
        addObject(event->modifiers() == Qt::ControlModifier ? WorldObject::Robot
                                                               : WorldObject::Obstacle, p);
        break;
    default:
        break;
    }
}

void WorldView::mousePressEvent(QMouseEvent *event) {
    _lastMousePos = event->pos();
    _mousePressed = true;

    auto *woi = worldObjectItemAt(event->pos());
    if(!woi || !isInteractive()) {
        QGraphicsView::mousePressEvent(event);
        return;
    }

    switch(_mode) {
    case SimulationToolBox::AddRobot:
    case SimulationToolBox::AddObstacle:
    case SimulationToolBox::Navigation:
        if(event->button() == Qt::RightButton) {
            auto *wo = baseObject(woi);
            if(wo) {
                emit changeTool(SimulationToolBox::SetTrajectory);
                changeTrajectoryBase(wo);
                return;
            }
        }
        break;
    case SimulationToolBox::SetTrajectory:
        if(event->button() == Qt::LeftButton) {
            auto *wo = baseObject(woi);
            if(wo && wo != _trajBase) {
                changeTrajectoryBase(wo);
                return;
            }
        }
        break;
    default:
        break;
    }

    QGraphicsView::mousePressEvent(event);
}

void WorldView::mouseMoveEvent(QMouseEvent *event) {
    if(event->buttons() & Qt::RightButton) {
        // Copy of QGraphicsView src to avoid default cursor override
        auto *hBar = horizontalScrollBar();
        auto *vBar = verticalScrollBar();
        auto delta = event->pos() - _lastMousePos;
        hBar->setValue(hBar->value() + (isRightToLeft() ? delta.x() : -delta.x()));
        vBar->setValue(vBar->value() - delta.y());
        _lastMousePos = event->pos();
    }
    QGraphicsView::mouseMoveEvent(event);
}

void WorldView::mouseReleaseEvent(QMouseEvent *event) {
    _mousePressed = false;
    QGraphicsView::mouseReleaseEvent(event);
}

void WorldView::wheelEvent(QWheelEvent *event) {
    QPointF sceneTarget = mapToScene(event->pos());

    if(event->delta() < 0) setScale(qMax(0.1, _viewScale * 0.85));
    else setScale(qMin(10.0, qMax(_viewScale * 1.15, 0.05)));

    QPointF viewportDelta = event->pos() - viewport()->rect().center();
    QPointF viewportCenter = mapFromScene(sceneTarget) - viewportDelta;
    centerOn(mapToScene(viewportCenter.toPoint()));

    emit zoomChanged(_viewScale);
}

void WorldView::keyPressEvent(QKeyEvent *event) {
    switch(event->key()) {
    case Qt::Key_Escape:
        switch(_mode) {
        case SimulationToolBox::AddRobot:
        case SimulationToolBox::AddObstacle:
            scene()->clearSelection();
            break;
        case SimulationToolBox::SetTrajectory:
            scene()->clearSelection();
            changeTrajectoryBase(0);
            break;
        default:
            break;
        }
        break;
    case Qt::Key_Delete:
        removeSelectedItems();
        break;
    default:
        break;
    }
    QGraphicsView::keyPressEvent(event);
}

void WorldView::updateSelection() {
    QSet<WorldObjectItem*> newSelection;
    for(auto *item : scene()->selectedItems()) {
        auto *woi = dynamic_cast<WorldObjectItem*>(item);
        if(woi) newSelection.insert(woi);
    }

    for(auto *woi : _selectedItems.subtract(newSelection)) {
        _itemControls->removeControl(woi, WorldObjectControlItem::Waypoint);
        _itemControls->removeControl(woi, WorldObjectControlItem::Shape);
    }

    _selectedItems = newSelection;
    if(_selectedItems.isEmpty()) {
        emit objectSelected(0);
        return;
    }

    auto *item = *_selectedItems.begin();
    if(isWaypoint(item)) {
        _itemControls->addControl(item, WorldObjectControlItem::Waypoint);
    } else {
        _itemControls->addControl(item, WorldObjectControlItem::Shape);

        auto *ov = _objectViews[item];
        if(ov) emit objectSelected(ov->object);
    }
}

//-----------------------------------------------------------------------------

void WorldView::setMap(const QPixmap &map) {
    clear();

    _map = scene()->addPixmap(map);

    _scan = new LaserScanItem();
    _scan->setZValue(1.0);
    scene()->addItem(_scan);
}

void WorldView::createObject(WorldObject *object) {
    WorldObjectItem *item;

    switch(object->type()) {
    case WorldObject::Robot:
        item = new RobotItem();
        item->setZValue(0.6);
        break;
    case WorldObject::Obstacle: {
        item = new ObstacleItem();
        item->setZValue(0.5);
        break;
    }
    default:
        return;
    }

    item->setSize(mapFromModel(object->worldSize()));
    item->setOrigin(mapFromModel(object->origin()));
    item->setPose(objectPose(object));
    item->setViewScale(_viewScale);
    scene()->addItem(item);

    _objectViews.add(object, item);

    for(int i = 0; i < object->waypointCount(); ++i) {
        createWaypoint(object, i);
    }

    connect(item, SIGNAL(poseEdited()), this, SLOT(updatePath()));
    connect(item, SIGNAL(originEdited()), this, SLOT(updateTrajectory()));
    connect(item, SIGNAL(shapeChanged()), this, SLOT(updateWaypointShape()));
    connect(item, SIGNAL(sizeChanged(QSizeF)), this, SLOT(updateObjectSize()));
    connect(item, SIGNAL(originChanged(QPointF)), this, SLOT(updateObjectOrigin()));
}

void WorldView::removeObject(WorldObject *object) {
    auto *ov = _objectViews[object];

    if(object == _trajBase) changeTrajectoryBase(0);
    _itemControls->removeAllControls(ov->item);
    delete ov->item;

    _objectViews.remove(object);

    if(_model->robotCount() == 0) {
        emit simulationAvailabilityChanged(false);
    }
}

void WorldView::updateObject(WorldObject *object, int role, const QVariant &data) {
    auto *ov = _objectViews[object];
    if(!ov) return;

    switch(role) {
    case WorldModel::BrushRole: {
        ov->item->setBrush(data.value<QBrush>());
        break;
    }
    case WorldModel::SizeRole: {
        auto size = mapFromModel(data.toSizeF());
        if(ov->item->size() != size) ov->item->setSize(size);
        break;
    }
    case WorldModel::ShapeRole:
        ov->item->setShapeType(static_cast<WorldObject::Shape>(data.toInt()));
        break;
    case WorldModel::ShapePixmapRole:
        ov->item->setShapePixmap(data.value<QPixmap>());
        break;
    default:
        return;
    }
}

void WorldView::createWaypoint(WorldObject *object, int i) {
    auto *ov = _objectViews[object];
    if(!ov) return;

    auto *base = ov->item;

    WorldObjectItem *wpi = 0;

    switch(object->type()) {
    case WorldObject::Robot:
        wpi = new RobotItem();
        break;
    case WorldObject::Obstacle:
        wpi = new ObstacleItem();
        wpi->setSize(base->size());
        wpi->setOrigin(base->origin());
        break;
    default:
        return;
    }

    wpi->setPose(objectPose(object, i));
    wpi->setBaseItem(base);
    wpi->setZValue(base->zValue() - 0.01);
    wpi->setViewScale(_viewScale);
    wpi->setBrush(base->brush());
    if(base->shapeType() == WorldObject::CustomShape) {
        wpi->setShapePixmap(base->shapePixmap());
    }
    wpi->setShapeType(base->shapeType());

    scene()->addItem(wpi);

    ov->waypoints.insert(i, wpi);
    if(i > 0) {
        ov->paths.insert(i - 1, createEmptyPath());
        if(i < object->waypointCount() - 1) {
            ov->paths.insert(i, createEmptyPath());
        }
    }

    if(i == 0) {
        connect(base, SIGNAL(poseChanged(QVector3D)), wpi, SLOT(setPose(QVector3D)));
        wpi->setVisible(false);
    }

    connect(base, SIGNAL(sizeChanged(QSizeF)), wpi, SLOT(setSize(QSizeF)));
    connect(base, SIGNAL(brushChanged(QBrush)), wpi, SLOT(setBrush(QBrush)));
    connect(base, SIGNAL(originChanged(QPointF)), wpi, SLOT(setOrigin(QPointF)));
    connect(wpi, SIGNAL(poseChanged(QVector3D)), this, SLOT(updateWaypoint()));
    connect(wpi, SIGNAL(poseEdited()), this, SLOT(updatePath()));
}

void WorldView::removeWaypoint(WorldObject *object, int i) {
    auto *ov = _objectViews[object];

    _itemControls->removeAllControls(ov->waypoints[i]);
    delete ov->waypoints[i];
    ov->waypoints.removeAt(i);

    if(i < ov->paths.size()) {
        delete ov->paths[i];
        ov->paths.removeAt(i);
    }

    if(i > 0) {
        setTempPath(ov, i - 1, i);
    }

    if(i == 0 && !ov->waypoints.isEmpty()) {
        connect(ov->item, SIGNAL(poseChanged(QVector3D)), ov->waypoints[0], SLOT(setPose(QVector3D)));
        _itemControls->addControl(ov->waypoints[0], WorldObjectControlItem::TrajectoryBase);
    }
}

void WorldView::setPath(WorldObject *object, int i) {
    auto &path = object->path(i);

    QPainterPath p;
    if(path.ok) {
        p.moveTo(mapFromModel(path.first().toPointF()));
        for(auto pi = path.begin() + 1; pi != path.end(); ++pi) {
            p.lineTo(mapFromModel(pi->toPointF()));
        }
    } else {
        p.moveTo(_objectViews[object]->waypoints[i]->sceneOrigin());
        p.lineTo(_objectViews[object]->waypoints[i + 1]->sceneOrigin());
    }

    auto *pathItem = _objectViews[object]->paths[i];
    pathItem->setPath(p);
    pathItem->setPen(path.ok ? QPen(Qt::black) : QPen(Qt::red));

    checkPaths();
}

//-----------------------------------------------------------------------------

void WorldView::addObject(WorldObject::Type type, const QPointF &pos) {
    if(type == WorldObject::Robot && _model->robotCount() > 0) {
        emit message(NotificationsWidget::Warning,
                     "<b>Warning:</b> Currently only a single robot can be simulated");
        return;
    }
    _model->addObject(type, mapToModel(_map->mapFromScene(pos)));
}

void WorldView::removeObject(WorldObjectItem *woi) {
    _model->removeObject(_objectViews[woi]->object);
}

void WorldView::addWaypoint(WorldObject *object, const QPointF &scenePos) {
    auto mapPos = mapToModel(_map->mapFromScene(scenePos));
    _model->addWaypoint(object, {mapPos.x(), mapPos.y(), 0});
}

bool WorldView::isWaypoint(WorldObjectItem *item) const {
    return item->baseItem() != 0;
}

Pose WorldView::waypointPose(WorldObjectItem *wp) const {
    auto p = mapToModel(_map->mapFromScene(wp->sceneOrigin()));
    return {p.x(), p.y(), normalizedRotation(wp->rotation())};
}

void WorldView::removeWaypoint() {
    auto *wp = qobject_cast<WaypointControlItem*>(sender());
    if(wp) removeWaypoint(wp->controlledItem());
}

void WorldView::removeWaypoint(WorldObjectItem *wp) {
    auto *ov = _objectViews[wp->baseItem()];
    int i = ov->waypoints.indexOf(wp);
    if(i >= 0) _model->removeWaypoint(ov->object, i);
}

//-----------------------------------------------------------------------------

void WorldView::updateObjectSize() {
    auto *ov = senderItemView();
    if(!ov) return;

    if(!_trajUpdateTimer->isActive()) {
        for(int i = 0; i < ov->waypoints.size() - 1; ++i) {
            setTempPath(ov, i, i + 1);
        }
    }

    _model->setData(ov->object, WorldModel::SizeRole, mapToModel(ov->item->size()));
    _trajUpdateTimer->delay();
}

void WorldView::updateObjectOrigin() {
    auto *ov = senderItemView();
    if(ov) _model->setData(ov->object, WorldModel::OriginRole, mapToModel(ov->item->origin()));
}

void WorldView::updateWaypoint() {
    auto *wp = qobject_cast<WorldObjectItem*>(sender());
    if(!wp) return;

    auto *ov = _objectViews[wp->baseItem()];
    int i = ov->waypoints.indexOf(wp);
    if(i < 0) return;
    if(i > 0) setTempPath(ov, i - 1, i);
    if(i < ov->waypoints.size() - 1) setTempPath(ov, i, i + 1);
    _model->setWaypoint(ov->object, i, waypointPose(wp));
}

void WorldView::updateWaypointShape() {
    auto *ov = senderItemView();
    if(!ov) return;

    auto stype = ov->item->shapeType();
    for(auto *wp : ov->waypoints) {
        wp->setShapeType(stype);
        if(stype == WorldObject::CustomShape) {
            wp->setShapePixmap(ov->item->shapePixmap());
        }
    }
}

void WorldView::updateTrajectory() {
    if(_mousePressed) {
        _trajUpdateTimer->delay();
        return;
    }

    _trajUpdateTimer->stop();

    ObjectView *ov = 0;
    if(sender() == _trajUpdateTimer) {
        for(auto *i : scene()->selectedItems()) {
            if(i->type() == RobotItem::Type || i->type() == ObstacleItem::Type) {
                ov = _objectViews[static_cast<WorldObjectItem*>(i)];
                break;
            }
        }
    } else {
        ov = senderItemView();
    }

    if(ov) {
        int i = 0;
        for(auto *wp : ov->waypoints) {
            _model->setWaypoint(ov->object, i++, waypointPose(wp));
        }
        _model->updateTrajectory(ov->object);
    }
}

void WorldView::updatePath() {
    auto *wpc = qobject_cast<WaypointControlItem*>(sender());
    if(wpc) {
        updatePath(wpc->controlledItem());
        return;
    }

    auto *wp = qobject_cast<WorldObjectItem*>(sender());
    if(!wp) return;

    if(!isWaypoint(wp)) {
        auto *ov = _objectViews[wp];
        if(!ov) return;
        wp = ov->waypoints[0];
    }

    updatePath(wp);
}

//-----------------------------------------------------------------------------

void WorldView::checkPaths() {
    bool ok = true;
    emit message(NotificationsWidget::Info, defaultMessage());

    bool robotFound = false;
    for(auto *wo : _model->objects()) {
        if(wo->type() == WorldObject::Robot) {
            robotFound = true;
            if(!wo->hasPath()) {
                ok = false;
                break;
            }
        }
        for(auto &p : wo->paths()) {
            if(!p.ok) {
                emit message(NotificationsWidget::Warning,
                             "<b>Warning:</b> unable to find path between waypoints");
                ok = false;
                break;
            }
        }
    }

    emit simulationAvailabilityChanged(robotFound && ok);
}

void WorldView::updatePath(WorldObjectItem *wp) {
    auto *wo = baseObject(wp);
    int i = _objectViews[wo]->waypoints.indexOf(wp);
    _model->updateWaypoint(wo, i);
}

void WorldView::setTempPath(ObjectView *ov, int p1, int p2) const {
    static const QPen tmpTrajPen = cosmeticPen(Qt::black);

    QPainterPath p;
    p.moveTo(ov->waypoints[p1]->sceneOrigin());
    p.lineTo(ov->waypoints[p2]->sceneOrigin());
    auto pi = ov->paths[p1];
    pi->setPath(p);
    pi->setPen(tmpTrajPen);
}

QGraphicsPathItem *WorldView::createEmptyPath() {
    auto *path = new QGraphicsPathItem();
    path->setZValue(0.1);
    scene()->addItem(path);
    return path;
}

//-----------------------------------------------------------------------------

void WorldView::setTrajectoryMode(ObjectView *ov, bool on) const {
    for(auto *wpi : ov->waypoints) {
        wpi->setFlag(QGraphicsItem::ItemIsSelectable, on);
        wpi->setFlag(QGraphicsItem::ItemIsMovable, on);
        wpi->setOpacity(on ? 1.0 : 0.25);
    }
    if(on) _model->setData(ov->object, WorldModel::MotionRole, WorldObject::Trajectory);
    if(!ov->waypoints.isEmpty()) ov->waypoints.first()->setVisible(on);
    ov->item->setVisible(!on);
}

void WorldView::changeTrajectoryBase(WorldObject *newBase) {
    if(_trajBase) {
        auto *bv = _objectViews[_trajBase];
        setTrajectoryMode(bv, false);

        if(!bv->waypoints.isEmpty()) {
            QSignalBlocker block(bv->item);
            bv->item->setPose(baseWaypoint(_trajBase)->pose());
            _itemControls->removeControl(baseWaypoint(_trajBase), WorldObjectControlItem::TrajectoryBase);
        }
    }
    _trajBase = newBase;
    if(newBase) {
        setTrajectoryMode(_objectViews[newBase], true);
        _itemControls->addControl(baseWaypoint(newBase), WorldObjectControlItem::TrajectoryBase);
    }
}

//-----------------------------------------------------------------------------

WorldObject *WorldView::baseObject(WorldObjectItem *item) const {
    auto *ov = _objectViews[item->baseItem() ? item->baseItem() : item];
    return ov ? ov->object : 0;
}

WorldObjectItem *WorldView::baseWaypoint(WorldObject *object) const {
    auto *ov = _objectViews[object];
    if(!ov || ov->waypoints.isEmpty()) return 0;
    return ov->waypoints.first();
}

WorldObjectItem *WorldView::worldObjectItemAt(const QPoint &p) const {
    WorldObjectItem *woi = 0;
    for(auto *i : items(p)) {
        while(i->parentItem()) i = i->parentItem();
        woi = dynamic_cast<WorldObjectItem*>(i);
        if(woi) break;
    }
    return woi;
}

WorldView::ObjectView *WorldView::senderItemView() const {
    auto i = qobject_cast<WorldObjectItem*>(sender());
    if(!i) return 0;
    if(i->baseItem()) i = i->baseItem();
    return _objectViews[i];
}

QVector3D WorldView::objectPose(const WorldObject *wo, int wpi) const {
    auto pose = wpi < 0 ? wo->pose() : wo->waypoint(wpi);
    auto pos = mapFromModel(pose.toPointF() - wo->origin());
    return QVector3D(pos.x(), pos.y(), pose.th);
}

//-----------------------------------------------------------------------------

QList<RobotItem*> WorldView::robotList() const {
    QList<RobotItem*> rlist;
    for(auto *r : _model->robots()) {
        rlist.append(static_cast<RobotItem*>(_objectViews[r]->item));
    }
    return rlist;
}

void WorldView::setScale(double value) {
    setTransform(QTransform::fromScale(value, value));
    _viewScale = value;
    for(auto &ov : _objectViews) {
        ov->item->setViewScale(value);
        for(auto *wp : ov->waypoints) wp->setViewScale(value);
    }
}

//=============================================================================

void WorldView::ControlStack::addControl(WorldObjectItem *item, WorldObjectControlItem::ControlType type) {
    auto csi = _controls.find(item);
    if(csi != _controls.end()) {
        for(auto *c : csi.value()) {
            if(c->controlType() == type) return;
        }
    }

    WorldObjectControlItem *control = 0;
    switch(item->type()) {
    case RobotItem::Type:
        control = new RobotControlItem(type, item);
        break;
    case ObstacleItem::Type:
        control = new ObstacleControlItem(type, item);
        break;
    default:
        return;
    }

    control->setZValue(1.0);
    _view->scene()->addItem(control);

    if(csi != _controls.end()) {
        csi.value().last()->stackBefore(control);
        csi.value().last()->setVisible(false);
        csi.value().append(control);
    } else {
        _controls[item].append(control);
    }

    if(type == WorldObjectControlItem::Waypoint) {
        auto *wpc = new WaypointControlItem(item);
        wpc->setZValue(1.0);
        connect(wpc, SIGNAL(updatePathRequested()), _view, SLOT(updatePath()));
        connect(wpc, SIGNAL(removeRequested()), _view, SLOT(removeWaypoint()));
        _view->scene()->addItem(wpc);
        _controls[item].append(wpc);
    }
}

void WorldView::ControlStack::removeControl(WorldObjectItem *item, WorldObjectControlItem::ControlType type) {
    auto csi = _controls.find(item);
    if(csi == _controls.end()) return;

    auto &cl = csi.value();
    for(auto ci = cl.begin(); ci != cl.end();) {
        if((*ci)->controlType() == type) {
            (*ci)->deleteLater();
            ci = cl.erase(ci);
        } else ++ci;
    }

    if(cl.isEmpty()) _controls.remove(item);
    else cl.last()->setVisible(true);
}

void WorldView::ControlStack::removeAllControls(WorldObjectItem *item) {
    auto csi = _controls.find(item);
    if(csi == _controls.end()) return;

    for(auto *c : csi.value()) c->deleteLater();
    _controls.erase(csi);
}

void WorldView::ControlStack::removeAllControls() {
    for(auto &cs : _controls) {
        for(auto *c : cs) c->deleteLater();
    }
    _controls.clear();
}

//=============================================================================

void WorldView::ObjectViewHash::add(WorldObject *object, WorldObjectItem *item) {
    auto v = ObjectViewPtr::create(object, item);
    _objectToView[object] = v;
    _itemToView[item] = v;
}

void WorldView::ObjectViewHash::remove(WorldObject *object) {
    auto v = _objectToView.find(object);
    if(v == _objectToView.end()) return;

    _itemToView.remove(v.value()->item);
    _objectToView.erase(v);
}

void WorldView::ObjectViewHash::clear() {
    _objectToView.clear();
    _itemToView.clear();
}

WorldView::ObjectView *WorldView::ObjectViewHash::operator [](WorldObject *object) const {
    auto v = _objectToView.find(object);
    return v == _objectToView.end() ? 0 : v.value().data();
}

WorldView::ObjectView *WorldView::ObjectViewHash::operator [](WorldObjectItem *item) const {
    auto v = _itemToView.find(item);
    return v == _itemToView.end() ? 0 : v.value().data();
}
