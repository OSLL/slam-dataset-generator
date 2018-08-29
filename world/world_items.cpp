#include "world_items.h"

#include <QPainter>
#include <QPushButton>
#include <QGraphicsScene>
#include <QGraphicsProxyWidget>
#include <QGraphicsLinearLayout>
#include <QGraphicsSceneMouseEvent>
#include <QtMath>

//=============================================================================

void WorldObjectItem::setShapeType(WorldObject::Shape shape) {
    if(_shapeType == shape) return;
    _shapeType = shape;
    update();
    emit shapeChanged();
}

void WorldObjectItem::setShapePixmap(const QPixmap &pix) {
    _shapePix = pix;
    if(_shapeType == WorldObject::CustomShape) {
        update();
        emit shapeChanged();
    }
}

void WorldObjectItem::setBrush(const QBrush &brush) {
    if(_brush == brush) return;
    _brush = brush;
    update();
    emit brushChanged(_brush);
}

void WorldObjectItem::setPose(const Pose &p) {
    setPos(p.x, p.y);
    setRotation(p.th.deg());
}

void WorldObjectItem::setOrigin(const QPointF &p) {
    setOrigin(p.x(), p.y());
}

void WorldObjectItem::setSize(const QSizeF &size) {
    setSize(size.width(), size.height());
}

void WorldObjectItem::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    _prevPose = pose();
    QGraphicsObject::mousePressEvent(event);
}

void WorldObjectItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    if(_prevPose != pose()) emit poseEdited();
    QGraphicsObject::mouseReleaseEvent(event);
}

QVariant WorldObjectItem::itemChange(GraphicsItemChange change, const QVariant &value) {
    switch(change) {
    case ItemPositionHasChanged:
    case ItemRotationHasChanged:
        emit poseChanged(pose());
        break;
    default:
        break;
    }
    return QGraphicsObject::itemChange(change, value);
}

QPen WorldObjectItem::cosmeticPen(const QBrush &color) {
    QPen p(color, 1.5, Qt::DashLine);
    QVector<double> dp; dp << 5 << 2.5;
    p.setCosmetic(true);
    p.setDashPattern(dp);
    return p;
}

//=============================================================================

void WorldObjectControlItem::setPose(const Pose &pose) {
    setPos(pose.x, pose.y);
    setRotation(pose.th.deg());
}

QBrush WorldObjectControlItem::brushForType(ControlType type) const {
    switch(type) {
    case Shape: return Qt::yellow;
    case Waypoint: return Qt::green;
    case TrajectoryBase: return Qt::red;
    default: break;
    }
    return QBrush();
}

bool WorldObjectControlItem::sceneEventFilter(QGraphicsItem *watched, QEvent *event) {
    switch(event->type()) {
    case QEvent::GraphicsSceneMousePress:
        _prevOrigin = _item->origin();
        _prevPose = _item->pose();
        break;
    case QEvent::GraphicsSceneMouseRelease:
        if(_prevOrigin != _item->origin()) emit _item->originEdited();
        if(_prevPose != _item->pose()) emit _item->poseEdited();
        break;
    default:
        break;
    }
    return QGraphicsObject::sceneEventFilter(watched, event);
}

//=============================================================================

ObstacleControlItem::ObstacleControlItem(ControlType type, WorldObjectItem *item, QGraphicsItem *parent)
    : WorldObjectControlItem(type, item, parent), _selection(0), _rotAxis(0), _rotHandle(0), _origHandle(0)
{
    connect(item, SIGNAL(poseChanged(Pose)), this, SLOT(setPose(Pose)));
    connect(item, SIGNAL(sizeChanged(QSizeF)), this, SLOT(adjustHandles()));
    connect(item, SIGNAL(originChanged(QPointF)), this, SLOT(adjustOrigin()));
    connect(item, SIGNAL(viewScaleChanged(double)), this, SLOT(adjustOrigin()));

    auto baseBrush = brushForType(type);
    auto spen = WorldObjectItem::cosmeticPen(baseBrush);

    _selection = new QGraphicsRectItem(boundingRect(), this);
    _selection->setPen(spen);

    if(type != TrajectoryBase) {
        _rotAxis = new QGraphicsLineItem(this);
        _rotAxis->setPen(spen);

        _rotHandle = new QGraphicsEllipseItem(-3.5, -3.5, 7, 7, this);
        _rotHandle->setFlags(ItemIsMovable | ItemIgnoresTransformations);
        _rotHandle->setBrush(baseBrush);

        _origHandle = new QGraphicsEllipseItem(-3.5, -3.5, 7, 7, this);
        _origHandle->setFlags(ItemIgnoresTransformations);
        _origHandle->setBrush(baseBrush);
    }

    if(type == Shape) {
        _handles.resize(9);
        for(int i = 0; i < 9; ++i) {
            QAbstractGraphicsShapeItem *h = 0;
            if(i == 4) h = _origHandle;
            else h = new QGraphicsRectItem(-3.5, -3.5, 7, 7, this);
            h->setFlags(ItemIsMovable | ItemIgnoresTransformations);
            h->setBrush(baseBrush);
            _handles[i] = h;
        }
    }

    setPose(_item->pose());
    adjustHandles();
}

bool ObstacleControlItem::sceneEventFilter(QGraphicsItem *watched, QEvent *event) {
    switch(event->type()) {
    case QEvent::GraphicsSceneMousePress:
        _prevMousePos = static_cast<QGraphicsSceneMouseEvent*>(event)->scenePos();
        break;
    case QEvent::GraphicsSceneMouseMove: {
        auto p = static_cast<QGraphicsSceneMouseEvent*>(event)->scenePos();
        if(watched == _rotHandle) {
            // hack to avoid usage of transformOriginPoint since Qt handles it oddly
            auto sp = mapToScene(_item->origin());
            _item->setRotation(qRadiansToDegrees(qAtan2(p.y() - sp.y(), p.x() - sp.x())) + 90);
            auto d = sp - mapToScene(_item->origin());
            _item->moveBy(d.x(), d.y());
        } else if(_type == Shape) {
            int i = _handles.indexOf(watched);
            if(i < 0) break;
            if(i == 4) _item->setOrigin(mapFromScene(p));
            else adjustSize(i, p - _prevMousePos);
        }
        _prevMousePos = p;
        return true;
    }
    default:
        break;
    }
    return WorldObjectControlItem::sceneEventFilter(watched, event);
}

QVariant ObstacleControlItem::itemChange(GraphicsItemChange change, const QVariant &value) {
    if(change == ItemSceneHasChanged) {
        if(_type != TrajectoryBase) _rotHandle->installSceneEventFilter(this);
        for(auto *h : _handles) h->installSceneEventFilter(this);
    }
    return QGraphicsObject::itemChange(change, value);
}

void ObstacleControlItem::adjustHandles() {
    auto rect = _item->boundingRect();
    if(_type == Shape) {
        for(int i = 0; i < 9; ++i) {
            if(i == 4) continue;
            _handles[i]->setPos((i % 3) * rect.width() / 2.0,
                                (i / 3) * rect.height() / 2.0);
        }
    }
    _selection->setRect(rect);
    adjustOrigin();
}

void ObstacleControlItem::adjustOrigin() {
    if(_type == TrajectoryBase) return;
    _origHandle->setPos(_item->origin());
    _rotHandle->setPos(_item->origin().x(), -15.0 / _item->viewScale());
    _rotAxis->setLine(QLineF(_origHandle->pos(), _rotHandle->pos()));
}

void ObstacleControlItem::adjustSize(int handle, const QPointF &delta) {
    if(handle < 0 || handle > 8) return;

    double width = _item->boundingRect().width();
    double height = _item->boundingRect().height();

    // hack to avoid long switch
    double wc = (handle % 3) - 1;
    double xc = handle % 3 ? 0 : 1;

    double hc = handle / 3 - 1;
    double yc = handle < 3 ? 1 : 0;

    double txc = _item->origin().x() / width;
    double tyc = _item->origin().y() / height;

    QTransform t;
    t.rotate(-rotation());
    auto d = t.map(delta);

    if(width  + wc * d.x() >= 1.0) width  += wc * d.x();
    if(height + hc * d.y() >= 1.0) height += hc * d.y();

    auto originOffset = _item->sceneOrigin() - mapToScene({txc * width, tyc * height});
    auto fullOffset = mapToScene(xc * d.x(), yc * d.y()) - mapToScene(0, 0);
    auto offset = fullOffset - originOffset;

    _item->setSize(width, height);
    _item->moveBy(offset.x(), offset.y());
}

//=============================================================================

RobotControlItem::RobotControlItem(ControlType type, WorldObjectItem *item, QGraphicsItem *parent)
    : WorldObjectControlItem(type, item, parent), _rotHandle(0)
{
    connect(item, SIGNAL(poseChanged(Pose)), this, SLOT(setPose(Pose)));

    auto baseBrush = brushForType(type);
    double r = _item->boundingRect().width();

    _selection = new QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r, this);
    _selection->setPen(WorldObjectItem::cosmeticPen(baseBrush));

    if(type != TrajectoryBase) {
        _rotHandle = new QGraphicsEllipseItem(-3.5, -3.5, 7, 7, this);
        _rotHandle->setFlags(ItemIsMovable | ItemIgnoresTransformations);
        _rotHandle->setPos(r, 0);
        _rotHandle->setBrush(baseBrush);
    }

    setPose(_item->pose());
}

QVariant RobotControlItem::itemChange(GraphicsItemChange change, const QVariant &value) {
    if(change == ItemSceneHasChanged && _type != TrajectoryBase) {
        _rotHandle->installSceneEventFilter(this);
    }
    return QGraphicsObject::itemChange(change, value);
}

bool RobotControlItem::sceneEventFilter(QGraphicsItem *watched, QEvent *event) {
    if(event->type() == QEvent::GraphicsSceneMouseMove && watched == _rotHandle) {
        QPointF sp = scenePos();
        QPointF ep = static_cast<QGraphicsSceneMouseEvent*>(event)->scenePos();
        _item->setRotation(qRadiansToDegrees(qAtan2(ep.y() - sp.y(), ep.x() - sp.x())));
        return true;
    }
    return WorldObjectControlItem::sceneEventFilter(watched, event);
}

//=============================================================================

WaypointControlItem::WaypointControlItem(ControlType type, WorldObjectItem *item, QGraphicsItem *parent)
    : WorldObjectControlItem(type, item, parent)
{
    connect(item, SIGNAL(poseChanged(Pose)), this, SLOT(adjustPos()));
    connect(item, SIGNAL(viewScaleChanged(double)), this, SLOT(adjustPos()));

    setFlag(ItemIgnoresTransformations);

    _mapper = new QSignalMapper(this);
    connect(_mapper, SIGNAL(mapped(int)), this, SIGNAL(activated(int)));

    _control = new QGraphicsWidget(this);
    connect(_control, SIGNAL(geometryChanged()), this, SLOT(adjustPos()));

    _layout = new QGraphicsLinearLayout(Qt::Vertical);
    _layout->setContentsMargins(0, 0, 0, 0);
    _layout->setSpacing(2);
    _control->setLayout(_layout);
}

void WaypointControlItem::addButton(const QString &tooltip, const QIcon &icon, int id) {
    auto *btn = new QPushButton();
    btn->setIcon(icon);
    btn->setIconSize({12, 12});
    btn->setFixedSize(16, 16);
    btn->setToolTip(tooltip);

    _mapper->setMapping(btn, id);
    connect(btn, SIGNAL(clicked(bool)), _mapper, SLOT(map()));

    auto *pw = new QGraphicsProxyWidget(this);
    pw->setWidget(btn);

    _layout->addItem(pw);
}

void WaypointControlItem::adjustPos() {
    switch(_item->type()) {
    case RobotItem::Type:
        setPos(_item->scenePos() - QPointF(36, 16) / _item->viewScale());
        break;
    case ObstacleItem::Type:
        setPos(_item->scenePos() - QPointF(_control->boundingRect().width() + 5, 1) / _item->viewScale());
        break;
    default:
        return;
    }
}

//=============================================================================

RobotItem::RobotItem(QGraphicsItem *parent) : WorldObjectItem(parent), _crashed(false) {
    setFlags(ItemIsSelectable | ItemIsMovable | ItemSendsGeometryChanges);
    setBrush(QColor(135, 206, 250));
}

void RobotItem::setSize(double w, double h) {
    emit sizeChanged({w, h});
}

void RobotItem::setCrashed(bool on) {
    _crashed = on;
    update();
}

QRectF RobotItem::boundingRect() const {
    return QRectF(-5, -5, 15, 10);
}

void RobotItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *) {
    QPolygonF p;
    p << QPointF(-5, -5) << QPointF(10, 0) << QPointF(-5, 5);
    painter->setPen(QPen(_brush.color().darker(500), 1));
    painter->setBrush(_brush);
    painter->drawPolygon(p);
    painter->drawLine(0, 0, -5, -5);
    painter->drawLine(0, 0, -5, 5);
    painter->drawLine(0, 0, 10, 0);

    if(_crashed) {
        auto r = boundingRect();
        auto x = r.width() / 2.0 + r.left();
        auto y = r.height() / 2.0;
        painter->setPen(QPen(Qt::red, 2));
        painter->drawLine(x - y, -y, x + y, +y);
        painter->drawLine(x - y, +y, x + y, -y);
    }
}

//=============================================================================

ObstacleItem::ObstacleItem(QGraphicsItem *parent)
    : WorldObjectItem(parent), _width(20), _height(20), _origin(_width / 2.0, _height / 2.0)
{
    setFlags(ItemIsSelectable | ItemIsMovable | ItemSendsGeometryChanges);
    setBrush(Qt::blue);
}

void ObstacleItem::setOrigin(double x, double y) {
    _origin = {x, y};
    emit originChanged(_origin);
}

void ObstacleItem::setSize(double w, double h) {
    prepareGeometryChange();

    double ox = _origin.x() / _width  * w;
    double oy = _origin.y() / _height * h;
    auto d = mapToScene(_origin) - mapToScene(ox, oy);

    _origin = { ox, oy };
    _width = w;
    _height = h;

    moveBy(d.x(), d.y());
    update();

    emit originChanged(_origin);
    emit sizeChanged({w, h});
}

QPixmap ObstacleItem::toPixmap() const {
    QPixmap pix(_width, _height);
    pix.fill(Qt::white);

    QPainter p(&pix);
    paint(&p);
    p.end();
    return pix;
}

QRectF ObstacleItem::boundingRect() const {
    return QRectF(0, 0, _width, _height);
}

void ObstacleItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    paint(painter);
}

void ObstacleItem::paint(QPainter *painter) const {
    painter->setBrush(_brush);
    painter->setPen(Qt::NoPen);
    switch(_shapeType) {
    case WorldObject::Box:
        painter->drawRect(boundingRect());
        break;
    case WorldObject::Ellipse:
        painter->drawEllipse(boundingRect());
        break;
    case WorldObject::CustomShape:
        painter->drawPixmap(boundingRect(), _shapePix, _shapePix.rect());
        break;
    default:
        return;
    }
}

//=============================================================================

WorldObjectPathItem::WorldObjectPathItem(WorldObject *object, QGraphicsItem *parent)
    : QGraphicsPathItem(parent), _object(object), _highlight(0) {}

void WorldObjectPathItem::setHighlighted(bool on) {
    if(!on) {
        delete _highlight;
        _highlight = 0;
        return;
    }

    if(_highlight) return;

    QPen p(Qt::yellow, 2.5);
//    p.setCosmetic(true);

    _highlight = new QGraphicsPathItem(path(), this);
    _highlight->setFlag(ItemStacksBehindParent);
    _highlight->setOpacity(0.5);
    _highlight->setPen(p);
}

//=============================================================================

LaserScanItem::LaserScanItem(QGraphicsItem *parent) : QGraphicsItem(parent), _color(Qt::green) {
}

void LaserScanItem::setColor(const QColor &color) {
    _color = color;
    update();
}

void LaserScanItem::setLaserScan(const LaserScan &scan) {
    prepareGeometryChange();

    _scan.clear();
    _scan.reserve(scan.ranges.size());
    double a = scan.minAngle;
    for(auto &r : scan.ranges) {
        if(r > scan.minRange && r < scan.maxRange) {
            _scan.append({r * qCos(a + scan.pose.th.rad()),
                         -r * qSin(a + scan.pose.th.rad())});
        }
        a += scan.angleIncrement;
    }

    _boundingRect = QPolygonF(_scan).boundingRect();

    update();
}

void LaserScanItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *) {
    painter->setPen(_color);
    painter->drawPoints(_scan.constData(), _scan.size());
}

//=============================================================================

PointerItem::PointerItem(QGraphicsItem *parent) : QGraphicsItem(parent) {
    setFlag(ItemIgnoresTransformations);
}

QRectF PointerItem::boundingRect() const {
    return QRectF(-5, -5, 10, 10);
}

void PointerItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    QPen p(Qt::black, 1.2);
    p.setCosmetic(true);

    painter->setPen(p);
    painter->drawEllipse({0, 0}, 2.5, 2.5);
    painter->drawLine(-5, 0, -2.5, 0);
    painter->drawLine(5, 0, 2.5, 0);
    painter->drawLine(0, 5, 0, 2.5);
    painter->drawLine(0, -5, 0, -2.5);
}
