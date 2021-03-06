#ifndef WORLD_ITEMS_H
#define WORLD_ITEMS_H

#include <QGraphicsWidget>
#include <QGraphicsLinearLayout>
#include <QSignalMapper>

#include "world_model.h"
#include "simulator.h"

//=============================================================================

class WorldObjectItem : public QGraphicsObject {
    Q_OBJECT

public:
    WorldObjectItem(QGraphicsItem *parent = 0)
        : QGraphicsObject(parent), _viewScale(1.0), _shapeType(WorldObject::Box), _baseItem(0) {}

    QBrush brush() const { return _brush; }
    Pose pose() const { return Pose(x(), y(), Degrees(rotation())); }

    WorldObject::Shape shapeType() const { return _shapeType; }
    void setShapeType(WorldObject::Shape shape);

    const QPixmap &shapePixmap() const { return _shapePix; }
    void setShapePixmap(const QPixmap &pix);

    double viewScale() const { return _viewScale; }
    void setViewScale(double scale) {
        _viewScale = scale;
        emit viewScaleChanged(scale);
    }

    virtual QSizeF size() const { return boundingRect().size(); }
    virtual void setSize(double w, double h) {}

    virtual QPointF origin() const { return transformOriginPoint(); }
    virtual void setOrigin(double x, double y) {
        setTransformOriginPoint(x, y);
        emit originChanged({x, y});
    }

    QPointF sceneOrigin() const { return mapToScene(origin()); }
    QPointF sceneCenter() const { return mapToScene(boundingRect().center()); }

    WorldObjectItem *baseItem() const { return _baseItem; }
    void setBaseItem(WorldObjectItem *item) { _baseItem = item; }

    virtual QPixmap toPixmap() const { return QPixmap(); }

    static QPen cosmeticPen(const QBrush &color);

public slots:
    void setBrush(const QBrush &brush);
    void setPose(const Pose &p);
    void setOrigin(const QPointF &p);
    void setSize(const QSizeF &size);

signals:
    void poseEdited();
    void originEdited();

    void poseChanged(const Pose &p);
    void originChanged(const QPointF &p);
    void sizeChanged(const QSizeF &size);
    void brushChanged(const QBrush &b);
    void viewScaleChanged(double scale);
    void shapeChanged();

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

    double _viewScale;
    QBrush _brush;

    WorldObject::Shape _shapeType;
    QPixmap _shapePix;

private:
    WorldObjectItem *_baseItem;
    Pose _prevPose;
};

//=============================================================================

class WorldObjectControlItem : public QGraphicsObject {
    Q_OBJECT

public:
    enum ControlType { Shape, Waypoint, TrajectoryBase };

    WorldObjectControlItem(ControlType type, WorldObjectItem *item, QGraphicsItem *parent = 0)
        : QGraphicsObject(parent), _type(type), _item(item) {}

    ControlType controlType() const { return _type; }
    WorldObjectItem *controlledItem() const { return _item; }

    QRectF boundingRect() const { return QRectF(); }
    void paint(QPainter *, const QStyleOptionGraphicsItem *, QWidget *) {}

protected slots:
    void setPose(const Pose &pose);

protected:
    bool sceneEventFilter(QGraphicsItem *watched, QEvent *event);
    QBrush brushForType(ControlType type) const;

    ControlType _type;
    WorldObjectItem *_item;

private:
    QPointF _prevOrigin;
    Pose _prevPose;
};

//=============================================================================

class ObstacleControlItem : public WorldObjectControlItem {
    Q_OBJECT

public:
    ObstacleControlItem(ControlType type, WorldObjectItem *item, QGraphicsItem *parent = 0);

protected:
    bool sceneEventFilter(QGraphicsItem *watched, QEvent *event);
    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

private slots:
    void adjustHandles();
    void adjustOrigin();

private:
    void adjustSize(int handle, const QPointF &delta);

    QGraphicsRectItem *_selection;
    QGraphicsLineItem *_rotAxis;
    QGraphicsEllipseItem *_rotHandle;
    QGraphicsEllipseItem *_origHandle;
    QVector<QGraphicsItem*> _handles;
    QPointF _prevMousePos;
};

//=============================================================================

class RobotControlItem : public WorldObjectControlItem {
public:
    RobotControlItem(ControlType type, WorldObjectItem *item, QGraphicsItem *parent = 0);

protected:
    bool sceneEventFilter(QGraphicsItem *watched, QEvent *event);
    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

private:
    QGraphicsEllipseItem *_selection;
    QGraphicsEllipseItem *_rotHandle;
};

//=============================================================================

class WaypointControlItem : public WorldObjectControlItem {
    Q_OBJECT

public:
    WaypointControlItem(ControlType type, WorldObjectItem *item, QGraphicsItem *parent = 0);

    void addButton(const QString &tooltip, const QIcon &icon, int id);

signals:
    void activated(int id);

private slots:
    void adjustPos();

private:
    QSignalMapper *_mapper;
    QGraphicsLinearLayout *_layout;
    QGraphicsWidget *_control;
};

//=============================================================================

class RobotItem : public WorldObjectItem {
public:
    enum { Type = UserType + 1 };

    RobotItem(QGraphicsItem *parent = 0);

    QSizeF size() const { return _size; }
    void setSize(double w, double h);
    void setCrashed(bool on);

    int type() const { return Type; }
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *);

private:
    QSizeF _size;
    bool _crashed;
};

//=============================================================================

class ObstacleItem : public WorldObjectItem {
public:
    enum { Type = UserType + 2 };

    ObstacleItem(QGraphicsItem *parent = 0);

    QPointF origin() const { return _origin; }
    void setOrigin(double x, double y);
    void setSize(double w, double h);

    QPixmap toPixmap() const;

    int type() const { return Type; }
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

private:
    void paint(QPainter *painter) const;

    double _width, _height;
    QPointF _origin;
};

//=============================================================================

class WorldObjectPathItem : public QGraphicsPathItem {
public:
    enum { Type = UserType + 3 };

    WorldObjectPathItem(WorldObject *object, QGraphicsItem *parent = 0);

    int type() const { return Type; }

    WorldObject *object() const { return _object; }

    void setHighlighted(bool on);

private:
    WorldObject *_object;
    QGraphicsPathItem *_highlight;
};

//=============================================================================

class LaserScanItem : public QGraphicsItem {
public:
    LaserScanItem(QGraphicsItem *parent = 0);

    void setColor(const QColor &color);
    void setLaserScan(const LaserScan &scan);

    QRectF boundingRect() const { return _boundingRect; }
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

private:
    QVector<QPointF> _scan;
    QColor _color;
    QRectF _boundingRect;
};

//=============================================================================

class PointerItem : public QGraphicsItem {
public:
    PointerItem(QGraphicsItem *parent = 0);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};

#endif // WORLD_ITEMS_H
