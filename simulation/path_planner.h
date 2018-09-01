#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <QObject>
#include <QPixmap>
#include <QVector>
#include <QElapsedTimer>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include "data_structures.h"

class IntegralImage {
public:
    IntegralImage(const QPixmap &pix);

    bool isNull() const { return _data.isEmpty(); }

    quint32 pixelSum(const QRect &r) const;

private:
    QVector<quint32> _data;
    int _w, _h;
};

//=============================================================================

class PathPlanner : public QObject {
public:
    PathPlanner(QObject *parent = 0);
    ~PathPlanner();

    Path makePlan(const Pose &start, const Pose &goal);

    void setMap(const QPixmap &map);
    void setObjectShape(const QSizeF &size, const QPointF &origin);

private:
    bool isStateValid(const ompl::base::State *state) const;
    bool isValidPixel(int x, int y) const;
    bool isValidPixel(const QPoint &p) const {
        return isValidPixel(p.x(), p.y());
    }

    void generateMask(const QTransform &t, const QRectF &r) const;
    bool checkMask(const QRect &r) const;

    QRectF _objectRect;
    QPointF _objectOrigin;
    QImage _map;
    mutable QImage _mask;

    mutable QElapsedTimer _timer;
    mutable quint64 _time, _count;
};

#endif // PATH_PLANNER_H
