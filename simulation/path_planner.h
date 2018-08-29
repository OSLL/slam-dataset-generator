#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <QObject>
#include <QPixmap>
#include <QImage>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include "data_structures.h"

class PathPlanner : public QObject {
public:
    PathPlanner(QObject *parent = 0);
    ~PathPlanner();

    Path makePlan(const Pose &start, const Pose &goal);

    void setMap(const QPixmap &map);
    void setRobotSize(const QSizeF &size);

private:
    bool isStateValid(const ompl::base::State *state) const;
    bool isValidPixel(int x, int y) const;

    QRectF _robot;
    QImage _map;
};

#endif // PATH_PLANNER_H
