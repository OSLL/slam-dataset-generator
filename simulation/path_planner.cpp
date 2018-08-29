#include "path_planner.h"

#include <QTransform>
#include <QtMath>

PathPlanner::PathPlanner(QObject *parent) : QObject(parent) {
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
}

PathPlanner::~PathPlanner() {
}

bool PathPlanner::isStateValid(const ompl::base::State *state) const {
    if(_map.isNull()) return false;

    double sx = state->as<ompl::base::SE2StateSpace::StateType>()->getX();
    double sy = state->as<ompl::base::SE2StateSpace::StateType>()->getY();
    double st = state->as<ompl::base::SE2StateSpace::StateType>()->getYaw();

    if(!isValidPixel(sx, sy)) return false;

    // TODO replace with mapToPolygon and proper constraints checking,
    //      current constraints are too strict
    auto t = QTransform::fromTranslate(sx, sy).rotateRadians(st);
    auto robotRect = t.mapRect(_robot);
    for(int y = robotRect.top(); y <= qCeil(robotRect.bottom()); ++y) {
        for(int x = robotRect.left(); x <= qCeil(robotRect.right()); ++x) {
            if(!isValidPixel(x, y)) return false;
            if(qGray(_map.pixel(x, y)) < 127) return false;
        }
    }

    return true;
}

bool PathPlanner::isValidPixel(int x, int y) const {
    return 0 <= x && x < _map.width() && 0 <= y && y < _map.height();
}

Path PathPlanner::makePlan(const Pose &start, const Pose &goal) {
    boost::shared_ptr<ompl::base::SE2StateSpace> space(new ompl::base::SE2StateSpace());
    //        auto space(std::make_shared<ompl::base::SE2StateSpace>());

    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(0, _map.width());
    bounds.setHigh(1, _map.height());
    space->setBounds(bounds);

    ompl::geometric::SimpleSetup ss(space);
    ss.setStateValidityChecker([this](const ompl::base::State *state) { return isStateValid(state); });
    ss.getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());

    ompl::base::ScopedState<> s(space);
    s[0] = start.x, s[1] = start.y, s[2] = start.th.rad();

    ompl::base::ScopedState<> g(space);
    g[0] = goal.x, g[1] = goal.y, g[2] = goal.th.rad();

    ss.setStartAndGoalStates(s, g);

    /*
        for (int i = 0 ; i < 10 ; ++i) {
            if (ss.getPlanner()) ss.getPlanner()->clear();
            ss.solve();
        }
*/

    ss.solve(1.0);

    Path path;
    if(ss.haveSolutionPath()) {
        ss.simplifySolution();
        auto p = ss.getSolutionPath();
        ss.getPathSimplifier()->simplifyMax(p);
//        ss.getPathSimplifier()->smoothBSpline(p);
        for(auto *state : p.getStates()) {
            auto st = state->as<ompl::base::SE2StateSpace::StateType>();
            path.append(Pose(st->getX(), st->getY(), Radians(st->getYaw())));
        }
    }

    return path;
}

void PathPlanner::setMap(const QPixmap &map) {
    _map = map.toImage();
}

void PathPlanner::setRobotSize(const QSizeF &size) {
    _robot = QRectF(-size.width() / 2.0, -size.height() / 2.0,
                     size.width(), size.height());
}
