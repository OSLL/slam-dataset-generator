#include "path_planner.h"

#include <QPainter>
#include <QTransform>

IntegralImage::IntegralImage(const QPixmap &pix) {
    auto img = pix.toImage().convertToFormat(QImage::Format_ARGB32);
    if(img.isNull()) return;

    _w = img.width();
    _h = img.height();
    _data.resize(img.byteCount() / 4);

    auto in = img.constBits();
    for(int i = 0, j = 0, y = 0; y < _h; ++y) {
        quint32 sum = 0;
        for(int x = 0; x < _w; ++x, i +=4, ++j) {
            uchar val;
            if(in[i + 3] < 127) val = 0;
            else val = qGray(in[i], in[i + 1], in[i + 2]) < 127 ? 1 : 0;
            sum += val;
            _data[j] = sum + (y > 0 ? _data[j - _w] : 0);
        }
    }
}

quint32 IntegralImage::pixelSum(const QRect &r) const {
    auto a = r.top() * _w + r.left();
    auto b = r.top() * _w + r.right();
    auto c = r.bottom() * _w + r.left();
    auto d = r.bottom() * _w + r.right();
    return _data[d] + _data[a] - _data[b] - _data[c];
}

//=============================================================================

PathPlanner::PathPlanner(QObject *parent) : QObject(parent) {
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
}

PathPlanner::~PathPlanner() {
}

Path PathPlanner::makePlan(const Pose &start, const Pose &goal) {
    if(_map.isNull()) return Path();

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

    _time = 0;
    _count = 0;
    ss.solve(1.0);
    if(_count > 0) {
        qDebug() << "mean validity check time:" << (_time / (double)_count) << "ns";
    }

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
    auto img = map.toImage().convertToFormat(QImage::Format_ARGB32);
    if(img.isNull()) {
        _map = QImage();
        return;
    }

    _map = QImage(img.size(), QImage::Format_Grayscale8);
    _mask = QImage(img.size(), QImage::Format_Grayscale8);
    _mask.fill(Qt::black);

    auto *in = img.constBits();
    for(int i = 0, y = 0; y < img.height(); ++y) {
        auto *d = _map.scanLine(y);
        for(int x = 0; x < img.width(); ++x, i += 4) {
            if(in[i + 3] < 127) d[x] = 0;
            else d[x] = qGray(in[i], in[i + 1], in[i + 2]) < 127 ? 0 : 255;
        }
    }
}

void PathPlanner::setObjectShape(const QSizeF &size, const QPointF &origin) {
    _objectRect = QRectF(QPointF(0, 0), size);
    _objectOrigin = origin;
}

bool PathPlanner::isStateValid(const ompl::base::State *state) const {
    _timer.start();

    double sx = state->as<ompl::base::SE2StateSpace::StateType>()->getX();
    double sy = state->as<ompl::base::SE2StateSpace::StateType>()->getY();
    double st = state->as<ompl::base::SE2StateSpace::StateType>()->getYaw();

    auto o = QTransform().rotateRadians(st).map(_objectOrigin);
    auto t = QTransform::fromTranslate(sx - o.x(), sy - o.y()).rotateRadians(st);
    auto r = t.mapRect(_objectRect);

    auto maxRect = QRect(QPoint(r.left(), r.top()),
                         QPoint(qCeil(r.right()), qCeil(r.bottom())));
    maxRect.adjust(-1, -1, 1, 1);

    if(!isValidPixel(maxRect.topLeft()) ||
            !isValidPixel(maxRect.bottomRight())) return false;

    generateMask(t, maxRect);
    bool res = checkMask(maxRect);

    _time += _timer.nsecsElapsed();
    ++_count;

    return res;
}

bool PathPlanner::isValidPixel(int x, int y) const {
    return 0 <= x && x < _map.width() && 0 <= y && y < _map.height();
}

void PathPlanner::generateMask(const QTransform &t, const QRectF &r) const {
    QPainter p(&_mask);
    p.setPen(Qt::NoPen);
    p.setBrush(Qt::white);
    p.fillRect(r, Qt::black);
    p.setTransform(t);
    p.drawRect(_objectRect);
    p.end();
}

bool PathPlanner::checkMask(const QRect &r) const {
    for(int y = r.top(); y <= r.bottom(); ++y) {
        auto *maskData = _mask.constScanLine(y);
        auto *mapData = _map.constScanLine(y);
        for(int x = r.left(); x <= r.right(); ++x) {
            if(maskData[x] == 0) continue;
            else if(mapData[x] != 255) return false;
        }
    }
    return true;
}
