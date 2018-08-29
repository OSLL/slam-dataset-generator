#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <QDataStream>
#include <QVector>
#include <QtMath>
#include <QDebug>

class Angle {
    Angle(double val) : _val(val) {}

public:
    Angle() : _val(0) {}

    void setRad(double val) { _val = val; }
    void setDeg(double val) { _val = qDegreesToRadians(val); }

    double rad() const { return _val; }
    double deg() const { return qRadiansToDegrees(_val); }

    Angle normalized() const {
        auto a = std::fmod(_val + M_PI, 2 * M_PI);
        a += a < 0 ? 2 * M_PI : -M_PI;
        return Angle(a);
    }

    void normalize() {
        *this = this->normalized();
    }

    Angle &operator +=(const Angle &other) {
        _val += other._val;
        return *this;
    }

    Angle &operator -=(const Angle &other) {
        _val -= other._val;
        return *this;
    }

    bool operator ==(const Angle &other) const {
        return qFuzzyCompare(this->normalized()._val, other.normalized()._val);
    }

    bool operator !=(const Angle &other) const {
        return !(*this == other);
    }

    Angle operator -() const {
        return Angle(-_val);
    }

private:
    friend QDataStream &operator <<(QDataStream &, const Angle &);
    friend QDataStream &operator >>(QDataStream &, Angle &);
    friend Angle Radians(double);
    friend Angle Degrees(double);

    double _val;
};

inline Angle Radians(double val) { return Angle(val); }
inline Angle Degrees(double val) { return Angle(qDegreesToRadians(val)); }

inline Angle operator+(const Angle &a1, const Angle &a2) {
    return Radians(a1.rad() + a2.rad());
}

inline Angle operator-(const Angle &a1, const Angle &a2) {
    return Radians(a1.rad() - a2.rad());
}

inline QDataStream &operator <<(QDataStream &stream, const Angle &angle) {
    stream << angle._val;
    return stream;
}

inline QDataStream &operator >>(QDataStream &stream, Angle &angle) {
    stream >> angle._val;
    return stream;
}

//=============================================================================

struct Pose {
    Pose() : x(0), y(0), th() {}
    Pose(double x, double y, const Angle &th = Angle()) : x(x), y(y), th(th) {}

    double x, y;
    Angle th;

    QPointF toPointF() const { return QPointF(x, y); }

    bool operator ==(const Pose &other) const {
        return qFuzzyCompare(x, other.x)
                && qFuzzyCompare(y, other.y)
                && th == other.th;
    }

    bool operator !=(const Pose &other) const {
        return !(*this == other);
    }

    Pose &operator +=(const QPointF &point) {
        x += point.x();
        y += point.y();
        return *this;
    }

    operator QVariant() const {
        QVariant v;
        v.setValue(*this);
        return v;
    }
};

inline QDebug operator <<(QDebug dbg, const Pose &pose) {
    QDebugStateSaver state(dbg);
    dbg.nospace() << "Pose(" << pose.x << ", " << pose.y << ", " << pose.th.deg() << ")";
    return dbg;
}

inline QDataStream &operator <<(QDataStream &stream, const Pose &pose) {
    stream << pose.x << pose.y << pose.th;
    return stream;
}

inline QDataStream &operator >>(QDataStream &stream, Pose &pose) {
    stream >> pose.x >> pose.y >> pose.th;
    return stream;
}

inline Pose operator+(const Pose &pose, const QPointF &point) {
    return Pose(pose.x + point.x(), pose.y + point.y(), pose.th);
}

inline Pose operator+(const Pose &p1, const Pose &p2) {
    return Pose(p1.x + p2.x, p1.y + p2.y, p1.th + p2.th);
}

inline Pose operator-(const Pose &p1, const Pose &p2) {
    return Pose(p1.x - p2.x, p1.y - p2.y, p1.th - p2.th);
}

Q_DECLARE_METATYPE(Pose)

//=============================================================================

typedef Pose Speed;

//=============================================================================

struct Path : public QVector<Pose> {
    Path() : QVector<Pose>(), ok(false) {}
    Path(const QVector<Pose> &other, bool ok) : QVector<Pose>(other), ok(ok) {}

    bool ok;
};

//=============================================================================

struct LaserScan {
    Pose pose;
    QVector<double> ranges;
    float minRange, maxRange;
    float minAngle, maxAngle;
    float angleIncrement;
};

#endif // DATA_STRUCTURES_H
