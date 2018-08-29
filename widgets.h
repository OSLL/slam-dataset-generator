#ifndef WIDGETS_H
#define WIDGETS_H

#include <QWidget>
#include <QPushButton>
#include <QProgressBar>
#include <QSignalMapper>
#include <QToolBar>
#include <QLabel>
#include <QAction>

// More convenient version of QSignalBlocker
template<class T>
class SignalBlocker {
public:
    SignalBlocker(T *object)
        : _object(object), _prevState(object->blockSignals(true)) {}

    ~SignalBlocker() {
        _object->blockSignals(_prevState);
    }

    T *operator->() { return _object; }

private:
    T *_object;
    bool _prevState;
};

//=============================================================================

class SimulationControl : public QWidget {
    Q_OBJECT

public:
    SimulationControl(QWidget *parent = 0);

public slots:
    void setProgress(int value);
    void setSimulateEnabled(bool on);
    void restoreDefaultState();

signals:
    void simulate();
    void cancel();

private slots:
    void requestSimulation();
    void requestCancel();

private:
    QPushButton *_simulate, *_cancel;
    QProgressBar *_progress;
};

//=============================================================================

class SimulationToolBox : public QToolBar {
    Q_OBJECT

public:
    enum Tool { Navigation, AddObject, AddRobot, AddObstacle, SetTrajectory };

    SimulationToolBox(QWidget *parent = 0);

public slots:
    void enableTool(SimulationToolBox::Tool tool);

signals:
    void toolChanged(int tool);

private slots:
    void trigger(int id);

private:
    QHash<Tool, QAction*> _actions;
    QSignalMapper *_actionMapper;

    QAction *_addObject, *_setTraj;
};

//=============================================================================

class NotificationsWidget : public QWidget {
    Q_OBJECT

public:
    enum MessageType { Info, Warning };

    NotificationsWidget(QWidget *parent = 0);

    void showInfo(const QString &msg);
    void showWarning(const QString &msg);

public slots:
    void showMessage(NotificationsWidget::MessageType type, const QString &msg);

protected:
    bool eventFilter(QObject *watched, QEvent *event);

private:
    void showMessage(const QPixmap &icon, const QString &msg);
    void elideToWidth(int w);

    QString _fullMsg;
    QLabel *_icon, *_msg;
};

//=============================================================================

class ColorPicker : public QPushButton {
    Q_OBJECT

public:
    ColorPicker(QWidget *parent = 0);

    QColor color() const { return _color; }
    void setColor(const QColor &color);

public slots:
    void edit();

signals:
    void colorChanged(const QColor &color);

private:
    void updateIcon();
    QColor _color;
};

//=============================================================================

namespace Ui {
class ObjectConfig;
}

class WorldObject;
class WorldModel;

class WorldObjectConfigWidget : public QWidget {
    Q_OBJECT

public:
    WorldObjectConfigWidget(WorldModel *model, QWidget *parent = 0);
    ~WorldObjectConfigWidget();

public slots:
    void setObject(WorldObject *object);

signals:
    void removeRequested();

protected:
    void keyPressEvent(QKeyEvent *event);

private slots:
    void updateObject();
    void updateData(WorldObject *object, int role, const QVariant &value);

    void changeShape(int id);
    void changeMotionType(int id);
    bool selectShape();

private:
    WorldModel *_model;
    WorldObject *_object;
    Ui::ObjectConfig *_ui;
};

#endif // WIDGETS_H
