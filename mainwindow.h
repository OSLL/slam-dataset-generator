#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "world_view.h"
#include "widgets.h"
#include "simulator.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void loadMap(const QString &fileName);

protected:
    void moveEvent(QMoveEvent *event);

private slots:
    void openMap();
    void simulate();
    void finishSimulation();
    void cancelSimulation();
    void updateSimulationSettings(SimulatorConfig::Options options);
    void showObjectConfigDialog(WorldObject *object);

private:
    void createMenus();

    WorldView *_view;
    WorldModel *_model;
    Simulator *_sim;
    SimulatorConfig *_simConfig;
    SimulationControl *_simControl;
    SimulationToolBox *_simTools;
    NotificationsWidget *_notifications;
    WorldObjectConfigWidget *_objConfig;
};


#endif // MAINWINDOW_H
