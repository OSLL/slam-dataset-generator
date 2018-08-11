#include "mainwindow.h"

#include <QMenuBar>
#include <QMenu>
#include <QPixmap>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QFileDialog>
#include <QToolBar>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    createMenus();

    _model = new WorldModel(this);

    _simControl = new SimulationControl(this);
    _simControl->setSimulateEnabled(false);
    connect(_simControl, SIGNAL(simulate()), this, SLOT(simulate()));
    connect(_simControl, SIGNAL(cancel()), this, SLOT(cancelSimulation()));

    _view = new WorldView(_model, this);
    _view->setEnabled(false);
    connect(_view, SIGNAL(simulationAvailabilityChanged(bool)), _simControl, SLOT(setSimulateEnabled(bool)));
    connect(_view, SIGNAL(objectSelected(WorldObject*)), this, SLOT(showObjectConfigDialog(WorldObject*)));

    _simConfig = new SimulatorConfig(this);
    connect(_simConfig, SIGNAL(settingsUpdated(SimulatorConfig::Options)),
            this, SLOT(updateSimulationSettings(SimulatorConfig::Options)));

    _notifications = new NotificationsWidget(this);
    _notifications->showInfo("<b>Load</b> world map (" + QKeySequence(QKeySequence::Open).toString() + ")");
    connect(_view, SIGNAL(message(NotificationsWidget::MessageType,QString)),
            _notifications, SLOT(showMessage(NotificationsWidget::MessageType,QString)));

    _sim = new Simulator(_model, this);
    connect(_sim, SIGNAL(positionChanged(QString,QVector3D)), _view, SLOT(setObjectPosition(QString,QVector3D)));
    connect(_sim, SIGNAL(scanChanged(LaserScan)), _view, SLOT(setLaserScan(LaserScan)));
    connect(_sim, SIGNAL(robotCrashed(bool)), _view, SLOT(setRobotCrashed(bool)));
    connect(_sim, SIGNAL(simulationProgress(int)), _simControl, SLOT(setProgress(int)));
    connect(_sim, SIGNAL(simulationFinished()), this, SLOT(finishSimulation()));

    _objConfig = new WorldObjectConfigWidget(_model, this);
    _objConfig->hide();
    connect(_view, SIGNAL(objectSelected(WorldObject*)), _objConfig, SLOT(setObject(WorldObject*)));
    connect(_objConfig, SIGNAL(removeRequested()), _view, SLOT(removeSelectedItems()));

    _simTools = new SimulationToolBox(this);
    connect(_simTools, SIGNAL(toolChanged(SimulationToolBox::Tool)), _view, SLOT(setInteractionMode(SimulationToolBox::Tool)));
    connect(_view, SIGNAL(changeTool(SimulationToolBox::Tool)), _simTools, SLOT(enableTool(SimulationToolBox::Tool)));
    addToolBar(Qt::TopToolBarArea, _simTools);

    QHBoxLayout *sl = new QHBoxLayout();
    sl->setContentsMargins(0, 0, 0, 0);
    sl->setSpacing(5);
    sl->addWidget(_notifications, 1);
//    sl->addStretch(1);
    sl->addWidget(_simControl);

    QWidget *w = new QWidget(this);
    QGridLayout *l = new QGridLayout();
    l->setSpacing(5);
    l->setContentsMargins(5, 5, 5, 5);
    l->addLayout(sl, 0, 0, 1, 2);
    l->addWidget(_view, 1, 0);
    l->addWidget(_simConfig, 1, 1);
    l->setColumnStretch(0, 1);
    w->setLayout(l);

    setCentralWidget(w);
    resize(1024, 700);

    updateSimulationSettings(SimulatorConfig::MapResolution);
    _simTools->enableTool(SimulationToolBox::AddRobot);
}

MainWindow::~MainWindow() {
}

void MainWindow::loadMap(const QString &fileName) {
    if(!_model->setMap(fileName)) {
        _view->setEnabled(false);
        QMessageBox::critical(this, "Load map", "Unable to load file: " + fileName);
        return;
    }

    _simConfig->setMapFileName(fileName);
    _view->setEnabled(true);
    _view->setSimulationMode(false);
    _notifications->showInfo(_view->defaultMessage());
}

void MainWindow::moveEvent(QMoveEvent *event) {
    auto d = event->pos() - event->oldPos();
    _objConfig->move(_objConfig->pos() + d);
    QMainWindow::moveEvent(event);
}

void MainWindow::openMap() {
    /*
    QString formats = "Images (";
    for(auto &f : QImageReader::supportedImageFormats()) {
        formats += "*." + QString::fromLocal8Bit(f) + " ";
    }
    *formats.rstart() = ')';
    * */
    QString fileName = QFileDialog::getOpenFileName(this, "Load map");
    if(!fileName.isEmpty()) loadMap(fileName);
}

void MainWindow::simulate() {
    _notifications->showInfo("Simulation in progress ...");
    _simTools->enableTool(SimulationToolBox::Navigation);
    _view->setSimulationMode(true, _simConfig->isVisualizationEnabled());
    _sim->simulate(_simConfig);
}

void MainWindow::finishSimulation() {
    _notifications->showInfo("Simulation complete");
    _simControl->restoreDefaultState();
    _view->setSimulationMode(false);
}

void MainWindow::cancelSimulation() {
    _notifications->showInfo("Simulation cancelled");
    _sim->stop();
    _view->setSimulationMode(false);
}

void MainWindow::updateSimulationSettings(SimulatorConfig::Options options) {
    if(options & SimulatorConfig::MapResolution) {
        _model->setWorldScale(_simConfig->mapResolution());
    }
}

void MainWindow::showObjectConfigDialog(WorldObject *object) {
    if(object) {
        _objConfig->move(_view->parentWidget()->mapToGlobal(_view->pos()));
        _objConfig->show();
    } else _objConfig->hide();
}

void MainWindow::createMenus() {
    QMenu *fileMenu = new QMenu("File", this);
    fileMenu->addAction("Load map ...", this, SLOT(openMap()))
            ->setShortcut(QKeySequence::Open);
    menuBar()->addMenu(fileMenu);
}
