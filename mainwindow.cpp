#include "mainwindow.h"
#include "project_file.h"

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

    _toolMap[SimulationToolBox::Navigation]    = WorldView::Navigation;
    _toolMap[SimulationToolBox::AddRobot]      = WorldView::AddRobot;
    _toolMap[SimulationToolBox::AddObstacle]   = WorldView::AddObstacle;
    _toolMap[SimulationToolBox::SetTrajectory] = WorldView::SetTrajectory;

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
    connect(_sim, SIGNAL(positionChanged(QString,Pose)), _view, SLOT(setObjectPosition(QString,Pose)));
    connect(_sim, SIGNAL(scanChanged(LaserScan)), _view, SLOT(setLaserScan(LaserScan)));
    connect(_sim, SIGNAL(robotCrashed(bool)), _view, SLOT(setRobotCrashed(bool)));
    connect(_sim, SIGNAL(simulationProgress(int)), _simControl, SLOT(setProgress(int)));
    connect(_sim, SIGNAL(simulationFinished()), this, SLOT(finishSimulation()));

    _objConfig = new WorldObjectConfigWidget(_model, this);
    _objConfig->hide();
    connect(_view, SIGNAL(objectSelected(WorldObject*)), _objConfig, SLOT(setObject(WorldObject*)));
    connect(_objConfig, SIGNAL(removeRequested()), _view, SLOT(removeSelectedItems()));

    _simTools = new SimulationToolBox(this);
    connect(_simTools, SIGNAL(toolChanged(int)), this, SLOT(setViewInteractionMode(int)));
    connect(_view, SIGNAL(changeTool(int)), this, SLOT(setActiveTool(int)));
//    connect(_view, SIGNAL(changeTool(SimulationToolBox::Tool)), _simTools, SLOT(enableTool(SimulationToolBox::Tool)));
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

void MainWindow::loadProject(const QString &fileName) {
    ProjectFile project;

    switch(project.load(fileName)) {
    case ProjectFile::IOError:
        QMessageBox::critical(this, "Load project", "Unable to unpack file: " + fileName);
        return;
    case ProjectFile::Corrupted:
        QMessageBox::critical(this, "Load project", "Unable to load project: file corrupted (" + fileName + ")");
        return;
    default:
        break;
    }

    auto *settings = project.settings();
    if(!settings) {
        QMessageBox::critical(this, "Load project", "Unable to load project: " + fileName);
        return;
    }

    settings->beginGroup("Simulation");
    _simConfig->load(*settings);
    settings->endGroup();

    settings->beginGroup("Model");
    if(!_model->load(project)) {
        _view->setEnabled(false);
        _notifications->showWarning("<b>Unable to load project</b>");
        QMessageBox::critical(this, "Load project", "Unable to load project: file corrupted (" + fileName + ")");
        return;
    }
    settings->endGroup();

    _currentProject = fileName;
    _notifications->showInfo("<b>Project loaded:</b> " + _currentProject);
    _view->setEnabled(true);
}

void MainWindow::moveEvent(QMoveEvent *event) {
    auto d = event->pos() - event->oldPos();
    _objConfig->move(_objConfig->pos() + d);
    QMainWindow::moveEvent(event);
}

void MainWindow::openProject() {
    auto fileName = QFileDialog::getOpenFileName(this, "Open project", "", "Dataset generator project (*.dgp)");
    if(!fileName.isEmpty()) loadProject(fileName);
}

void MainWindow::saveProject() {
    if(_currentProject.isEmpty()) {
        saveProjectAs();
        return;
    }

    ProjectFile project;

    auto *settings = project.settings();
    if(!settings) {
        QMessageBox::critical(this, "Save project", "Unable to write file: " + _currentProject);
        return;
    }

    settings->beginGroup("Simulation");
    _simConfig->save(*settings);
    settings->endGroup();

    settings->beginGroup("Model");
    if(!_model->save(project)) {
        QMessageBox::critical(this, "Save project", "Unable to write file: " + _currentProject);
        return;
    }
    settings->endGroup();

    if(project.save(_currentProject) != ProjectFile::NoError) {
        QMessageBox::critical(this, "Save project", "Unable to write file: " + _currentProject);
        return;
    }

    _notifications->showInfo("<b>Project saved:</b> " + _currentProject);
}

void MainWindow::saveProjectAs() {
    auto fileName = QFileDialog::getSaveFileName(this, "Save project", "", "Dataset generator project (*.dgp)");
    if(fileName.isEmpty()) return;
    else if(!fileName.endsWith(".dgp")) fileName += ".dgp";

    _currentProject = fileName;
    saveProject();
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

void MainWindow::setViewInteractionMode(int tool) {
    auto t = _toolMap.find(tool);
    if(t != _toolMap.end()) _view->setInteractionMode(static_cast<WorldView::InteractionMode>(t.value()));
}

void MainWindow::setActiveTool(int mode) {
    int tool = _toolMap.key(mode, -1);
    if(tool != -1) _simTools->enableTool(static_cast<SimulationToolBox::Tool>(tool));
}

void MainWindow::createMenus() {
    QMenu *fileMenu = new QMenu("File", this);
    fileMenu->addAction("Open project ...", this, SLOT(openProject()))
            ->setShortcut(QKeySequence::Open);
    fileMenu->addSeparator();
    fileMenu->addAction("Save project", this, SLOT(saveProject()))
            ->setShortcut(QKeySequence::Save);
    fileMenu->addAction("Save project as ...", this, SLOT(saveProjectAs()));
    fileMenu->addSeparator();
    fileMenu->addAction("Exit", this, SLOT(close()))
            ->setShortcut(QKeySequence::Quit);
    menuBar()->addMenu(fileMenu);

    QMenu *editMenu = new QMenu("Edit", this);
    editMenu->addAction("Set map ...", this, SLOT(openMap()))
            ->setShortcut(Qt::CTRL + Qt::Key_M);
    menuBar()->addMenu(editMenu);
}
