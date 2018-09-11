#include "widgets.h"

#include <QHBoxLayout>
#include <QResizeEvent>
#include <QFontMetrics>
#include <QColorDialog>
#include <QSignalBlocker>
#include <QStyleOptionFrame>
#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QMenu>

#include "world/world_model.h"
#include "ui_object_config.h"

template<class T> inline SignalBlocker<T> blocked(T *object) {
    return SignalBlocker<T>(object);
}

//=============================================================================

SimulationControl::SimulationControl(QWidget *parent) : QWidget(parent) {
    _simulate = new QPushButton("Simulate", this);
    _simulate->setMinimumWidth(150);
    auto f = _simulate->font();
    f.setBold(true);
    _simulate->setFont(f);

    _cancel = new QPushButton("Cancel", this);
//    _cancel->setFixedWidth(_simulate->width());
    _cancel->hide();

    _progress = new QProgressBar(this);
    _progress->setRange(0, 100);
    _progress->setMinimumWidth(200);
    _progress->hide();

    auto *l = new QHBoxLayout();
    l->setContentsMargins(0, 0, 0, 0);
    l->setSpacing(5);
    l->addWidget(_progress);
    l->addWidget(_cancel);
    l->addWidget(_simulate);
    setLayout(l);

    connect(_simulate, SIGNAL(clicked(bool)), this, SLOT(requestSimulation()));
    connect(_cancel, SIGNAL(clicked(bool)), this, SLOT(requestCancel()));
}

void SimulationControl::setProgress(int value) {
    _progress->setValue(value);
}

void SimulationControl::setSimulateEnabled(bool on) {
    _simulate->setEnabled(on);
}

void SimulationControl::restoreDefaultState() {
    _simulate->show();
    _cancel->hide();
    _progress->hide();
}

void SimulationControl::requestSimulation() {
    _simulate->hide();
    _cancel->show();
    _progress->show();
    _progress->setValue(0);
    emit simulate();
}

void SimulationControl::requestCancel() {
    restoreDefaultState();
    emit cancel();
}

//=============================================================================

SimulationToolBox::SimulationToolBox(QWidget *parent) : QToolBar("Tools", parent) {
    _actionMapper = new QSignalMapper(this);
    connect(_actionMapper, SIGNAL(mapped(int)), this, SLOT(trigger(int)));

#define ADD_ACTION(action, id) \
    _actionMapper->setMapping(action, id); \
    _actions[id] = action; \
    connect(action, SIGNAL(triggered(bool)), _actionMapper, SLOT(map()));

    auto addMenu = new QMenu(this);

    auto addRobot = addMenu->addAction("Robot");
    addRobot->setShortcut(Qt::ALT + Qt::Key_R);
    addRobot->setCheckable(true);
    addRobot->setChecked(true);
    ADD_ACTION(addRobot, AddRobot);

    auto addObstacle = addMenu->addAction("Obstacle");
    addObstacle->setShortcut(Qt::ALT + Qt::Key_O);
    addObstacle->setCheckable(true);
    ADD_ACTION(addObstacle, AddObstacle);

    auto addGroup = new QActionGroup(this);
    addGroup->addAction(addRobot);
    addGroup->addAction(addObstacle);

    auto addObject = addAction(QIcon(":/icons/cube.png"), "Add object");
    addObject->setCheckable(true);
    addObject->setMenu(addMenu);
    ADD_ACTION(addObject, AddObject);

    auto setTraj = addAction(QIcon(":/icons/route.png"), "Set trajectory");
    setTraj->setShortcut(Qt::ALT + Qt::Key_T);
    setTraj->setCheckable(true);
    ADD_ACTION(setTraj, SetTrajectory);

#undef ADD_ACTION

    auto toolsGroup = new QActionGroup(this);
    toolsGroup->addAction(addObject);
    toolsGroup->addAction(setTraj);

    setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
}

void SimulationToolBox::enableTool(SimulationToolBox::Tool tool) {
    auto act = _actions.find(tool);
    if(act != _actions.end()) act.value()->trigger();
    else trigger(tool);
}

void SimulationToolBox::trigger(int id) {
    switch (id) {
    case Navigation:
        _actions[AddObject]->setChecked(false);
        _actions[SetTrajectory]->setChecked(false);
        emit toolChanged(Navigation);
        break;
    case AddObject:
        emit toolChanged(_actions[AddRobot]->isChecked() ? AddRobot : AddObstacle);
        break;
    case AddRobot:
    case AddObstacle:
        _actions[AddObject]->trigger();
        break;
    case SetTrajectory:
        emit toolChanged(SetTrajectory);
        break;
    default:
        break;
    }
}

//=============================================================================

NotificationsWidget::NotificationsWidget(QWidget *parent) : QWidget(parent) {
    _icon = new QLabel(this);
    _icon->setMinimumWidth(16);
    _msg = new QLabel(this);
    _msg->installEventFilter(this);

    auto *l = new QHBoxLayout();
    l->setContentsMargins(0, 0, 0, 0);
    l->setSpacing(5);
    l->addWidget(_icon);
    l->addWidget(_msg, 1);
    setLayout(l);
}

void NotificationsWidget::showInfo(const QString &msg) {
    showMessage(Info, msg);
}

void NotificationsWidget::showWarning(const QString &msg) {
    showMessage(Warning, msg);
}

void NotificationsWidget::showMessage(NotificationsWidget::MessageType type, const QString &msg) {
    switch(type) {
    case Info:
        showMessage(QPixmap(":/icons/info.png"), msg);
        break;
    case Warning:
        showMessage(QPixmap(":/icons/warning.png"), msg);
        break;
    default:
        showMessage(QPixmap(), msg);
    }
}

bool NotificationsWidget::eventFilter(QObject *watched, QEvent *event) {
    if(watched == _msg && event->type() == QEvent::Resize) {
        elideToWidth(static_cast<QResizeEvent*>(event)->size().width());
    }
    return QWidget::eventFilter(watched, event);
}

void NotificationsWidget::showMessage(const QPixmap &icon, const QString &msg) {
    _icon->setPixmap(icon.scaled(16, 16, Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
    _fullMsg = msg;
    elideToWidth(_msg->width());
}

void NotificationsWidget::elideToWidth(int w) {
    _msg->setText(_msg->fontMetrics().elidedText(_fullMsg, Qt::ElideRight, w));
}

//=============================================================================

ColorPicker::ColorPicker(QWidget *parent) : QPushButton(parent), _color(Qt::black) {
    setFlat(true);
    setFixedSize(iconSize());
    connect(this, SIGNAL(clicked(bool)), this, SLOT(edit()));
    updateIcon();
}

void ColorPicker::edit() {
    auto *cd = new QColorDialog(_color);
    if(cd->exec() == QColorDialog::Accepted) setColor(cd->currentColor());
    delete cd;
}

void ColorPicker::setColor(const QColor &color) {
    if(color == _color) return;
    _color = color;
    updateIcon();
    emit colorChanged(color);
}

void ColorPicker::updateIcon() {
    QPixmap pix(iconSize());
    QPainter p(&pix);

    p.fillRect(pix.rect(), _color);

    QStyleOptionFrame fo;
    fo.initFrom(this);
    fo.rect = pix.rect();
    fo.frameShape = QFrame::Panel;
    fo.state = QStyle::State_Sunken;
    fo.lineWidth = 2;
    fo.midLineWidth = 2;
    style()->drawControl(QStyle::CE_ShapedFrame, &fo, &p);

    setIcon(QIcon(pix));
}

//=============================================================================

WorldObjectConfigWidget::WorldObjectConfigWidget(WorldModel *model, QWidget *parent)
    : QWidget(parent, Qt::Tool), _model(model), _object(0), _ui(new Ui::ObjectConfig)
{
    _ui->setupUi(this);
    _ui->_color->setColor(Qt::black);
    connect(_ui->_objectType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateObject()));
    connect(_ui->_driveType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateObject()));
    connect(_ui->_motionType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateObject()));
    connect(_ui->_color, SIGNAL(colorChanged(QColor)), this, SLOT(updateObject()));
    connect(_ui->_sizeX, SIGNAL(valueChanged(double)), this, SLOT(updateObject()));
    connect(_ui->_sizeY, SIGNAL(valueChanged(double)), this, SLOT(updateObject()));
    connect(_ui->_speedAngular, SIGNAL(valueChanged(double)), this, SLOT(updateObject()));
    connect(_ui->_speedLinear, SIGNAL(valueChanged(double)), this, SLOT(updateObject()));
    connect(_ui->_trajEndAction, SIGNAL(currentIndexChanged(int)), this, SLOT(updateObject()));
    connect(_ui->_trajLoopsCount,SIGNAL(valueChanged(int)), this, SLOT(updateObject()));
    connect(_ui->_trajHideAtEnd, SIGNAL(toggled(bool)), this, SLOT(updateObject()));
    connect(_ui->_trajLoopsInf, &QCheckBox::toggled, this, [this]() {
        if(!_ui->_trajLoopsInf->isChecked()) {
            blocked(_ui->_trajLoopsCount)->setValue(1);
        }
        updateObject();
    });

    connect(_ui->_gbOdomNoise, SIGNAL(toggled(bool)), this, SLOT(updateObject()));
    connect(_ui->_odomNoiseLin, SIGNAL(valueChanged(double)), this, SLOT(updateObject()));
    connect(_ui->_odomNoiseAng, SIGNAL(valueChanged(double)), this, SLOT(updateObject()));

    connect(_ui->_shapeType, SIGNAL(currentIndexChanged(int)), this, SLOT(changeShape(int)));
    connect(_ui->_pbSelectShape, SIGNAL(clicked(bool)), this, SLOT(selectShape()));
    connect(_ui->_pbRemove, &QPushButton::clicked, this, [this]() { emit removeObjectRequested(_object); });
    connect(_ui->_pbRemoveTraj, &QPushButton::clicked, this, [this]() { emit removeTrajectoryRequested(_object); });

    connect(_model, SIGNAL(dataChanged(WorldObject*,int,QVariant)),
            this,   SLOT(updateData(WorldObject*,int,QVariant)));
}

WorldObjectConfigWidget::~WorldObjectConfigWidget() {
    delete _ui;
}

void WorldObjectConfigWidget::setObject(WorldObject *object) {
    _object = object;
    if(!object) return;
    for(auto w : findChildren<QWidget*>()) w->blockSignals(true);
    _ui->_objectType->setCurrentIndex(object->type());
    _ui->_shapeType->setCurrentIndex(object->shapeType());
    _ui->_motionType->setCurrentIndex(object->motionType());
    _ui->_driveType->setCurrentIndex(object->driveType());
    _ui->_sizeX->setValue(object->worldSize().width());
    _ui->_sizeY->setValue(object->worldSize().height());
    _ui->_color->setColor(object->brush().color());
    _ui->_speedAngular->setValue(object->angularSpeed());
    _ui->_speedLinear->setValue(object->linearSpeed());
    _ui->_trajEndAction->setCurrentIndex(object->endAction());
    _ui->_trajLoopsInf->setChecked(object->trajectoryLoops() < 0);
    _ui->_trajLoopsCount->setValue(object->trajectoryLoops());
    _ui->_trajHideAtEnd->setChecked(object->hideAtEnd());

    _ui->_motionType->setEnabled(object->type() != WorldObject::Robot);
    _ui->_pbSelectShape->setEnabled(object->type() != WorldObject::Robot);
    _ui->_shapeType->setEnabled(object->type() != WorldObject::Robot);
    _ui->_gbOdomNoise->setVisible(object->type() == WorldObject::Robot);
    _ui->_pbRemoveTraj->setEnabled(!object->paths().isEmpty());

    if(object->type() == WorldObject::Robot) {
        auto *robot = static_cast<RobotObject*>(object);
        _ui->_gbOdomNoise->setEnabled(robot->isOdomNoiseEnabled());
        _ui->_odomNoiseLin->setValue(robot->odomNoiseLinear());
        _ui->_odomNoiseAng->setValue(robot->odomNoiseAngular());
    }

    updateData(_object, WorldModel::MotionRole, object->motionType());
    updateData(_object, WorldModel::TrjEndActionRole, object->endAction());

    for(auto w : findChildren<QWidget*>()) w->blockSignals(false);
}

void WorldObjectConfigWidget::keyPressEvent(QKeyEvent *event) {
    switch(event->key()) {
    case Qt::Key_Escape: hide(); break;
    case Qt::Key_Delete: emit removeObjectRequested(_object); break;
    default: break;
    }
    QWidget::keyPressEvent(event);
}

void WorldObjectConfigWidget::updateObject() {
    if(!_object) return;
    _model->setData(_object, WorldModel::DriveRole,    _ui->_driveType->currentIndex());
    _model->setData(_object, WorldModel::MotionRole,   _ui->_motionType->currentIndex());
    _model->setData(_object, WorldModel::SizeRole,     QSizeF(_ui->_sizeX->value(), _ui->_sizeY->value()));
    _model->setData(_object, WorldModel::SpeedAngRole, _ui->_speedAngular->value());
    _model->setData(_object, WorldModel::SpeedLinRole, _ui->_speedLinear->value());
    _model->setData(_object, WorldModel::BrushRole,    QBrush(_ui->_color->color()));
    _model->setData(_object, WorldModel::TrjEndActionRole, _ui->_trajEndAction->currentIndex());
    _model->setData(_object, WorldModel::TrjLoopCountRole, _ui->_trajLoopsInf->isChecked() ? 0 : _ui->_trajLoopsCount->value());
    _model->setData(_object, WorldModel::TrjHideAtEndRole, _ui->_trajHideAtEnd->isChecked());
    if(_object->type() == WorldObject::Robot) {
        _model->setData(_object, WorldModel::OdomNoiseRole,    _ui->_gbOdomNoise->isEnabled());
        _model->setData(_object, WorldModel::OdomNoiseLinRole, _ui->_odomNoiseLin->value());
        _model->setData(_object, WorldModel::OdomNoiseAngRole, _ui->_odomNoiseAng->value());
    }
}

void WorldObjectConfigWidget::updateData(WorldObject *object, int role, const QVariant &value) {
    if(!_object || object != _object) return;
    switch(role) {
    case WorldModel::SizeRole: {
        auto size = value.toSizeF();
        blocked(_ui->_sizeX)->setValue(size.width());
        blocked(_ui->_sizeY)->setValue(size.height());
        break;
    }
    case WorldModel::MotionRole: {
        auto type = static_cast<WorldObject::Motion>(value.toInt());
        _ui->_driveType->setEnabled(type != WorldObject::Static);
        _ui->_gbSpeed->setEnabled(type != WorldObject::Static);
        _ui->_gbTrajectory->setVisible(type == WorldObject::Trajectory);
        break;
    }
    case WorldModel::TrjEndActionRole: {
        bool withLoops = value.toInt() != WorldObject::StopAtEnd;
        _ui->_trajLoopsInf->setEnabled(withLoops);
        _ui->_trajLoopsCount->setEnabled(withLoops && object->trajectoryLoops() > 0);
        _ui->_trajHideAtEnd->setEnabled(!withLoops || object->trajectoryLoops() > 0);
        break;
    }
    case WorldModel::TrjLoopCountRole: {
        auto val = value.toInt();
        blocked(_ui->_trajLoopsInf)->setChecked(val == 0);
        blocked(_ui->_trajLoopsCount)->setValue(val);
        _ui->_trajLoopsCount->setEnabled(val > 0);
        _ui->_trajHideAtEnd->setEnabled(_object->endAction() == WorldObject::StopAtEnd || val > 0);
        break;
    }
    case WorldModel::WaypointAddedRole:
    case WorldModel::WaypointRemovedRole:
        _ui->_pbRemoveTraj->setEnabled(!_object->paths().isEmpty());
        break;
    default:
        break;
    }
}

void WorldObjectConfigWidget::changeShape(int id) {
    if(id == WorldObject::CustomShape && !_object->hasShapePixmap()) {
        if(!selectShape()) {
            blocked(_ui->_shapeType)->setCurrentIndex(_object->shapeType());
        }
    } else {
        _model->setData(_object, WorldModel::ShapeRole, id);
    }
}

bool WorldObjectConfigWidget::selectShape() {
    auto fn = QFileDialog::getOpenFileName(this, "Select image");
    if(fn.isEmpty()) return false;

    QPixmap pix;
    if(!pix.load(fn)) {
        QMessageBox::warning(this, "Warning", "Unable to load file: " + fn);
        return false;
    }

    blocked(_ui->_shapeType)->setCurrentIndex(WorldObject::CustomShape);
    _model->setData(_object, WorldModel::ShapePixmapRole, pix);
    _model->setData(_object, WorldModel::ShapeRole, WorldObject::CustomShape);
    return true;
}
