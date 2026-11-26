#include "MainWindow.h"
#include "LoadPlotWidget.h"
#include "PidActorManager.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <QAction>
#include <QActionGroup>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QPixmap>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>  // for animation
#include <QToolBar>
#include <QToolButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include <QVTKOpenGLNativeWidget.h>

#include <vtkActor.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCellPicker.h>
#include <vtkCellType.h>
#include <vtkColorTransferFunction.h>
#include <vtkCommand.h>
#include <vtkCoordinate.h>
#include <vtkDataSetMapper.h>
#include <vtkDoubleArray.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLight.h>
#include <vtkLightKit.h>
#include <vtkLookupTable.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkObject.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkScalarBarActor.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnstructuredGrid.h>

namespace
{
// ---------------------------------------------------------------------------
// PIDs
// ---------------------------------------------------------------------------
namespace Pid
{
// vehicle
constexpr int Cab = 1;
constexpr int Body = 2;
constexpr int Tires = 3;
constexpr int Camera = 4;
constexpr int Suspension = 5;
constexpr int Frame = 6;
// map
constexpr int Terrain = 10;
constexpr int Road = 11;
constexpr int Curbs = 12;
constexpr int Roadside = 13;
constexpr int Scenery = 14;
constexpr int Potholes = 15;
constexpr int Bumps = 16;
// analysis
constexpr int Trail = 20;
constexpr int Arrows = 21;
} // namespace Pid

constexpr int kMaxWheels = VehicleModel::kMaxWheels;
constexpr double kPhysicsDt = 0.0005;    // 0.5 ms physics sub-step
constexpr double kPlannerDecel = 1.4;    // driver braking for slower zones / corners [m/s^2]
constexpr double kTrailStep = 0.4;       // trail station length [m]
constexpr double kTruckFrameZ0 = 0.95;   // bottom of the truck frame rails (body frame)
constexpr int kArrowPoints = 13;
constexpr int kArrowCells = 11;
constexpr int kMiniDiscSegments = 16;

const double kTimeScales[] = { 0.25, 0.5, 1.0, 2.0, 4.0, 8.0 };
const MapType kMaps[] = { MapType::Highway, MapType::City, MapType::National };
const VehicleType kVehicles[] = { VehicleType::Compact, VehicleType::Sedan, VehicleType::SUV, VehicleType::Truck };

const QColor kWheelColors[kMaxWheels] = { QColor(90, 175, 255), QColor(40, 110, 225),
                                          QColor(255, 186, 70), QColor(232, 120, 32),
                                          QColor(130, 225, 130), QColor(60, 168, 92) };

QString mapLabel(MapType t)
{
    switch (t)
    {
    case MapType::Highway: return QString::fromUtf8("Highway  (고속도로)");
    case MapType::City: return QString::fromUtf8("City driving  (시내 주행)");
    case MapType::National: return QString::fromUtf8("National road  (일반 국도)");
    }
    return {};
}

QString vehicleLabel(VehicleType t)
{
    switch (t)
    {
    case VehicleType::Compact: return QString::fromUtf8("Compact car  (경차, ~1.1 t)");
    case VehicleType::Sedan: return QString::fromUtf8("Sedan  (세단, ~1.8 t)");
    case VehicleType::SUV: return QString::fromUtf8("SUV  (~2.2 t)");
    case VehicleType::Truck: return QString::fromUtf8("Truck  (트럭, ~20 t)");
    }
    return {};
}

int mapIndex(MapType t)
{
    for (int i = 0; i < 3; ++i)
        if (kMaps[i] == t)
            return i;
    return 0;
}

int vehicleIndex(VehicleType t)
{
    for (int i = 0; i < 4; ++i)
        if (kVehicles[i] == t)
            return i;
    return 3;
}

bool isBodyPid(int pid)
{
    return pid == Pid::Cab || pid == Pid::Body || pid == Pid::Camera || pid == Pid::Frame;
}

bool isVehicleFramePid(int pid)
{
    return pid == Pid::Tires || pid == Pid::Suspension || pid == Pid::Arrows;
}

double rad2deg(double r)
{
    return vtkMath::DegreesFromRadians(r);
}

vtkSmartPointer<vtkLookupTable> makeLut(
    const std::vector<std::pair<double, std::array<double, 3>>>& stops, double lo, double hi)
{
    vtkNew<vtkColorTransferFunction> ctf;
    for (const auto& [x, c] : stops)
        ctf->AddRGBPoint(x, c[0], c[1], c[2]);

    auto lut = vtkSmartPointer<vtkLookupTable>::New();
    lut->SetNumberOfTableValues(256);
    lut->SetRange(lo, hi);
    lut->Build();
    for (int i = 0; i < 256; ++i)
    {
        double rgb[3];
        ctf->GetColor(lo + (hi - lo) * i / 255.0, rgb);
        lut->SetTableValue(i, rgb[0], rgb[1], rgb[2], 1.0);
    }
    return lut;
}

std::array<double, 3> severityRgb(Severity s)
{
    switch (s)
    {
    case Severity::Low: return { 0.35, 0.80, 0.40 };
    case Severity::Moderate: return { 0.98, 0.84, 0.25 };
    case Severity::High: return { 0.98, 0.55, 0.15 };
    case Severity::Severe: return { 0.95, 0.20, 0.17 };
    }
    return { 1.0, 1.0, 1.0 };
}

QColor toQColor(const std::array<double, 3>& c)
{
    return QColor::fromRgbF(static_cast<float>(c[0]), static_cast<float>(c[1]), static_cast<float>(c[2]));
}

QColor lutColor(vtkLookupTable* lut, double v)
{
    double rgb[3];
    lut->GetColor(v, rgb);
    return QColor::fromRgbF(static_cast<float>(rgb[0]), static_cast<float>(rgb[1]), static_cast<float>(rgb[2]));
}

QColor readableText(const QColor& bg)
{
    return (bg.lightnessF() > 0.55) ? QColor(20, 20, 20) : QColor(245, 245, 245);
}

QString fmtDamage(double d)
{
    if (d <= 0.0)
        return "0";
    if (d < 0.01)
        return QString::number(d, 'e', 1);
    return QString::number(d, 'g', 3);
}

double damageScalar(double d)
{
    return std::log10(std::max(d, 1e-8));
}

void transformPoint(vtkMatrix4x4* m, const double in[3], double out[3], double w = 1.0)
{
    const double a[4] = { in[0], in[1], in[2], w };
    double b[4];
    m->MultiplyPoint(a, b);
    out[0] = b[0];
    out[1] = b[1];
    out[2] = b[2];
}

QTableWidgetItem* centered(const QString& text)
{
    auto* item = new QTableWidgetItem(text);
    item->setTextAlignment(Qt::AlignCenter);
    return item;
}

// passenger car body proportions (vehicle frame: front axle at x = 0)
struct CarShape
{
    Rgb paint;
    double z0, z1, halfW;                 // lower body
    double ghX0b, ghX1b, ghYb;            // greenhouse bottom (at z1)
    double ghX0t, ghX1t, ghYt, roofZ;     // greenhouse top
    bool roofRails;
};

CarShape carShape(VehicleType t)
{
    switch (t)
    {
    case VehicleType::Compact:
        return { makeRgb(236, 190, 40), 0.20, 0.78, 0.80, -2.75, -0.05, 0.76, -2.65, -0.75, 0.66, 1.49, false };
    case VehicleType::SUV:
        return { makeRgb(42, 64, 102), 0.32, 1.02, 0.95, -3.75, -0.35, 0.90, -3.65, -1.05, 0.80, 1.72, true };
    default:  // sedan
        return { makeRgb(176, 180, 190), 0.22, 0.80, 0.93, -3.00, -0.25, 0.88, -2.60, -0.95, 0.70, 1.45, false };
    }
}
} // namespace

// ===========================================================================
// construction
// ===========================================================================

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_pidActorManager(new PidActorManager())
    , m_road(MapType::Highway)
    , m_vehicle(VehicleType::Truck)
{
    // load / static load: purple (lift-off) - blue - green (static) - yellow - orange - red
    m_loadLut = makeLut({ { 0.0, { 0.50, 0.25, 0.80 } },
                          { 0.5, { 0.20, 0.50, 0.95 } },
                          { 1.0, { 0.20, 0.78, 0.35 } },
                          { 1.5, { 0.98, 0.86, 0.20 } },
                          { 2.0, { 0.98, 0.50, 0.12 } },
                          { 3.0, { 0.82, 0.06, 0.06 } } },
                        0.0, 3.0);
    // log10(pseudo-damage)
    m_damageLut = makeLut({ { -6.0, { 0.58, 0.64, 0.60 } },
                            { -3.0, { 0.25, 0.70, 0.40 } },
                            { 0.0, { 0.98, 0.85, 0.20 } },
                            { 1.0, { 0.98, 0.50, 0.12 } },
                            { 2.0, { 0.78, 0.05, 0.10 } } },
                          -6.0, 2.0);

    m_yawMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    m_bodyMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    m_bodyRelMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    m_bodyTrueMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    m_prevYawMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

    m_clock.start();

    setupUi();
    setupVtk();

    resetSimulation();
    setCameraPreset(0);
    renderAll();

    statusBar()->showMessage("Ready - choose a map (Road) and a vehicle (Vehicle), then press Start");
}

MainWindow::~MainWindow()
{
    delete m_pidActorManager;
}

// ===========================================================================
// UI
// ===========================================================================

void MainWindow::setupUi()
{
    setWindowTitle("Drive-vehicle - road hazard load & damage simulation");
    resize(1720, 1040);

    m_vtkWidget = new QVTKOpenGLNativeWidget(this);
#ifdef Q_OS_MACOS
    // macOS trackpad delivers raw touches; VTK maps touch-down to LeftButtonPress,
    // so merely moving the cursor rotated the camera. Use normal mouse events instead.
    m_vtkWidget->setEnableTouchEventProcessing(false);
#endif
    setCentralWidget(m_vtkWidget);

    setupMenus();

    // side docks use the full window height; the bottom dock sits under the 3D view only
    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

    // left: PID tree
    QDockWidget* modelDock = new QDockWidget("PID / Parts", this);
    m_treeWidget = new QTreeWidget(modelDock);
    m_treeWidget->setHeaderLabel("Model");
    m_treeWidget->setMaximumWidth(220);
    modelDock->setWidget(m_treeWidget);
    addDockWidget(Qt::LeftDockWidgetArea, modelDock);

    connect(m_treeWidget, &QTreeWidget::itemChanged,
            this, &MainWindow::onTreeItemChanged);

    connect(m_treeWidget, &QTreeWidget::currentItemChanged,
            this, &MainWindow::onCurrentTreeItemChanged);

    // right: two tabs (setup / results)
    QDockWidget* simDock = new QDockWidget("Simulation", this);
    m_sideTabs = new QTabWidget(simDock);
    auto wrap = [](QWidget* w)
    {
        auto* scroll = new QScrollArea;
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        scroll->setWidget(w);
        return scroll;
    };
    m_sideTabs->addTab(wrap(createDrivePanel()), "Drive && hazards");
    m_sideTabs->addTab(wrap(createLoadsPanel()), "Loads && damage");
    m_sideTabs->setMinimumWidth(400);
    simDock->setWidget(m_sideTabs);
    addDockWidget(Qt::RightDockWidgetArea, simDock);

    // bottom: load history + event log
    QDockWidget* analysisDock = new QDockWidget("Load history / hazard events", this);
    QWidget* analysis = createAnalysisPanel();
    analysis->setMinimumHeight(220);
    analysisDock->setWidget(analysis);
    addDockWidget(Qt::BottomDockWidgetArea, analysisDock);

    refreshPayloadControls();
    refreshWheelWidgets();
    refreshSectionCombo();
}

void MainWindow::setupMenus()
{
    // ---- File ----
    QMenu* fileMenu = menuBar()->addMenu("File");
    QAction* shotAction = new QAction("Save Screenshot...", this);
    connect(shotAction, &QAction::triggered, this, [this]()
    {
        const QString path = QFileDialog::getSaveFileName(this, "Save screenshot", "drive-vehicle.png", "PNG image (*.png)");
        if (!path.isEmpty())
            saveScreenshot(path);
    });
    fileMenu->addAction(shotAction);
    fileMenu->addSeparator();
    QAction* exitAction = new QAction("Exit", this);
    connect(exitAction, &QAction::triggered, this, &QMainWindow::close);
    fileMenu->addAction(exitAction);

    // ---- View (original actions + cameras) ----
    QMenu* viewMenu = menuBar()->addMenu("View");

    QAction* surfaceAction = new QAction("Surface", this);
    QAction* wireframeAction = new QAction("Wireframe", this);

    QAction* startAction = new QAction("Start", this);
    QAction* stopAction = new QAction("Stop", this);

    QAction* resetAction = new QAction("Reset", this);

    connect(surfaceAction, &QAction::triggered, this, &MainWindow::setSurfaceMode);
    connect(wireframeAction, &QAction::triggered, this, &MainWindow::setWireframeMode);

    connect(startAction, &QAction::triggered, this, &MainWindow::startMotion);
    connect(stopAction, &QAction::triggered, this, &MainWindow::stopMotion);

    connect(resetAction, &QAction::triggered, this, &MainWindow::resetMotion);

    viewMenu->addAction(surfaceAction);
    viewMenu->addAction(wireframeAction);
    viewMenu->addSeparator();
    viewMenu->addAction(startAction);
    viewMenu->addAction(stopAction);
    viewMenu->addAction(resetAction);
    viewMenu->addSeparator();

    QMenu* cameraMenu = viewMenu->addMenu("Camera");
    const char* presetNames[] = { "Chase", "Wheels (side)", "Top", "Front", "Map overview" };
    for (int i = 0; i < 5; ++i)
    {
        QAction* a = cameraMenu->addAction(presetNames[i]);
        a->setShortcut(QKeySequence(QString("F%1").arg(i + 1)));
        connect(a, &QAction::triggered, this, [this, i]() { setCameraView(i); });
    }
    m_followAction = viewMenu->addAction("Follow vehicle");
    m_followAction->setCheckable(true);
    m_followAction->setChecked(true);
    connect(m_followAction, &QAction::toggled, this, &MainWindow::onFollowToggled);
    m_labelsAction = viewMenu->addAction("Hazard labels");
    m_labelsAction->setCheckable(true);
    m_labelsAction->setChecked(true);
    connect(m_labelsAction, &QAction::toggled, this, &MainWindow::onHazardLabelsToggled);

    // ---- Road (map selection) ----
    QMenu* roadMenu = menuBar()->addMenu("Road");
    m_mapGroup = new QActionGroup(this);
    m_mapGroup->setExclusive(true);
    for (int i = 0; i < 3; ++i)
    {
        QAction* a = roadMenu->addAction(mapLabel(kMaps[i]));
        a->setCheckable(true);
        a->setShortcut(QKeySequence(QString("Ctrl+%1").arg(i + 1)));
        a->setData(i);
        m_mapGroup->addAction(a);
        if (kMaps[i] == m_road.type())
            a->setChecked(true);
    }
    connect(m_mapGroup, &QActionGroup::triggered, this, [this](QAction* a) { onMapIndexChanged(a->data().toInt()); });

    // ---- Vehicle ----
    QMenu* vehicleMenu = menuBar()->addMenu("Vehicle");
    m_vehicleGroup = new QActionGroup(this);
    m_vehicleGroup->setExclusive(true);
    for (int i = 0; i < 4; ++i)
    {
        QAction* a = vehicleMenu->addAction(vehicleLabel(kVehicles[i]));
        a->setCheckable(true);
        a->setShortcut(QKeySequence(QString("Ctrl+Shift+%1").arg(i + 1)));
        a->setData(i);
        m_vehicleGroup->addAction(a);
        if (kVehicles[i] == m_vehicle.type())
            a->setChecked(true);
    }
    connect(m_vehicleGroup, &QActionGroup::triggered, this, [this](QAction* a) { onVehicleIndexChanged(a->data().toInt()); });

    // ---- Hazards ----
    QMenu* hazardMenu = menuBar()->addMenu("Hazards");
    m_placeAheadAction = hazardMenu->addAction("Place ahead of the vehicle");
    m_placeAheadAction->setShortcut(QKeySequence("Ctrl+P"));
    m_placeAheadAction->setToolTip("Install the selected hazard the chosen distance in front of the vehicle");
    connect(m_placeAheadAction, &QAction::triggered, this, &MainWindow::placeHazardAhead);
    m_placeClickAction = hazardMenu->addAction("Click on the road to place");
    m_placeClickAction->setCheckable(true);
    m_placeClickAction->setShortcut(QKeySequence("Ctrl+K"));
    m_placeClickAction->setToolTip("While enabled, a left click on the road installs the selected hazard there");
    connect(m_placeClickAction, &QAction::toggled, this, [this](bool on)
    {
        statusBar()->showMessage(on ? "Placement mode: left-click on the road to install the selected hazard"
                                    : "Placement mode off");
    });
    hazardMenu->addSeparator();
    QMenu* typeMenu = hazardMenu->addMenu("Hazard type");
    m_hazardKindGroup = new QActionGroup(this);
    m_hazardKindGroup->setExclusive(true);
    for (int i = 0; i < static_cast<int>(HazardKind::Count); ++i)
    {
        QAction* a = typeMenu->addAction(RoadNetwork::hazardKindName(static_cast<HazardKind>(i)));
        a->setCheckable(true);
        a->setData(i);
        a->setChecked(i == static_cast<int>(HazardKind::PotholeMedium));
        m_hazardKindGroup->addAction(a);
    }
    connect(m_hazardKindGroup, &QActionGroup::triggered, this, [this](QAction* a)
    {
        m_hazardKindCombo->setCurrentIndex(a->data().toInt());
    });
    hazardMenu->addSeparator();
    QAction* undoAction = hazardMenu->addAction("Undo last placed hazard");
    undoAction->setShortcut(QKeySequence::Undo);
    connect(undoAction, &QAction::triggered, this, &MainWindow::undoHazard);
    QAction* clearAction = hazardMenu->addAction("Remove all hazards");
    connect(clearAction, &QAction::triggered, this, &MainWindow::clearHazards);
    QAction* restoreAction = hazardMenu->addAction("Restore default hazards");
    connect(restoreAction, &QAction::triggered, this, &MainWindow::restoreHazards);

    // ---- Drive (speed) ----
    QMenu* driveMenu = menuBar()->addMenu("Drive");
    m_autoSpeedAction = driveMenu->addAction("Auto speed (obey speed limits)");
    m_autoSpeedAction->setCheckable(true);
    m_autoSpeedAction->setChecked(true);
    connect(m_autoSpeedAction, &QAction::toggled, this, &MainWindow::onAutoSpeedToggled);
    QAction* fasterAction = driveMenu->addAction("Faster (+5 km/h)");
    fasterAction->setShortcut(QKeySequence("Ctrl+Up"));
    connect(fasterAction, &QAction::triggered, this, &MainWindow::faster);
    QAction* slowerAction = driveMenu->addAction("Slower (-5 km/h)");
    slowerAction->setShortcut(QKeySequence("Ctrl+Down"));
    connect(slowerAction, &QAction::triggered, this, &MainWindow::slower);

    // ---- toolbar ----
    QToolBar* toolbar = addToolBar("View");
    toolbar->addAction(surfaceAction);
    toolbar->addAction(wireframeAction);
    toolbar->addSeparator();
    toolbar->addAction(startAction);
    toolbar->addAction(stopAction);
    toolbar->addAction(resetAction);
    toolbar->addSeparator();
    toolbar->addWidget(new QLabel(" Speed "));
    m_speedSpin = new QSpinBox;
    m_speedSpin->setRange(5, 130);
    m_speedSpin->setSingleStep(5);
    m_speedSpin->setSuffix(" km/h");
    m_speedSpin->setValue(m_manualSpeedKmh);
    m_speedSpin->setToolTip("Fixed vehicle speed (switches Auto off)");
    toolbar->addWidget(m_speedSpin);
    connect(m_speedSpin, qOverload<int>(&QSpinBox::valueChanged), this, &MainWindow::onSpeedSpinChanged);
    m_autoSpeedAction->setIconText("Auto");
    toolbar->addAction(m_autoSpeedAction);
    toolbar->addSeparator();
    m_placeAheadAction->setIconText("Place hazard");
    m_placeClickAction->setIconText("Click to place");
    toolbar->addAction(m_placeAheadAction);
    toolbar->addAction(m_placeClickAction);
}

QWidget* MainWindow::createDrivePanel()
{
    auto* panel = new QWidget;
    auto* layout = new QVBoxLayout(panel);

    // ---- map & vehicle ----
    auto* mapBox = new QGroupBox("Map && vehicle");
    auto* form = new QFormLayout(mapBox);

    m_mapCombo = new QComboBox;
    for (MapType t : kMaps)
        m_mapCombo->addItem(mapLabel(t));
    m_mapCombo->setCurrentIndex(mapIndex(m_road.type()));
    form->addRow("Map (Road)", m_mapCombo);

    m_vehicleCombo = new QComboBox;
    for (VehicleType t : kVehicles)
        m_vehicleCombo->addItem(vehicleLabel(t));
    m_vehicleCombo->setCurrentIndex(vehicleIndex(m_vehicle.type()));
    form->addRow("Vehicle", m_vehicleCombo);

    m_startCombo = new QComboBox;
    m_startCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_mapCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_vehicleCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    form->addRow("Start at", m_startCombo);

    m_payloadSlider = new QSlider(Qt::Horizontal);
    m_payloadLabel = new QLabel;
    m_payloadLabel->setMinimumWidth(130);
    auto* payloadRow = new QWidget;
    auto* payloadLayout = new QHBoxLayout(payloadRow);
    payloadLayout->setContentsMargins(0, 0, 0, 0);
    payloadLayout->addWidget(m_payloadSlider);
    payloadLayout->addWidget(m_payloadLabel);
    m_payloadTitle = new QLabel("Payload");
    form->addRow(m_payloadTitle, payloadRow);
    layout->addWidget(mapBox);

    // ---- speed ----
    auto* speedBox = new QGroupBox("Speed");
    auto* speedForm = new QFormLayout(speedBox);
    auto* autoButton = new QToolButton;
    autoButton->setDefaultAction(m_autoSpeedAction);
    autoButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    autoButton->setText("Auto: obey speed limits");
    autoButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    speedForm->addRow("Mode", autoButton);

    m_speedSlider = new QSlider(Qt::Horizontal);
    m_speedSlider->setRange(5, 130);
    m_speedSlider->setSingleStep(5);
    m_speedSlider->setPageStep(10);
    m_speedSlider->setValue(m_manualSpeedKmh);
    m_speedLabel = new QLabel;
    m_speedLabel->setMinimumWidth(70);
    auto* speedRow = new QWidget;
    auto* speedLayout = new QHBoxLayout(speedRow);
    speedLayout->setContentsMargins(0, 0, 0, 0);
    speedLayout->addWidget(m_speedSlider);
    speedLayout->addWidget(m_speedLabel);
    speedForm->addRow("Set speed", speedRow);

    m_timeScaleCombo = new QComboBox;
    m_timeScaleCombo->addItems({ "0.25x (slow motion)", "0.5x", "1x (real time)", "2x", "4x", "8x" });
    m_timeScaleCombo->setCurrentIndex(2);
    speedForm->addRow("Time scale", m_timeScaleCombo);
    auto* speedHelp = new QLabel("<small>Moving the speed slider (or Ctrl+Up / Ctrl+Down) switches to a fixed speed. "
                                 "The vehicle still slows down for sharp corners and at the end of the course.</small>");
    speedHelp->setWordWrap(true);
    speedForm->addRow(speedHelp);
    layout->addWidget(speedBox);

    // ---- hazards ----
    auto* hazardBox = new QGroupBox("Place hazards");
    auto* hazardForm = new QFormLayout(hazardBox);
    m_hazardKindCombo = new QComboBox;
    for (int i = 0; i < static_cast<int>(HazardKind::Count); ++i)
        m_hazardKindCombo->addItem(RoadNetwork::hazardKindName(static_cast<HazardKind>(i)));
    m_hazardKindCombo->setCurrentIndex(static_cast<int>(HazardKind::PotholeMedium));
    hazardForm->addRow("Type", m_hazardKindCombo);
    m_hazardLaneCombo = new QComboBox;
    m_hazardLaneCombo->addItems({ "Right wheel track", "Left wheel track", "Both wheel tracks (wide)", "Where clicked" });
    hazardForm->addRow("Pothole at", m_hazardLaneCombo);
    m_hazardDistanceSpin = new QSpinBox;
    m_hazardDistanceSpin->setRange(20, 400);
    m_hazardDistanceSpin->setSingleStep(10);
    m_hazardDistanceSpin->setValue(80);
    m_hazardDistanceSpin->setSuffix(" m ahead");
    hazardForm->addRow("Distance", m_hazardDistanceSpin);

    auto* buttons = new QWidget;
    auto* grid = new QGridLayout(buttons);
    grid->setContentsMargins(0, 0, 0, 0);
    auto makeButton = [](QAction* action, const QString& text)
    {
        auto* b = new QToolButton;
        b->setDefaultAction(action);
        b->setToolButtonStyle(Qt::ToolButtonTextOnly);
        b->setText(text);
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        return b;
    };
    grid->addWidget(makeButton(m_placeAheadAction, "Place ahead"), 0, 0);
    grid->addWidget(makeButton(m_placeClickAction, "Click on road..."), 0, 1);
    auto* undoButton = new QPushButton("Undo");
    auto* clearButton = new QPushButton("Remove all");
    auto* restoreButton = new QPushButton("Defaults");
    connect(undoButton, &QPushButton::clicked, this, &MainWindow::undoHazard);
    connect(clearButton, &QPushButton::clicked, this, &MainWindow::clearHazards);
    connect(restoreButton, &QPushButton::clicked, this, &MainWindow::restoreHazards);
    auto* row2 = new QWidget;
    auto* row2Layout = new QHBoxLayout(row2);
    row2Layout->setContentsMargins(0, 0, 0, 0);
    row2Layout->addWidget(undoButton);
    row2Layout->addWidget(clearButton);
    row2Layout->addWidget(restoreButton);
    grid->addWidget(row2, 1, 0, 1, 2);
    hazardForm->addRow(buttons);
    layout->addWidget(hazardBox);

    // ---- view ----
    auto* viewBox = new QGroupBox("View");
    auto* viewGrid = new QGridLayout(viewBox);
    const char* presetNames[] = { "Chase", "Wheels", "Top", "Front", "Map" };
    for (int i = 0; i < 5; ++i)
    {
        auto* button = new QPushButton(presetNames[i]);
        viewGrid->addWidget(button, 0, i);
        connect(button, &QPushButton::clicked, this, [this, i]() { setCameraView(i); });
    }
    auto* followButton = new QToolButton;
    followButton->setDefaultAction(m_followAction);
    followButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    auto* labelsButton = new QToolButton;
    labelsButton->setDefaultAction(m_labelsAction);
    labelsButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    viewGrid->addWidget(followButton, 1, 0, 1, 2);
    viewGrid->addWidget(labelsButton, 1, 2, 1, 3);

    m_exaggerationSpin = new QDoubleSpinBox;
    m_exaggerationSpin->setRange(1.0, 10.0);
    m_exaggerationSpin->setSingleStep(0.5);
    m_exaggerationSpin->setValue(m_exaggeration);
    m_exaggerationSpin->setSuffix(" x");
    m_exaggerationSpin->setToolTip("Display only: suspension and tyre deflections are drawn this many times larger.\n"
                                   "The dash-cam view always shows the true motion.");
    viewGrid->addWidget(new QLabel("Deflection exaggeration"), 2, 0, 1, 3);
    viewGrid->addWidget(m_exaggerationSpin, 2, 3, 1, 2);
    layout->addWidget(viewBox);
    layout->addStretch(1);

    connect(m_mapCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::onMapIndexChanged);
    connect(m_vehicleCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::onVehicleIndexChanged);
    connect(m_startCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::onStartSectionChanged);
    connect(m_speedSlider, &QSlider::valueChanged, this, &MainWindow::onSpeedSliderChanged);
    connect(m_payloadSlider, &QSlider::valueChanged, this, &MainWindow::onPayloadChanged);
    connect(m_timeScaleCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::onTimeScaleChanged);
    connect(m_exaggerationSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onExaggerationChanged);
    connect(m_hazardKindCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::onHazardKindChanged);

    m_speedLabel->setText(QString("%1 km/h").arg(m_manualSpeedKmh));
    return panel;
}

QWidget* MainWindow::createLoadsPanel()
{
    auto* panel = new QWidget;
    auto* layout = new QVBoxLayout(panel);

    // ---- live status ----
    auto* liveBox = new QGroupBox("Live");
    auto* liveLayout = new QVBoxLayout(liveBox);
    m_liveLabel = new QLabel;
    m_liveLabel->setTextFormat(Qt::RichText);
    liveLayout->addWidget(m_liveLabel);
    layout->addWidget(liveBox);

    // ---- wheel loads ----
    auto* wheelBox = new QGroupBox("Wheel loads && fatigue damage");
    auto* wheelLayout = new QVBoxLayout(wheelBox);
    m_wheelTable = new QTableWidget(0, 6);
    m_wheelTable->setHorizontalHeaderLabels({ "Wheel", "Static\n[kN]", "Load\n[kN]", "Load /\nstatic", "Peak\nratio", "Damage" });
    m_wheelTable->verticalHeader()->setVisible(false);
    m_wheelTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_wheelTable->setSelectionMode(QAbstractItemView::NoSelection);
    m_wheelTable->setFocusPolicy(Qt::NoFocus);
    m_wheelTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_wheelTable->verticalHeader()->setDefaultSectionSize(24);
    wheelLayout->addWidget(m_wheelTable);

    auto* expRow = new QWidget;
    auto* expLayout = new QHBoxLayout(expRow);
    expLayout->setContentsMargins(0, 0, 0, 0);
    expLayout->addWidget(new QLabel("S-N slope k (damage exponent)"));
    m_exponentSpin = new QDoubleSpinBox;
    m_exponentSpin->setRange(3.0, 12.0);
    m_exponentSpin->setSingleStep(0.5);
    m_exponentSpin->setValue(m_analyzer.exponent());
    m_exponentSpin->setToolTip("Pseudo-damage D = sum 0.5*(dF/50 kN)^k over all load half cycles.\n"
                               "k = 5 is common for steel suspension parts; larger k makes big impacts dominate.");
    expLayout->addWidget(m_exponentSpin);
    wheelLayout->addWidget(expRow);
    connect(m_exponentSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onExponentChanged);

    m_summaryLabel = new QLabel;
    m_summaryLabel->setTextFormat(Qt::RichText);
    m_summaryLabel->setWordWrap(true);
    wheelLayout->addWidget(m_summaryLabel);
    layout->addWidget(wheelBox);

    // ---- run comparison ----
    auto* runBox = new QGroupBox("Run comparison (vehicle / map)");
    auto* runLayout = new QVBoxLayout(runBox);
    m_runTable = new QTableWidget(0, 7);
    m_runTable->setHorizontalHeaderLabels({ "Vehicle", "Map", "Speed", "km", "Damage", "Peak\n[kN]", "Events" });
    m_runTable->verticalHeader()->setVisible(false);
    m_runTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_runTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_runTable->horizontalHeader()->setStretchLastSection(true);
    m_runTable->verticalHeader()->setDefaultSectionSize(22);
    m_runTable->setMinimumHeight(140);
    m_runTable->setToolTip("A row is added when a course is completed, or when you reset / change the\n"
                           "map, vehicle or payload after driving at least 100 m.");
    runLayout->addWidget(m_runTable);
    layout->addWidget(runBox);

    auto* help = new QLabel(
        "<small><b>Load / static</b> = dynamic wheel load divided by the static wheel load "
        "(1.0 = standing still, 0 = tyre lifted off).<br>"
        "<b>Damage</b> = Miner pseudo-damage of the wheel station with one common reference for all "
        "vehicles (one unit = one load cycle with a 50 kN range), so heavier vehicles score higher.<br>"
        "Colours: tyres, arrows and the wheel-path trail show load / static; "
        "springs and axles show accumulated damage.</small>");
    help->setWordWrap(true);
    help->setTextFormat(Qt::RichText);
    layout->addWidget(help);
    layout->addStretch(1);
    return panel;
}

QWidget* MainWindow::createAnalysisPanel()
{
    auto* splitter = new QSplitter(Qt::Horizontal);

    m_plot = new LoadPlotWidget;
    m_plot->setWindowSeconds(8.0);

    m_eventTable = new QTableWidget(0, 8);
    m_eventTable->setHorizontalHeaderLabels({ "Hazard", "km/h", "Peak\n[kN]", "Peak /\nstatic",
                                              "Wheel", "Lift\noff", "Damage", "= road\n[km]" });
    m_eventTable->verticalHeader()->setVisible(false);
    m_eventTable->verticalHeader()->setDefaultSectionSize(22);
    m_eventTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_eventTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_eventTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_eventTable->horizontalHeader()->setStretchLastSection(true);
    m_eventTable->setToolTip("One row per hazard the vehicle has driven over.\n"
                             "'= road' converts the damage into km of hazard-free road\n"
                             "with the roughness damage rate measured so far.");

    splitter->addWidget(m_plot);
    splitter->addWidget(m_eventTable);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({ 440, 580 });
    return splitter;
}

void MainWindow::populateTree()
{
    m_treeWidget->blockSignals(true);
    m_treeWidget->clear();

    std::map<std::string, QTreeWidgetItem*> groups;
    for (int pid : m_pidActorManager->getPidList())
    {
        const PidActorManager::PidSpec* spec = m_pidActorManager->getSpec(pid);
        if (!spec)
            continue;

        QTreeWidgetItem*& group = groups[spec->group];
        if (!group)
        {
            group = new QTreeWidgetItem(m_treeWidget);
            group->setText(0, QString::fromStdString(spec->group));
            group->setFlags(group->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsAutoTristate);
            group->setCheckState(0, Qt::Checked);
            group->setData(0, Qt::UserRole, -1);
        }

        auto* item = new QTreeWidgetItem(group);
        item->setText(0, QString("PID %1  %2").arg(pid).arg(QString::fromStdString(spec->name)));
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, Qt::Checked);
        item->setData(0, Qt::UserRole, pid);
    }

    m_treeWidget->expandAll();
    m_treeWidget->blockSignals(false);
}

void MainWindow::refreshSectionCombo()
{
    if (!m_startCombo)
        return;
    m_startCombo->blockSignals(true);
    m_startCombo->clear();
    for (const auto& sec : m_road.sections())
        m_startCombo->addItem(QString::fromStdString(sec.name));
    m_startCombo->setCurrentIndex(std::clamp(m_startSection, 0, m_startCombo->count() - 1));
    m_startCombo->blockSignals(false);
}

void MainWindow::refreshPayloadControls()
{
    if (!m_payloadSlider)
        return;
    const VehicleSpec& spec = m_vehicle.spec();
    m_payloadSlider->blockSignals(true);
    m_payloadSlider->setRange(0, static_cast<int>(std::round(spec.payloadMax / spec.payloadStep)));
    m_payloadSlider->setValue(static_cast<int>(std::round(m_vehicle.payload() / spec.payloadStep)));
    m_payloadSlider->blockSignals(false);

    const bool truck = (spec.type == VehicleType::Truck);
    m_payloadTitle->setText(truck ? "Payload (cargo)" : "Load (people, luggage)");
    if (truck)
        m_payloadLabel->setText(QString("%1 t  (GVW %2 t)")
                                    .arg(m_vehicle.payload() / 1000.0, 0, 'f', 1)
                                    .arg(m_vehicle.totalMass() / 1000.0, 0, 'f', 1));
    else
        m_payloadLabel->setText(QString("%1 kg  (total %2 t)")
                                    .arg(m_vehicle.payload(), 0, 'f', 0)
                                    .arg(m_vehicle.totalMass() / 1000.0, 0, 'f', 2));
}

void MainWindow::refreshWheelWidgets()
{
    const int n = m_vehicle.wheelCount();
    if (m_wheelTable)
    {
        m_wheelTable->setRowCount(n);
        for (int r = 0; r < n; ++r)
        {
            for (int c = 0; c < 6; ++c)
            {
                if (!m_wheelTable->item(r, c))
                    m_wheelTable->setItem(r, c, centered(QString()));
            }
            m_wheelTable->item(r, 0)->setText(QString::fromStdString(m_vehicle.wheel(r).name));
        }
        m_wheelTable->setFixedHeight(24 * n + 50);
    }
    if (m_plot)
    {
        QStringList names;
        QVector<QColor> colors;
        for (int i = 0; i < n; ++i)
        {
            names << QString::fromStdString(m_vehicle.wheel(i).name);
            colors << kWheelColors[i];
        }
        m_plot->setSeries(names, colors);
    }
}

// ===========================================================================
// scene
// ===========================================================================

void MainWindow::setupVtk()
{
    m_renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_renderer = vtkSmartPointer<vtkRenderer>::New();

    m_vtkWidget->setRenderWindow(m_renderWindow);
    m_renderWindow->AddRenderer(m_renderer);
    // layer 1: front camera + mini map insets drawn on top of the main view
    m_renderWindow->SetNumberOfLayers(2);

    auto specs = createSceneSpecs();
    m_pidActorManager->buildFromSpecs(m_renderer, specs);

    // daylight sky
    m_renderer->GradientBackgroundOn();
    m_renderer->SetBackground(0.82, 0.87, 0.92);   // horizon
    m_renderer->SetBackground2(0.33, 0.53, 0.80);  // zenith
    m_renderer->SetNearClippingPlaneTolerance(0.0005);

    vtkNew<vtkLightKit> lightKit;
    lightKit->SetKeyLightIntensity(0.9);
    lightKit->AddLightsToRenderer(m_renderer);

    populateTree();

    m_cellPicker = vtkSmartPointer<vtkCellPicker>::New();
    m_cellPicker->SetTolerance(0.0005);

    m_leftButtonPressCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    m_leftButtonPressCallback->SetClientData(this);
    m_leftButtonPressCallback->SetCallback(MainWindow::onLeftButtonPress);

    vtkNew<vtkInteractorStyleTrackballCamera> style;
    m_renderWindow->GetInteractor()->SetInteractorStyle(style);
    // higher priority than the interactor style, so a placement click can be consumed
    m_renderWindow->GetInteractor()->AddObserver(
        vtkCommand::LeftButtonPressEvent,
        m_leftButtonPressCallback, 1.0f);

    // animation timer
    m_motionTimer = new QTimer(this);
    m_motionTimer->setTimerType(Qt::PreciseTimer);
    connect(m_motionTimer, &QTimer::timeout,
        this, &MainWindow::updateMotion);

    createOverlays();
    setupFrontCamera();
    setupMiniMap();
}

std::map<int, PidActorManager::PidSpec> MainWindow::createSceneSpecs()
{
    using Spec = PidActorManager::PidSpec;
    using Mode = PidActorManager::ColorMode;
    std::map<int, Spec> specs;

    // ---- vehicle ----
    buildVehicleMeshes();
    auto vehicleSpec = [](const char* name, vtkSmartPointer<vtkDataSet> data)
    {
        Spec s;
        s.name = name;
        s.group = "Vehicle";
        s.data = data;
        s.showEdges = true;
        return s;
    };
    specs[Pid::Cab] = vehicleSpec("Cab / cabin", m_cabGrid);
    specs[Pid::Cab].colorMode = Mode::DirectRgb;
    specs[Pid::Cab].arrayName = "Color";
    specs[Pid::Body] = vehicleSpec("Body / cargo box", m_bodyGrid);
    specs[Pid::Body].colorMode = Mode::DirectRgb;
    specs[Pid::Body].arrayName = "Color";
    specs[Pid::Tires] = vehicleSpec("Tyres (load / static)", m_tirePoly);
    specs[Pid::Tires].colorMode = Mode::Mapped;
    specs[Pid::Tires].arrayName = "Scalar";
    specs[Pid::Tires].lut = m_loadLut;
    specs[Pid::Tires].range = { 0.0, 3.0 };
    specs[Pid::Camera] = vehicleSpec("Front camera", m_cameraGrid);
    specs[Pid::Camera].color = { 0.08, 0.08, 0.10 };
    specs[Pid::Suspension] = vehicleSpec("Springs / axles (damage)", m_suspPoly);
    specs[Pid::Suspension].colorMode = Mode::Mapped;
    specs[Pid::Suspension].arrayName = "Scalar";
    specs[Pid::Suspension].lut = m_damageLut;
    specs[Pid::Suspension].range = { -6.0, 2.0 };
    specs[Pid::Frame] = vehicleSpec("Chassis / underbody", m_frameGrid);
    specs[Pid::Frame].color = { 0.16, 0.16, 0.18 };

    // ---- map ----
    auto mapSpec = [](const char* name, vtkSmartPointer<vtkUnstructuredGrid> grid, bool miniMap)
    {
        Spec s;
        s.name = name;
        s.group = "Map";
        s.data = grid;
        s.colorMode = Mode::DirectRgb;
        s.arrayName = "Color";
        s.inMiniMap = miniMap;
        return s;
    };
    specs[Pid::Terrain]  = mapSpec("Terrain", m_road.buildTerrain(), true);
    specs[Pid::Road]     = mapSpec("Road surface / markings", m_road.buildRoadSurface(), true);
    specs[Pid::Curbs]    = mapSpec("Median / sidewalks / blocks", m_road.buildCurbs(), true);
    specs[Pid::Roadside] = mapSpec("Rails / lamps / signals", m_road.buildRoadside(), false);
    specs[Pid::Scenery]  = mapSpec("Buildings / trees / fields", m_road.buildScenery(), true);
    specs[Pid::Potholes] = mapSpec("Potholes", m_road.buildPotholes(), false);
    specs[Pid::Bumps]    = mapSpec("Speed bumps / crosswalks", m_road.buildBumps(), false);

    createAnalysisSpecs(specs);
    return specs;
}

void MainWindow::buildVehicleMeshes()
{
    const VehicleSpec& spec = m_vehicle.spec();
    const bool truck = (spec.type == VehicleType::Truck);
    const Rgb glass = makeRgb(44, 60, 80);
    const Rgb dark = makeRgb(50, 50, 56);
    const Rgb lamp = makeRgb(250, 240, 190);
    const Rgb tail = makeRgb(200, 30, 30);

    MeshBuilder cab, body, camera, frame;
    if (truck)
    {
        // PID 1 : cab-over cab with painted windows / lights
        cab.setColor(makeRgb(34, 94, 180));
        cab.addBox({ -0.80, -1.12, 1.10 }, { 1.25, 1.12, 3.05 });
        cab.setColor(glass);
        cab.addQuad({ 1.256, -1.0, 2.00 }, { 1.256, 1.0, 2.00 }, { 1.256, 1.0, 2.85 }, { 1.256, -1.0, 2.85 });
        cab.addQuad({ 0.35, 1.126, 2.00 }, { 1.15, 1.126, 2.00 }, { 1.15, 1.126, 2.80 }, { 0.35, 1.126, 2.80 });
        cab.addQuad({ 0.35, -1.126, 2.00 }, { 0.35, -1.126, 2.80 }, { 1.15, -1.126, 2.80 }, { 1.15, -1.126, 2.00 });
        cab.setColor(dark);
        cab.addQuad({ 1.257, -0.55, 1.25 }, { 1.257, 0.55, 1.25 }, { 1.257, 0.55, 1.75 }, { 1.257, -0.55, 1.75 });
        cab.setColor(lamp);
        cab.addQuad({ 1.258, 0.70, 1.30 }, { 1.258, 1.00, 1.30 }, { 1.258, 1.00, 1.50 }, { 1.258, 0.70, 1.50 });
        cab.addQuad({ 1.258, -1.00, 1.30 }, { 1.258, -0.70, 1.30 }, { 1.258, -0.70, 1.50 }, { 1.258, -1.00, 1.50 });
        cab.setColor(makeRgb(56, 56, 62));
        cab.addBox({ 1.10, -1.16, 0.62 }, { 1.36, 1.16, 0.95 });

        // PID 2 : cargo box
        body.setColor(makeRgb(220, 220, 212));
        body.addBox({ -6.35, -1.20, 1.25 }, { -0.95, 1.20, 3.40 });

        // PID 4 : camera module on the cab roof
        camera.addCylinder({ 1.10, 0.0, 3.15 }, 'X', 0.10, 0.30, 16);

        // PID 6 : frame rails + cross members
        frame.addBox({ -6.20, 0.43, kTruckFrameZ0 }, { 1.00, 0.57, 1.20 });
        frame.addBox({ -6.20, -0.57, kTruckFrameZ0 }, { 1.00, -0.43, 1.20 });
        for (double x : { 0.6, -1.0, -3.2, -4.5, -5.9 })
            frame.addBox({ x - 0.06, -0.43, 1.00 }, { x + 0.06, 0.43, 1.15 });

        m_springTopZ = kTruckFrameZ0;
    }
    else
    {
        const CarShape c = carShape(spec.type);
        const WheelDef& wf = spec.wheels[0];
        const double wb = -spec.wheels[2].x;
        const double arch = wf.radius + 0.06;           // half length of the wheel arch opening
        const double archZ = 2.0 * wf.radius + 0.04;    // bottom of the body above the wheels

        // PID 2 : lower body with wheel-arch openings, lights
        body.setColor(c.paint);
        auto piece = [&](double x0, double x1, double z0)
        {
            body.addBox({ x0, -c.halfW, z0 }, { x1, c.halfW, c.z1 });
        };
        piece(arch, spec.frontX, c.z0);
        piece(-arch, arch, archZ);
        piece(-wb + arch, -arch, c.z0);
        piece(-wb - arch, -wb + arch, archZ);
        piece(spec.rearX, -wb - arch, c.z0);
        const double fx = spec.frontX + 0.003;
        const double rx = spec.rearX - 0.003;
        const double ly0 = c.halfW - 0.34, ly1 = c.halfW - 0.06;
        body.setColor(lamp);
        body.addQuad({ fx, ly0, c.z1 - 0.18 }, { fx, ly1, c.z1 - 0.18 }, { fx, ly1, c.z1 - 0.06 }, { fx, ly0, c.z1 - 0.06 });
        body.addQuad({ fx, -ly1, c.z1 - 0.18 }, { fx, -ly0, c.z1 - 0.18 }, { fx, -ly0, c.z1 - 0.06 }, { fx, -ly1, c.z1 - 0.06 });
        body.setColor(dark);
        body.addQuad({ fx, -0.42, c.z0 + 0.12 }, { fx, 0.42, c.z0 + 0.12 }, { fx, 0.42, c.z1 - 0.24 }, { fx, -0.42, c.z1 - 0.24 });
        body.setColor(tail);
        body.addQuad({ rx, ly1, c.z1 - 0.2 }, { rx, ly0, c.z1 - 0.2 }, { rx, ly0, c.z1 - 0.06 }, { rx, ly1, c.z1 - 0.06 });
        body.addQuad({ rx, -ly0, c.z1 - 0.2 }, { rx, -ly1, c.z1 - 0.2 }, { rx, -ly1, c.z1 - 0.06 }, { rx, -ly0, c.z1 - 0.06 });

        // PID 1 : greenhouse (glass) with a painted roof panel
        cab.setColor(glass);
        cab.addHex({ Vec3{ c.ghX0b, -c.ghYb, c.z1 }, Vec3{ c.ghX1b, -c.ghYb, c.z1 },
                     Vec3{ c.ghX1b, c.ghYb, c.z1 }, Vec3{ c.ghX0b, c.ghYb, c.z1 },
                     Vec3{ c.ghX0t, -c.ghYt, c.roofZ }, Vec3{ c.ghX1t, -c.ghYt, c.roofZ },
                     Vec3{ c.ghX1t, c.ghYt, c.roofZ }, Vec3{ c.ghX0t, c.ghYt, c.roofZ } });
        cab.setColor(c.paint);
        const double rz = c.roofZ + 0.004;
        cab.addQuad({ c.ghX0t + 0.04, -c.ghYt + 0.04, rz }, { c.ghX1t - 0.04, -c.ghYt + 0.04, rz },
                    { c.ghX1t - 0.04, c.ghYt - 0.04, rz }, { c.ghX0t + 0.04, c.ghYt - 0.04, rz });
        if (c.roofRails)
        {
            cab.setColor(dark);
            cab.addBox({ c.ghX0t + 0.1, c.ghYt - 0.12, c.roofZ }, { c.ghX1t - 0.1, c.ghYt - 0.06, c.roofZ + 0.07 });
            cab.addBox({ c.ghX0t + 0.1, -c.ghYt + 0.06, c.roofZ }, { c.ghX1t - 0.1, -c.ghYt + 0.12, c.roofZ + 0.07 });
        }

        // PID 4 : small roof camera at the top of the windshield
        camera.addCylinder({ spec.cameraMount[0] - 0.08, 0.0, spec.cameraMount[2] }, 'X', 0.045, 0.14, 12);

        // PID 6 : underbody pan
        frame.addBox({ spec.rearX + 0.25, -c.halfW + 0.1, c.z0 - 0.03 }, { spec.frontX - 0.25, c.halfW - 0.1, c.z0 });

        m_springTopZ = archZ - 0.02;
    }
    m_cabGrid = cab.buildGrid();
    m_bodyGrid = body.buildGrid();
    m_cameraGrid = camera.buildGrid();
    m_frameGrid = frame.buildGrid();

    // PID 3 : tyres. Points are rebuilt every frame (suspension travel + rolling).
    {
        MeshBuilder mb;
        m_tirePointWheel.clear();
        m_tireCellWheel.clear();
        for (int i = 0; i < m_vehicle.wheelCount(); ++i)
        {
            const WheelDef& w = m_vehicle.wheel(i);
            const vtkIdType p0 = mb.numberOfPoints();
            const vtkIdType c0 = mb.numberOfCells();
            mb.setScalar(1.0);
            if (w.dual)
            {
                mb.addCylinder({ 0.0, -0.155, 0.0 }, 'Y', w.radius, 0.25, 20);
                mb.addCylinder({ 0.0, 0.155, 0.0 }, 'Y', w.radius, 0.25, 20);
            }
            else
            {
                mb.addCylinder({ 0.0, 0.0, 0.0 }, 'Y', w.radius, w.width, 20);
            }
            m_tirePointWheel.insert(m_tirePointWheel.end(), static_cast<size_t>(mb.numberOfPoints() - p0), i);
            m_tireCellWheel.insert(m_tireCellWheel.end(), static_cast<size_t>(mb.numberOfCells() - c0), i);
        }
        m_tireBase = mb.points();
        m_tirePoly = mb.buildPolyData();
    }

    // PID 5 : springs (one per wheel) + axle beams (one per axle), colour = damage
    {
        MeshBuilder mb;
        m_suspCellOwner.clear();
        const std::array<Vec3, 8> zero{};
        mb.setScalar(-6.0);
        for (int owner = 0; owner < m_vehicle.wheelCount() + m_vehicle.axleCount(); ++owner)
        {
            mb.addHex(zero, true);
            m_suspCellOwner.insert(m_suspCellOwner.end(), 6, owner);
        }
        m_suspPoly = mb.buildPolyData();
    }

    // PID 21 : load arrows beside every wheel
    {
        MeshBuilder mb;
        const std::array<Vec3, 8> zero{};
        mb.setScalar(1.0);
        for (int i = 0; i < m_vehicle.wheelCount(); ++i)
        {
            mb.addHex(zero, true);  // shaft
            const vtkIdType b0 = mb.addPoint({});
            const vtkIdType b1 = mb.addPoint({});
            const vtkIdType b2 = mb.addPoint({});
            const vtkIdType b3 = mb.addPoint({});
            const vtkIdType apex = mb.addPoint({});
            mb.addCell(VTK_QUAD, { b0, b3, b2, b1 });
            mb.addCell(VTK_TRIANGLE, { b0, b1, apex });
            mb.addCell(VTK_TRIANGLE, { b1, b2, apex });
            mb.addCell(VTK_TRIANGLE, { b2, b3, apex });
            mb.addCell(VTK_TRIANGLE, { b3, b0, apex });
        }
        m_arrowPoly = mb.buildPolyData();
    }

    m_arrowScale = 0.38 * spec.height + 0.1;

    // wheel-path trail follows the wheel tracks of this vehicle
    double track = 0.0;
    for (int i = 0; i < m_vehicle.wheelCount(); ++i)
        track += std::abs(m_vehicle.wheel(i).y);
    m_trailOffset = track / m_vehicle.wheelCount();
    m_trailHalfWidth = truck ? 0.22 : 0.14;
}

void MainWindow::applyVehicleMeshes()
{
    m_pidActorManager->setData(Pid::Cab, m_cabGrid);
    m_pidActorManager->setData(Pid::Body, m_bodyGrid);
    m_pidActorManager->setData(Pid::Tires, m_tirePoly);
    m_pidActorManager->setData(Pid::Camera, m_cameraGrid);
    m_pidActorManager->setData(Pid::Suspension, m_suspPoly);
    m_pidActorManager->setData(Pid::Frame, m_frameGrid);
    m_pidActorManager->setData(Pid::Arrows, m_arrowPoly);
}

void MainWindow::applyMapMeshes()
{
    m_pidActorManager->setData(Pid::Terrain, m_road.buildTerrain());
    m_pidActorManager->setData(Pid::Road, m_road.buildRoadSurface());
    m_pidActorManager->setData(Pid::Curbs, m_road.buildCurbs());
    m_pidActorManager->setData(Pid::Roadside, m_road.buildRoadside());
    m_pidActorManager->setData(Pid::Scenery, m_road.buildScenery());
    m_pidActorManager->setData(Pid::Potholes, m_road.buildPotholes());
    m_pidActorManager->setData(Pid::Bumps, m_road.buildBumps());
}

void MainWindow::createAnalysisSpecs(std::map<int, PidActorManager::PidSpec>& specs)
{
    using Spec = PidActorManager::PidSpec;
    using Mode = PidActorManager::ColorMode;

    // PID 20 : load trail painted along both wheel paths (max load / static seen at each station)
    {
        m_trailPoints = vtkSmartPointer<vtkPoints>::New();
        m_trailPoints->SetDataTypeToDouble();
        m_trailPolys = vtkSmartPointer<vtkCellArray>::New();
        m_trailScalars = vtkSmartPointer<vtkDoubleArray>::New();
        m_trailScalars->SetName("Scalar");
        m_trailPoly = vtkSmartPointer<vtkPolyData>::New();
        m_trailPoly->SetPoints(m_trailPoints);
        m_trailPoly->SetPolys(m_trailPolys);
        m_trailPoly->GetCellData()->AddArray(m_trailScalars);

        Spec s;
        s.name = "Wheel-path load trail";
        s.group = "Analysis";
        s.data = m_trailPoly;
        s.colorMode = Mode::Mapped;
        s.arrayName = "Scalar";
        s.lut = m_loadLut;
        s.range = { 0.0, 3.0 };
        s.inFrontCamera = false;
        specs[Pid::Trail] = s;
    }

    // PID 21 : load arrows beside every wheel (length and colour = load / static)
    {
        Spec s;
        s.name = "Wheel load arrows";
        s.group = "Analysis";
        s.data = m_arrowPoly;
        s.colorMode = Mode::Mapped;
        s.arrayName = "Scalar";
        s.lut = m_loadLut;
        s.range = { 0.0, 3.0 };
        s.inFrontCamera = false;
        specs[Pid::Arrows] = s;
    }
}

void MainWindow::createOverlays()
{
    // HUD (top-left)
    m_hudActor = vtkSmartPointer<vtkTextActor>::New();
    m_hudActor->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
    m_hudActor->SetPosition(0.008, 0.985);
    vtkTextProperty* hud = m_hudActor->GetTextProperty();
    hud->SetFontSize(15);
    hud->SetFontFamilyToCourier();
    hud->SetColor(1.0, 1.0, 1.0);
    hud->SetVerticalJustificationToTop();
    hud->SetBackgroundColor(0.0, 0.0, 0.0);
    hud->SetBackgroundOpacity(0.5);
    m_renderer->AddViewProp(m_hudActor);

    // event alert (bottom-centre, right of the mini map)
    m_alertActor = vtkSmartPointer<vtkTextActor>::New();
    m_alertActor->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
    m_alertActor->SetPosition(0.60, 0.025);
    vtkTextProperty* alert = m_alertActor->GetTextProperty();
    alert->SetFontSize(16);
    alert->BoldOn();
    alert->SetJustificationToCentered();
    alert->SetVerticalJustificationToBottom();
    alert->SetBackgroundColor(0.0, 0.0, 0.0);
    alert->SetBackgroundOpacity(0.65);
    m_alertActor->SetVisibility(0);
    m_renderer->AddViewProp(m_alertActor);

    // colour legends
    auto makeBar = [this](vtkLookupTable* lut, const char* title, const char* fmt, double x)
    {
        auto bar = vtkSmartPointer<vtkScalarBarActor>::New();
        bar->SetLookupTable(lut);
        bar->SetTitle(title);
        bar->SetLabelFormat(fmt);
        bar->SetNumberOfLabels(7);
        bar->SetPosition(x, 0.30);
        bar->SetWidth(0.055);
        bar->SetHeight(0.40);
        bar->SetBarRatio(0.35);
        bar->UnconstrainedFontSizeOn();
        bar->GetTitleTextProperty()->SetFontSize(13);
        bar->GetTitleTextProperty()->SetColor(1.0, 1.0, 1.0);
        bar->GetTitleTextProperty()->SetBackgroundColor(0.0, 0.0, 0.0);
        bar->GetTitleTextProperty()->SetBackgroundOpacity(0.45);
        bar->GetLabelTextProperty()->SetFontSize(12);
        bar->GetLabelTextProperty()->SetColor(1.0, 1.0, 1.0);
        bar->GetLabelTextProperty()->SetBackgroundColor(0.0, 0.0, 0.0);
        bar->GetLabelTextProperty()->SetBackgroundOpacity(0.35);
        m_renderer->AddViewProp(bar);
        return bar;
    };
    m_loadBar = makeBar(m_loadLut, "Load /\nstatic", "%.1f", 0.935);
    m_damageBar = makeBar(m_damageLut, "log10\ndamage", "%.0f", 0.865);
    // exact integer ticks (avoids a "-0" label from round-off)
    vtkNew<vtkDoubleArray> ticks;
    for (int v = -6; v <= 2; v += 2)
        ticks->InsertNextValue(v);
    m_damageBar->SetCustomLabels(ticks);
    m_damageBar->SetUseCustomLabels(true);

    rebuildZoneSigns();
}

void MainWindow::rebuildZoneSigns()
{
    for (auto& sign : m_zoneSigns)
        m_renderer->RemoveViewProp(sign);
    m_zoneSigns.clear();

    const auto& zones = m_road.zones();
    for (size_t k = 0; k < zones.size(); ++k)
    {
        const RoadZone& z = zones[k];
        const double s = (k == 0) ? 30.0 : z.s0 + 25.0;
        char text[128];
        std::snprintf(text, sizeof(text), "%s\n%.0f km/h", z.name.c_str(), z.speedLimitKmh);

        auto sign = vtkSmartPointer<vtkBillboardTextActor3D>::New();
        const Vec3 p = m_road.course().point(s, m_road.signOffset(), 7.0);
        sign->SetPosition(p[0], p[1], p[2]);
        sign->SetInput(text);
        vtkTextProperty* tp = sign->GetTextProperty();
        tp->SetFontSize(20);
        tp->BoldOn();
        tp->SetJustificationToCentered();
        switch (z.roadClass)
        {
        case RoadClass::Highway:
            tp->SetColor(1.0, 1.0, 1.0);
            tp->SetBackgroundColor(0.07, 0.44, 0.26);
            break;
        case RoadClass::National:
        case RoadClass::City:
            tp->SetColor(1.0, 1.0, 1.0);
            tp->SetBackgroundColor(0.10, 0.28, 0.60);
            break;
        case RoadClass::Village:
        case RoadClass::SchoolZone:
            tp->SetColor(0.1, 0.1, 0.1);
            tp->SetBackgroundColor(0.98, 0.80, 0.15);
            break;
        }
        tp->SetBackgroundOpacity(0.9);
        sign->PickableOff();
        m_renderer->AddViewProp(sign);
        m_zoneSigns.push_back(sign);
    }
}

std::string MainWindow::hazardResultText(const Hazard& h, const HazardEvent& ev) const
{
    const std::string wheelName = (ev.peakWheel >= 0 && ev.peakWheel < m_vehicle.wheelCount())
                                      ? m_vehicle.wheel(ev.peakWheel).name : std::string("-");
    char text[256];
    std::snprintf(text, sizeof(text), "%s\npeak %.2fx static (%s)%s\n@ %.0f km/h  damage %.3g",
                  h.label.c_str(), ev.peakRatio, wheelName.c_str(), ev.liftOff ? "  LIFT-OFF" : "",
                  ev.speedKmh, ev.damage);
    return text;
}

void MainWindow::rebuildHazardOverlays()
{
    // 3D labels above every hazard
    for (auto& [id, label] : m_hazardLabels)
        m_renderer->RemoveViewProp(label);
    m_hazardLabels.clear();

    const auto& hazards = m_road.hazards();
    for (size_t k = 0; k < hazards.size(); ++k)
    {
        const Hazard& h = hazards[k];
        auto label = vtkSmartPointer<vtkBillboardTextActor3D>::New();
        const Vec3 p = m_road.course().point(h.s, h.d, 4.8);
        label->SetPosition(p[0], p[1], p[2]);
        vtkTextProperty* tp = label->GetTextProperty();
        tp->SetFontSize(14);
        tp->BoldOn();
        tp->SetJustificationToCentered();
        tp->SetBackgroundColor(0.05, 0.05, 0.08);
        tp->SetBackgroundOpacity(0.6);
        if (m_analyzer.hazardState(static_cast<int>(k)) == LoadAnalyzer::Done)
        {
            const HazardEvent& ev = m_analyzer.hazardResult(static_cast<int>(k));
            const auto rgb = severityRgb(severityOf(ev.peakRatio));
            label->SetInput(hazardResultText(h, ev).c_str());
            tp->SetColor(rgb[0], rgb[1], rgb[2]);
        }
        else
        {
            label->SetInput(h.label.c_str());
            tp->SetColor(1.0, 1.0, 1.0);
        }
        label->PickableOff();
        m_renderer->AddViewProp(label);
        m_hazardLabels[h.id] = label;
    }

    // hazard discs on the mini map
    if (m_miniHazardMapper)
    {
        const auto b = m_road.bounds();
        const double ext = std::max(b[1] - b[0], b[3] - b[2]);
        const double radius = 0.009 * ext;
        MeshBuilder mb;
        m_miniHazardCells.clear();
        for (size_t k = 0; k < hazards.size(); ++k)
        {
            const Hazard& h = hazards[k];
            Rgb color = makeRgb(255, 255, 255);
            if (m_analyzer.hazardState(static_cast<int>(k)) == LoadAnalyzer::Done)
            {
                const auto rgb = severityRgb(severityOf(m_analyzer.hazardResult(static_cast<int>(k)).peakRatio));
                color = makeRgb(static_cast<int>(rgb[0] * 255), static_cast<int>(rgb[1] * 255), static_cast<int>(rgb[2] * 255));
            }
            mb.setColor(color);
            m_miniHazardCells[h.id] = mb.numberOfCells();
            const Vec3 c = m_road.course().point(h.s, h.d, 60.0);
            std::vector<Vec3> ring;
            for (int i = 0; i < kMiniDiscSegments; ++i)
            {
                const double t = 2.0 * vtkMath::Pi() * i / kMiniDiscSegments;
                ring.push_back({ c[0] + radius * std::cos(t), c[1] + radius * std::sin(t), c[2] });
            }
            mb.addTriangleFan(c, ring);
        }
        m_miniHazards = mb.buildPolyData();
        m_miniHazardMapper->SetInputData(m_miniHazards);
    }

    updateHazardLabels();
}

// ===========================================================================
// front camera (dash cam) inset
// ===========================================================================

void MainWindow::setupFrontCamera()
{
    m_frontCamRenderer = vtkSmartPointer<vtkRenderer>::New();

    // top-right inset: 20% of width and height (normalized viewport coords)
    m_frontCamRenderer->SetViewport(0.8, 0.8, 1.0, 1.0);
    m_frontCamRenderer->GradientBackgroundOn();
    m_frontCamRenderer->SetBackground(0.80, 0.86, 0.92);
    m_frontCamRenderer->SetBackground2(0.33, 0.53, 0.80);

    // draw on top of the main renderer, but still clear its own background
    m_frontCamRenderer->SetLayer(1);
    m_frontCamRenderer->PreserveColorBufferOff();

    // mouse interaction / picking stays on the main renderer
    m_frontCamRenderer->InteractiveOff();

    // camera-only actors: share the mapper (geometry), but use their own property without
    // mesh edges. Transform / visibility are synced in updateFrontCamera().
    m_frontCamActors.clear();
    for (int pid : m_pidActorManager->getPidList())
    {
        const PidActorManager::PidSpec* spec = m_pidActorManager->getSpec(pid);
        vtkActor* src = m_pidActorManager->getActorByPid(pid);
        if (!spec || !src || !spec->inFrontCamera)
            continue;

        auto camActor = vtkSmartPointer<vtkActor>::New();
        camActor->SetMapper(src->GetMapper());
        camActor->GetProperty()->DeepCopy(src->GetProperty());
        camActor->GetProperty()->SetRepresentationToSurface();
        camActor->GetProperty()->EdgeVisibilityOff();

        m_frontCamRenderer->AddActor(camActor);
        m_frontCamActors[pid] = camActor;
    }

    vtkCamera* cam = m_frontCamRenderer->GetActiveCamera();
    cam->SetViewAngle(m_frontCamViewAngle);
    cam->SetViewUp(0.0, 0.0, 1.0);
    cam->SetClippingRange(0.05, 450.0);

    // daylight from above + headlight (positional spotlight at the lens)
    m_frontCamRenderer->AutomaticLightCreationOff();

    vtkNew<vtkLight> sun;
    sun->SetLightTypeToSceneLight();
    sun->PositionalOff();
    sun->SetPosition(0.4, 0.3, 1.0);
    sun->SetFocalPoint(0.0, 0.0, 0.0);
    sun->SetIntensity(0.85);
    m_frontCamRenderer->AddLight(sun);

    m_frontCamHeadlight = vtkSmartPointer<vtkLight>::New();
    m_frontCamHeadlight->SetLightTypeToSceneLight();
    m_frontCamHeadlight->PositionalOn();
    m_frontCamHeadlight->SetConeAngle(50.0);      // half angle [deg]
    m_frontCamHeadlight->SetExponent(2.0);        // soft falloff toward the cone edge
    m_frontCamHeadlight->SetAttenuationValues(1.0, 0.0, 0.0);  // no distance falloff
    m_frontCamHeadlight->SetColor(1.0, 0.97, 0.9);
    m_frontCamHeadlight->SetIntensity(0.5);
    m_frontCamRenderer->AddLight(m_frontCamHeadlight);

    m_renderWindow->AddRenderer(m_frontCamRenderer);
}

void MainWindow::updateFrontCamera()
{
    if (!m_frontCamRenderer)
        return;

    for (auto& [pid, camActor] : m_frontCamActors)
    {
        if (vtkActor* src = m_pidActorManager->getActorByPid(pid))
        {
            camActor->SetUserMatrix(src->GetUserMatrix());
            camActor->SetVisibility(src->GetVisibility());
        }
    }

    // the lens rides on the body with its true (not exaggerated) motion
    const auto& mount = m_vehicle.spec().cameraMount;
    const double pitchDeg = (m_vehicle.type() == VehicleType::Truck) ? 12.0 : 7.0;
    double pos[3], dir[3], up[3];
    transformPoint(m_bodyTrueMatrix, mount.data(), pos);
    const double pitch = vtkMath::RadiansFromDegrees(pitchDeg);
    const double dirL[3] = { std::cos(pitch), 0.0, -std::sin(pitch) };  // forward, pitched down
    const double upL[3] = { std::sin(pitch), 0.0, std::cos(pitch) };    // orthogonal to dir
    transformPoint(m_bodyTrueMatrix, dirL, dir, 0.0);
    transformPoint(m_bodyTrueMatrix, upL, up, 0.0);

    vtkCamera* cam = m_frontCamRenderer->GetActiveCamera();
    cam->SetPosition(pos[0], pos[1], pos[2]);
    cam->SetFocalPoint(pos[0] + dir[0], pos[1] + dir[1], pos[2] + dir[2]);
    cam->SetViewUp(up[0], up[1], up[2]);
    cam->SetClippingRange(0.05, 450.0);

    // headlight shares the camera axis, so it is pitched by the same angle
    if (m_frontCamHeadlight)
    {
        double fp[3];
        cam->GetFocalPoint(fp);
        m_frontCamHeadlight->SetPosition(pos[0], pos[1], pos[2]);
        m_frontCamHeadlight->SetFocalPoint(fp[0], fp[1], fp[2]);
    }
}

// ===========================================================================
// mini map inset
// ===========================================================================

void MainWindow::setupMiniMap()
{
    m_miniRenderer = vtkSmartPointer<vtkRenderer>::New();
    m_miniRenderer->SetViewport(0.0, 0.0, 0.22, 0.30);
    m_miniRenderer->SetBackground(0.30, 0.42, 0.22);
    m_miniRenderer->SetLayer(1);
    m_miniRenderer->PreserveColorBufferOff();
    m_miniRenderer->InteractiveOff();

    for (int pid : m_pidActorManager->getPidList())
    {
        const PidActorManager::PidSpec* spec = m_pidActorManager->getSpec(pid);
        vtkActor* src = m_pidActorManager->getActorByPid(pid);
        if (!spec || !src || !spec->inMiniMap)
            continue;

        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(src->GetMapper());
        actor->GetProperty()->DeepCopy(src->GetProperty());
        actor->GetProperty()->EdgeVisibilityOff();
        actor->GetProperty()->SetAmbient(0.4);
        m_miniRenderer->AddActor(actor);
        m_miniActors[pid] = actor;
    }

    // hazard discs (white = not reached yet, severity colour afterwards)
    m_miniHazardMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_miniHazardMapper->SetScalarModeToUseCellFieldData();
    m_miniHazardMapper->SelectColorArray("Color");
    m_miniHazardMapper->SetColorModeToDirectScalars();
    vtkNew<vtkActor> hazardActor;
    hazardActor->SetMapper(m_miniHazardMapper);
    hazardActor->GetProperty()->SetAmbient(1.0);
    hazardActor->GetProperty()->SetDiffuse(0.0);
    m_miniRenderer->AddActor(hazardActor);

    // vehicle marker: big arrow-head in the vehicle frame
    m_miniVehicleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_miniVehicleMapper->ScalarVisibilityOff();
    m_miniVehicle = vtkSmartPointer<vtkActor>::New();
    m_miniVehicle->SetMapper(m_miniVehicleMapper);
    m_miniVehicle->GetProperty()->SetColor(1.0, 0.15, 0.1);
    m_miniVehicle->GetProperty()->SetAmbient(1.0);
    m_miniVehicle->GetProperty()->SetDiffuse(0.0);
    m_miniRenderer->AddActor(m_miniVehicle);

    m_renderWindow->AddRenderer(m_miniRenderer);
    rebuildMiniMap();
}

void MainWindow::rebuildMiniMap()
{
    const auto b = m_road.bounds();
    const double ext = std::max(b[1] - b[0], b[3] - b[2]);

    MeshBuilder mb;
    const double L = 0.035 * ext;
    mb.addTriangle({ 0.6 * L, 0.0, 80.0 }, { -0.45 * L, -0.38 * L, 80.0 }, { -0.45 * L, 0.38 * L, 80.0 });
    m_miniVehicleMapper->SetInputData(mb.buildPolyData());

    vtkCamera* cam = m_miniRenderer->GetActiveCamera();
    const double cx = 0.5 * (b[0] + b[1]);
    const double cy = 0.5 * (b[2] + b[3]);
    cam->ParallelProjectionOn();
    cam->SetFocalPoint(cx, cy, 0.0);
    cam->SetPosition(cx, cy, 2000.0);
    cam->SetViewUp(0.0, 1.0, 0.0);
    cam->SetClippingRange(100.0, 3000.0);
}

void MainWindow::updateMiniMap()
{
    if (!m_miniRenderer)
        return;

    m_miniVehicle->SetUserMatrix(m_yawMatrix);

    // fit the map (without most of its margin) into the viewport
    const auto b = m_road.bounds();
    const double w = 0.9 * (b[1] - b[0]);
    const double h = 0.9 * (b[3] - b[2]);
    double aspect = m_miniRenderer->GetTiledAspectRatio();
    if (aspect <= 0.0)
        aspect = 1.0;
    m_miniRenderer->GetActiveCamera()->SetParallelScale(0.5 * std::max(h, w / aspect));
}

// ===========================================================================
// picking / tree
// ===========================================================================

void MainWindow::setSurfaceMode()
{
    m_pidActorManager->setSurfaceMode();
    renderAll();
    statusBar()->showMessage("Surface mode");
}

void MainWindow::setWireframeMode()
{
    m_pidActorManager->setWireframeMode();
    renderAll();
    statusBar()->showMessage("Wireframe mode");
}

void MainWindow::onTreeItemChanged(QTreeWidgetItem* item, int column)
{
    Q_UNUSED(column);

    if (!item)
        return;

    int pid = item->data(0, Qt::UserRole).toInt();
    if (pid <= 0)
        return;  // group item: its children emit their own change

    bool visible = (item->checkState(0) == Qt::Checked);

    m_pidActorManager->setVisibility(pid, visible);
    updateFrontCamera();
    renderAll();

    statusBar()->showMessage(
        QString("PID %1 visibility: %2")
            .arg(pid)
            .arg(visible ? "ON" : "OFF"));
}

void MainWindow::onLeftButtonPress(
    vtkObject* caller,
    unsigned long eid,
    void* clientData,
    void* callData)
{
    Q_UNUSED(caller);
    Q_UNUSED(eid);
    Q_UNUSED(callData);

    auto* self = static_cast<MainWindow*>(clientData);
    if (self)
    {
        self->handleLeftButtonPress();
    }
}

void MainWindow::handleLeftButtonPress()
{
    auto* interactor = m_renderWindow->GetInteractor();
    if (!interactor || !m_cellPicker)
        return;

    int* clickPos = interactor->GetEventPosition();

    m_cellPicker->Pick(clickPos[0], clickPos[1], 0.0, m_renderer);

    vtkActor* pickedActor = m_cellPicker->GetActor();
    vtkIdType pickedCellId = m_cellPicker->GetCellId();

    // ---- hazard placement mode: the click installs a hazard instead of selecting a PID ----
    if (m_placeClickAction && m_placeClickAction->isChecked())
    {
        // keep the camera still for this click
        m_leftButtonPressCallback->SetAbortFlag(1);

        const int pid = pickedActor ? m_pidActorManager->findPidByActor(pickedActor) : -1;
        double p[3];
        m_cellPicker->GetPickPosition(p);
        double s = 0.0, d = 0.0;
        if (!pickedActor || pickedCellId < 0 || (pid > 0 && pid < 10) || p[2] > 0.6)
        {
            statusBar()->showMessage("Placement: click on the road surface (not on the vehicle or buildings)");
            return;
        }
        if (!m_road.project(p[0], p[1], s, d) || std::abs(d) > 15.0)
        {
            statusBar()->showMessage("Placement: click on the road the vehicle drives on");
            return;
        }
        placeHazard(s, d);
        return;
    }

    if (!pickedActor || pickedCellId < 0)
    {
        m_pidActorManager->clearSelection();
        renderAll();
        statusBar()->showMessage("No cell picked");
        return;
    }

    int pid = m_pidActorManager->findPidByActor(pickedActor);
    if (pid < 0)
    {
        m_pidActorManager->clearSelection();
        renderAll();
        statusBar()->showMessage("Picked actor is not managed by PID manager");
        return;
    }

    m_pidActorManager->setSelectedPid(pid);
    selectTreeItemByPid(pid);

    renderAll();

    QString extra;
    if (pid == Pid::Tires && pickedCellId < static_cast<vtkIdType>(m_tireCellWheel.size()))
    {
        const int w = m_tireCellWheel[pickedCellId];
        extra = QString(" | wheel %1: load %2 kN (%3 x static), damage %4")
                    .arg(QString::fromStdString(m_vehicle.wheel(w).name))
                    .arg(m_vehicle.tireForce(w) / 1000.0, 0, 'f', 1)
                    .arg(m_vehicle.loadRatio(w), 0, 'f', 2)
                    .arg(fmtDamage(m_analyzer.wheelDamage(w)));
    }

    statusBar()->showMessage(
        QString("Picked PID %1, local cell id = %2%3")
            .arg(pid)
            .arg(static_cast<long long>(pickedCellId))
            .arg(extra));
}

void MainWindow::selectTreeItemByPid(int pid)
{
    if (!m_treeWidget)
        return;

    for (int g = 0; g < m_treeWidget->topLevelItemCount(); ++g)
    {
        QTreeWidgetItem* group = m_treeWidget->topLevelItem(g);
        for (int i = 0; group && i < group->childCount(); ++i)
        {
            QTreeWidgetItem* item = group->child(i);
            if (item && item->data(0, Qt::UserRole).toInt() == pid)
            {
                m_treeWidget->setCurrentItem(item);
                return;
            }
        }
    }
}

void MainWindow::onCurrentTreeItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous)
{
    Q_UNUSED(previous);

    const int pid = current ? current->data(0, Qt::UserRole).toInt() : -1;
    if (pid <= 0)
    {
        m_pidActorManager->clearSelection();
        renderAll();
        statusBar()->showMessage("Selection cleared");
        return;
    }

    m_pidActorManager->setSelectedPid(pid);
    renderAll();

    statusBar()->showMessage(QString("PID %1 selected").arg(pid));
}

// ===========================================================================
// motion control
// ===========================================================================

void MainWindow::startMotion()
{
    if (!m_motionTimer)
        return;

    if (courseFinished())
    {
        recordRun();
        resetSimulation();
    }

    if (!m_motionTimer->isActive())
    {
        if (m_simTime == 0.0 && m_sideTabs)
            m_sideTabs->setCurrentIndex(1);  // show the loads while driving

        // Start the timer with the configured time step in milliseconds
        m_frameClock.invalidate();
        m_motionTimer->start(static_cast<int>(m_motionDt * 1000.0));
        statusBar()->showMessage(QString("Driving | %1 on %2 | time scale x%3")
                                     .arg(VehicleModel::typeName(m_vehicle.type()))
                                     .arg(RoadNetwork::mapName(m_road.type()))
                                     .arg(m_timeScale));
    }
}

void MainWindow::stopMotion()
{
    if (!m_motionTimer)
        return;

    if (m_motionTimer->isActive())
    {
        m_motionTimer->stop();
        statusBar()->showMessage(QString("Stopped at s = %1 km").arg(m_s / 1000.0, 0, 'f', 3));
    }
}

void MainWindow::resetMotion()
{
    if (m_motionTimer && m_motionTimer->isActive())
    {
        m_motionTimer->stop();
    }

    recordRun();
    resetSimulation();
    setCameraPreset(m_cameraPreset);
    renderAll();

    statusBar()->showMessage("Reset");
}

void MainWindow::updateMotion()
{
    // advance by the real elapsed time, so the simulation stays real-time even if a frame
    // takes longer than the timer interval (capped to avoid a jump after a stall)
    double wallDt = m_motionDt;
    if (m_frameClock.isValid())
        wallDt = std::clamp(m_frameClock.nsecsElapsed() * 1e-9, 0.001, 0.1);
    m_frameClock.restart();

    simulateFrame(wallDt * m_timeScale);

    updateVisuals();
    updateFollowCamera();
    updateFrontCamera();
    updateMiniMap();
    if ((++m_frameCounter % 3) == 0)
        updateTables();
    m_plot->update();
    renderAll();

    if (courseFinished())
    {
        m_motionTimer->stop();
        updateTables();
        recordRun();
        char text[256];
        std::snprintf(text, sizeof(text),
                      "COURSE COMPLETE  |  %s  |  %.2f km  |  total damage %.3g  |  hazards %.0f%%",
                      VehicleModel::typeName(m_vehicle.type()), m_analyzer.distance() / 1000.0,
                      m_analyzer.totalDamage(),
                      m_analyzer.totalDamage() > 0.0 ? 100.0 * m_analyzer.hazardDamage() / m_analyzer.totalDamage() : 0.0);
        showAlert(text, { 0.6, 0.9, 1.0 }, 15000);
        renderAll();
        statusBar()->showMessage("Course complete - press Reset or Start to drive again");
    }
}

// ===========================================================================
// map / vehicle / hazards
// ===========================================================================

void MainWindow::loadMap(MapType type)
{
    if (m_motionTimer && m_motionTimer->isActive())
        m_motionTimer->stop();
    recordRun();

    m_road = RoadNetwork(type);
    m_userHazards.clear();
    m_customStartS = -1.0;
    m_startSection = 0;

    // keep the menu and the combo box in sync
    const int idx = mapIndex(type);
    m_mapCombo->blockSignals(true);
    m_mapCombo->setCurrentIndex(idx);
    m_mapCombo->blockSignals(false);
    for (QAction* a : m_mapGroup->actions())
        a->setChecked(a->data().toInt() == idx);
    refreshSectionCombo();

    applyMapMeshes();
    rebuildZoneSigns();
    rebuildMiniMap();
    resetSimulation();
    setCameraPreset(m_cameraPreset);
    renderAll();

    statusBar()->showMessage(QString("Map: %1 (%2 km, %3 hazards)")
                                 .arg(mapLabel(type))
                                 .arg(m_road.courseEnd() / 1000.0, 0, 'f', 2)
                                 .arg(m_road.hazards().size()));
}

void MainWindow::changeVehicle(VehicleType type)
{
    if (m_motionTimer && m_motionTimer->isActive())
        m_motionTimer->stop();
    recordRun();

    m_vehicle.setType(type);

    const int idx = vehicleIndex(type);
    m_vehicleCombo->blockSignals(true);
    m_vehicleCombo->setCurrentIndex(idx);
    m_vehicleCombo->blockSignals(false);
    for (QAction* a : m_vehicleGroup->actions())
        a->setChecked(a->data().toInt() == idx);

    buildVehicleMeshes();
    applyVehicleMeshes();
    refreshPayloadControls();
    refreshWheelWidgets();
    resetSimulation();
    setCameraPreset(m_cameraPreset);
    renderAll();

    statusBar()->showMessage(QString("Vehicle: %1, total %2 t")
                                 .arg(vehicleLabel(type))
                                 .arg(m_vehicle.totalMass() / 1000.0, 0, 'f', 2));
}

bool MainWindow::placeHazard(double s, double pointD)
{
    const auto kind = static_cast<HazardKind>(m_hazardKindCombo->currentIndex());
    const auto lane = static_cast<HazardLane>(m_hazardLaneCombo->currentIndex());

    std::string error;
    const int id = m_road.addHazard(kind, s, lane, pointD, &error);
    if (id < 0)
    {
        const QString msg = QString("Cannot place hazard: %1").arg(QString::fromStdString(error));
        showAlert(msg.toStdString(), { 1.0, 0.6, 0.4 }, 2500);
        statusBar()->showMessage(msg);
        renderAll();
        return false;
    }

    m_userHazards.push_back(id);
    const Hazard* h = m_road.findHazard(id);
    onHazardsChanged(QString("Placed %1 at %2 m%3")
                         .arg(QString::fromStdString(h->label))
                         .arg(s, 0, 'f', 0)
                         .arg(s < m_s ? "  (behind the vehicle: reset to drive over it)" : ""));
    return true;
}

void MainWindow::placeHazardAhead()
{
    const double ahead = m_hazardDistanceSpin ? m_hazardDistanceSpin->value() : 80.0;
    placeHazard(m_s + ahead, 0.0);
}

void MainWindow::onHazardsChanged(const QString& message)
{
    m_pidActorManager->setData(Pid::Potholes, m_road.buildPotholes());
    m_pidActorManager->setData(Pid::Bumps, m_road.buildBumps());
    m_analyzer.syncHazards(m_road, m_s);
    rebuildHazardOverlays();
    updateHud();
    renderAll();
    statusBar()->showMessage(message);
}

void MainWindow::undoHazard()
{
    while (!m_userHazards.empty())
    {
        const int id = m_userHazards.back();
        m_userHazards.pop_back();
        const Hazard* h = m_road.findHazard(id);
        if (!h)
            continue;
        const QString label = QString::fromStdString(h->label);
        m_road.removeHazard(id);
        onHazardsChanged(QString("Removed %1").arg(label));
        return;
    }
    statusBar()->showMessage("Nothing to undo");
}

void MainWindow::clearHazards()
{
    m_road.clearHazards();
    m_userHazards.clear();
    onHazardsChanged("All hazards removed - place your own (Hazards menu or 'Place hazards' panel)");
}

void MainWindow::restoreHazards()
{
    m_road.restoreDefaultHazards();
    m_userHazards.clear();
    onHazardsChanged("Default hazards restored");
}

void MainWindow::onHazardKindChanged(int index)
{
    const bool pothole = RoadNetwork::isPotholeKind(static_cast<HazardKind>(index));
    m_hazardLaneCombo->setEnabled(pothole);
    for (QAction* a : m_hazardKindGroup->actions())
        a->setChecked(a->data().toInt() == index);
}

void MainWindow::setManualSpeed(int kmh, bool switchToManual)
{
    m_manualSpeedKmh = std::clamp(kmh, 5, 130);
    m_speedSlider->blockSignals(true);
    m_speedSlider->setValue(m_manualSpeedKmh);
    m_speedSlider->blockSignals(false);
    m_speedSpin->blockSignals(true);
    m_speedSpin->setValue(m_manualSpeedKmh);
    m_speedSpin->blockSignals(false);
    m_speedLabel->setText(QString("%1 km/h").arg(m_manualSpeedKmh));
    if (switchToManual && m_autoSpeedAction->isChecked())
        m_autoSpeedAction->setChecked(false);
    statusBar()->showMessage(QString("Fixed speed %1 km/h").arg(m_manualSpeedKmh));
}

// ===========================================================================
// simulation
// ===========================================================================

void MainWindow::resetSimulation()
{
    const auto& sections = m_road.sections();
    const int sec = std::clamp(m_startSection, 0, static_cast<int>(sections.size()) - 1);
    m_s = (m_customStartS >= 0.0) ? m_customStartS : sections[sec].s;
    m_simTime = 0.0;
    m_accel = 0.0;
    m_wheelSpin.fill(0.0);
    m_vehicle.resetDynamics();
    m_v = 0.0;
    m_v = targetSpeed();  // flying start at the planned speed

    m_analyzer.reset(m_road, m_vehicle, m_s);
    m_runRecorded = false;

    if (m_eventTable)
        m_eventTable->setRowCount(0);
    if (m_plot)
        m_plot->clear();

    rebuildHazardOverlays();
    m_alertUntilMs = 0;
    if (m_alertActor)
        m_alertActor->SetVisibility(0);

    resetTrail();
    updateVisuals();
    updateFrontCamera();
    updateMiniMap();
    updateTables();
}

double MainWindow::targetSpeed() const
{
    // look ahead by ~1.5 s so the speed controller lag is compensated, and do not speed up
    // before the whole vehicle has left a bend
    const double lookahead = m_s + 1.5 * m_v;
    const double rear = m_s + m_vehicle.wheel(m_vehicle.wheelCount() - 1).x - 1.0;
    const double corner = m_road.cornerSpeed(rear, lookahead);
    if (m_autoSpeed)
        return std::min(corner, m_road.plannedSpeed(lookahead, kPlannerDecel, m_vehicle.spec().maxSpeedKmh / 3.6, true));
    return std::min(corner, m_road.plannedSpeed(lookahead, kPlannerDecel, m_manualSpeedKmh / 3.6, false));
}

bool MainWindow::courseFinished() const
{
    return m_s > m_road.courseEnd() - 40.0 && m_v < 0.05;
}

void MainWindow::recordRun()
{
    if (!m_runTable || m_runRecorded || m_analyzer.distance() < 100.0)
        return;
    m_runRecorded = true;

    double maxRatio = 0.0;
    for (int i = 0; i < m_vehicle.wheelCount(); ++i)
        maxRatio = std::max(maxRatio, m_analyzer.wheelPeakRatio(i));

    const int row = m_runTable->rowCount();
    m_runTable->insertRow(row);
    m_runTable->setItem(row, 0, centered(VehicleModel::typeName(m_vehicle.type())));
    m_runTable->setItem(row, 1, centered(RoadNetwork::mapName(m_road.type())));
    m_runTable->setItem(row, 2, centered(m_autoSpeed ? QString("auto") : QString("%1 km/h").arg(m_manualSpeedKmh)));
    m_runTable->setItem(row, 3, centered(QString::number(m_analyzer.distance() / 1000.0, 'f', 2)));
    QTableWidgetItem* d = centered(fmtDamage(m_analyzer.totalDamage()));
    const QColor dc = lutColor(m_damageLut, damageScalar(m_analyzer.totalDamage()));
    d->setBackground(dc);
    d->setForeground(readableText(dc));
    m_runTable->setItem(row, 4, d);
    m_runTable->setItem(row, 5, centered(QString::number(m_analyzer.peakForce() / 1000.0, 'f', 1)));
    m_runTable->setItem(row, 6, centered(QString("%1 (max %2x)").arg(m_analyzer.finishedEventCount()).arg(maxRatio, 0, 'f', 2)));
    m_runTable->scrollToBottom();
}

void MainWindow::simulateFrame(double frameDt)
{
    const int nSub = std::max(1, static_cast<int>(std::ceil(frameDt / kPhysicsDt - 1e-9)));
    const double h = frameDt / nSub;
    const int n = m_vehicle.wheelCount();

    // simple speed controller (gentle acceleration, firm braking)
    const double maxAccel = (m_vehicle.type() == VehicleType::Truck) ? 1.2 : 2.0;
    const double accel = std::clamp((targetSpeed() - m_v) / 1.5, -3.0, maxAccel);

    VehicleModel::WheelArray zr{};
    double values[kMaxWheels];
    for (int k = 0; k < nSub; ++k)
    {
        m_v = std::max(0.0, m_v + accel * h);
        m_s += m_v * h;
        m_simTime += h;

        // lateral acceleration from the path curvature at mid-wheelbase
        const double ay = m_v * m_v * m_road.course().curvature(m_s + 0.5 * m_vehicle.wheel(n - 1).x);

        for (int i = 0; i < n; ++i)
        {
            const WheelDef& w = m_vehicle.wheel(i);
            zr[i] = m_road.wheelInput(m_s + w.x, w.y, w.radius);
        }
        m_vehicle.step(h, zr, accel, ay);
        m_analyzer.update(m_simTime, h, m_s, m_v, m_road, m_vehicle);
        updateTrailSubstep();

        if ((++m_plotDecimator & 1u) == 0u)
        {
            for (int i = 0; i < n; ++i)
                values[i] = m_vehicle.loadRatio(i);
            m_plot->addSample(m_simTime, values);
        }
    }
    m_accel = accel;

    for (int i = 0; i < n; ++i)
    {
        m_wheelSpin[i] = std::fmod(m_wheelSpin[i] + m_v * frameDt / m_vehicle.wheel(i).radius,
                                   2.0 * vtkMath::Pi());
    }

    for (const auto& ev : m_analyzer.takeFinishedEvents())
        handleFinishedEvent(ev);

    const HazardEvent* active = m_analyzer.activeEvent();
    const Hazard* activeHazard = active ? m_road.findHazard(active->hazardId) : nullptr;
    if (activeHazard)
        m_plot->setOpenSpan(true, active->tStart, QString::fromStdString(activeHazard->tag));
    else
        m_plot->setOpenSpan(false, 0.0, QString());
}

void MainWindow::handleFinishedEvent(const HazardEvent& ev)
{
    const Hazard* hp = m_road.findHazard(ev.hazardId);
    if (!hp)
        return;
    const Hazard& h = *hp;
    const Severity sev = severityOf(ev.peakRatio);
    const auto rgb = severityRgb(sev);
    const std::string wheelName = (ev.peakWheel >= 0) ? m_vehicle.wheel(ev.peakWheel).name : std::string("-");

    // ---- event table ----
    const int row = m_eventTable->rowCount();
    m_eventTable->insertRow(row);
    auto setCell = [&](int col, const QString& text)
    {
        QTableWidgetItem* item = centered(text);
        m_eventTable->setItem(row, col, item);
        return item;
    };
    QTableWidgetItem* hazardItem = setCell(0, QString::fromStdString(h.brief));
    hazardItem->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    hazardItem->setToolTip(QString("%1\n%2").arg(QString::fromStdString(h.label))
                                            .arg(QString::fromStdString(m_road.zoneAt(h.s).name)));
    setCell(1, QString::number(ev.speedKmh, 'f', 0));
    setCell(2, QString::number(ev.peakForce / 1000.0, 'f', 1));
    QTableWidgetItem* ratioItem = setCell(3, QString::number(ev.peakRatio, 'f', 2));
    ratioItem->setBackground(toQColor(rgb));
    ratioItem->setForeground(readableText(toQColor(rgb)));
    ratioItem->setToolTip(QString("%1 impact").arg(severityName(sev)));
    setCell(4, QString::fromStdString(wheelName));
    setCell(5, ev.liftOff ? "YES" : "-");
    QTableWidgetItem* dmgItem = setCell(6, fmtDamage(ev.damage));
    const QColor dmgColor = lutColor(m_damageLut, damageScalar(ev.damage));
    dmgItem->setBackground(dmgColor);
    dmgItem->setForeground(readableText(dmgColor));
    setCell(7, ev.equivalentKm >= 0.0 ? QString::number(ev.equivalentKm, 'f', ev.equivalentKm < 10.0 ? 1 : 0) : QString("-"));
    m_eventTable->scrollToBottom();

    // ---- 3D label above the hazard ----
    auto labelIt = m_hazardLabels.find(h.id);
    if (labelIt != m_hazardLabels.end())
    {
        labelIt->second->SetInput(hazardResultText(h, ev).c_str());
        labelIt->second->GetTextProperty()->SetColor(rgb[0], rgb[1], rgb[2]);
    }

    // ---- mini map marker ----
    auto cellIt = m_miniHazardCells.find(h.id);
    auto* colors = m_miniHazards ? vtkUnsignedCharArray::SafeDownCast(m_miniHazards->GetCellData()->GetArray("Color")) : nullptr;
    if (colors && cellIt != m_miniHazardCells.end())
    {
        const unsigned char c[3] = { static_cast<unsigned char>(rgb[0] * 255.0),
                                     static_cast<unsigned char>(rgb[1] * 255.0),
                                     static_cast<unsigned char>(rgb[2] * 255.0) };
        for (int k = 0; k < kMiniDiscSegments; ++k)
            colors->SetTypedTuple(cellIt->second + k, c);
        colors->Modified();
        m_miniHazards->Modified();
    }

    // ---- plot band ----
    m_plot->addSpan(ev.tStart, ev.tEnd, QString::fromStdString(h.tag), toQColor(rgb));

    // ---- alert ----
    char eq[64] = "";
    if (ev.equivalentKm >= 0.0)
        std::snprintf(eq, sizeof(eq), "  (= %.0f km of normal road)", ev.equivalentKm);
    char text[320];
    std::snprintf(text, sizeof(text), "%s impact:  %s  @ %.0f km/h\npeak %.2fx static load on %s (%.1f kN)%s\ndamage +%.3g%s",
                  severityName(sev), h.label.c_str(), ev.speedKmh,
                  ev.peakRatio, wheelName.c_str(), ev.peakForce / 1000.0,
                  ev.liftOff ? ",  wheel lift-off" : "", ev.damage, eq);
    showAlert(text, rgb, 4500);

    statusBar()->showMessage(QString("%1: peak %2 x static, damage %3")
                                 .arg(QString::fromStdString(h.label))
                                 .arg(ev.peakRatio, 0, 'f', 2)
                                 .arg(fmtDamage(ev.damage)));
}

// ===========================================================================
// visual update
// ===========================================================================

void MainWindow::updateVisuals()
{
    const int n = m_vehicle.wheelCount();

    // world <- vehicle frame. The heading follows the chord from the rear axle to the front
    // axle, so in tight city corners the body cuts the curve like a real vehicle.
    {
        const Pose2D front = m_road.course().pose(m_s);
        const double wb = -m_vehicle.wheel(n - 1).x;
        const Vec3 rear = m_road.course().point(m_s - wb, 0.0);
        const double heading = std::atan2(front.y - rear[1], front.x - rear[0]);
        vtkNew<vtkTransform> t;
        t->Translate(front.x, front.y, 0.0);
        t->RotateZ(rad2deg(heading));
        t->GetMatrix(m_yawMatrix);
    }

    // body: rotate about the CG. Deflections relative to the road plane are exaggerated
    // for display, so the vehicle still sits on bumps / in potholes.
    auto bodyRelative = [this](double z, double px, double py, vtkMatrix4x4* out)
    {
        vtkNew<vtkTransform> r;
        r->Translate(m_vehicle.cgX(), 0.0, m_vehicle.cgZ() + z);
        r->RotateY(rad2deg(-std::atan(px)));
        r->RotateX(rad2deg(std::atan(py)));
        r->Translate(-m_vehicle.cgX(), 0.0, -m_vehicle.cgZ());
        r->GetMatrix(out);
    };

    double c0, c1, c2;
    m_vehicle.roadPlane(c0, c1, c2);
    const double ex = m_exaggeration;
    const double zv = c0 + ex * (m_vehicle.heave() - c0);
    const double pxv = c1 + ex * (m_vehicle.pitchSlope() - c1);
    const double pyv = c2 + ex * (m_vehicle.rollSlope() - c2);

    // note: the static Multiply4x4 does not bump the output's MTime, so call Modified()
    // explicitly - otherwise the actors keep using their cached (stale) matrix
    bodyRelative(zv, pxv, pyv, m_bodyRelMatrix);
    vtkMatrix4x4::Multiply4x4(m_yawMatrix, m_bodyRelMatrix, m_bodyMatrix);
    m_bodyMatrix->Modified();

    vtkNew<vtkMatrix4x4> trueRel;
    bodyRelative(m_vehicle.heave(), m_vehicle.pitchSlope(), m_vehicle.rollSlope(), trueRel);
    vtkMatrix4x4::Multiply4x4(m_yawMatrix, trueRel, m_bodyTrueMatrix);
    m_bodyTrueMatrix->Modified();

    for (int pid : m_pidActorManager->getPidList())
    {
        if (isBodyPid(pid))
            m_pidActorManager->setUserMatrix(pid, m_bodyMatrix);
        else if (isVehicleFramePid(pid))
            m_pidActorManager->setUserMatrix(pid, m_yawMatrix);
    }

    VehicleModel::WheelArray zuVis{};
    for (int i = 0; i < n; ++i)
    {
        const double zr = m_vehicle.roadInput(i);
        zuVis[i] = zr + ex * (m_vehicle.wheelZ(i) - zr);
    }

    updateTireGeometry(zuVis);
    updateSuspensionGeometry(zuVis);
    updateArrowGeometry();

    m_trailPoints->Modified();
    m_trailPolys->Modified();
    m_trailScalars->Modified();
    m_trailPoly->DeleteCells();  // cell links are rebuilt on demand (picking)
    m_trailPoly->Modified();

    updateHazardLabels();
    updateHud();
}

void MainWindow::updateTireGeometry(const VehicleModel::WheelArray& zuVis)
{
    vtkPoints* pts = m_tirePoly->GetPoints();
    for (vtkIdType p = 0; p < static_cast<vtkIdType>(m_tireBase.size()); ++p)
    {
        const int i = m_tirePointWheel[p];
        const WheelDef& w = m_vehicle.wheel(i);
        const Vec3& b = m_tireBase[p];
        // rolling: rotation about the axle (+y), top of the tyre moves forward
        const double c = std::cos(m_wheelSpin[i]);
        const double s = std::sin(m_wheelSpin[i]);
        const double x = b[0] * c + b[2] * s;
        const double z = -b[0] * s + b[2] * c;
        pts->SetPoint(p, w.x + x, w.y + b[1], w.radius + zuVis[i] + z);
    }
    pts->Modified();

    auto* scalars = vtkDoubleArray::SafeDownCast(m_tirePoly->GetCellData()->GetArray("Scalar"));
    for (vtkIdType c = 0; scalars && c < static_cast<vtkIdType>(m_tireCellWheel.size()); ++c)
        scalars->SetValue(c, m_vehicle.loadRatio(m_tireCellWheel[c]));
    if (scalars)
        scalars->Modified();
    m_tirePoly->Modified();
}

void MainWindow::updateSuspensionGeometry(const VehicleModel::WheelArray& zuVis)
{
    const bool truck = (m_vehicle.type() == VehicleType::Truck);
    vtkPoints* pts = m_suspPoly->GetPoints();
    vtkIdType p = 0;
    auto setHex = [&](const std::array<Vec3, 8>& corners)
    {
        for (const auto& q : corners)
            pts->SetPoint(p++, q.data());
    };
    auto bodyToVehicle = [this](double x, double y, double z)
    {
        const double in[3] = { x, y, z };
        Vec3 out;
        transformPoint(m_bodyRelMatrix, in, out.data());
        return out;
    };

    // spring between the wheel (moves with the wheel) and its seat on the body
    for (int i = 0; i < m_vehicle.wheelCount(); ++i)
    {
        const WheelDef& w = m_vehicle.wheel(i);
        const double ys = truck ? 0.5 * w.side : w.side * (std::abs(w.y) - 0.5 * w.width - 0.09);
        const double hl = truck ? 0.55 : 0.07;   // leaf spring vs. coil spring
        const double hw = truck ? 0.07 : 0.06;
        const double x0 = w.x - hl, x1 = w.x + hl;
        const double y0 = ys - hw, y1 = ys + hw;
        const double zb = w.radius + zuVis[i] + (truck ? 0.09 : 0.0);
        setHex({ Vec3{ x0, y0, zb }, Vec3{ x1, y0, zb }, Vec3{ x1, y1, zb }, Vec3{ x0, y1, zb },
                 bodyToVehicle(x0, y0, m_springTopZ), bodyToVehicle(x1, y0, m_springTopZ),
                 bodyToVehicle(x1, y1, m_springTopZ), bodyToVehicle(x0, y1, m_springTopZ) });
    }

    // axle beams / drive shafts between the left and right wheel centres
    for (int a = 0; a < m_vehicle.axleCount(); ++a)
    {
        const int l = 2 * a;
        const int r = 2 * a + 1;
        const WheelDef& wl = m_vehicle.wheel(l);
        const WheelDef& wr = m_vehicle.wheel(r);
        const double x = wl.x;
        const double zl = wl.radius + zuVis[l];
        const double zr = wr.radius + zuVis[r];
        const double yl = wl.y, yr = wr.y;
        const double t = truck ? 0.08 : 0.035;
        setHex({ Vec3{ x - t, yr, zr - t }, Vec3{ x + t, yr, zr - t }, Vec3{ x + t, yl, zl - t }, Vec3{ x - t, yl, zl - t },
                 Vec3{ x - t, yr, zr + t }, Vec3{ x + t, yr, zr + t }, Vec3{ x + t, yl, zl + t }, Vec3{ x - t, yl, zl + t } });
    }
    pts->Modified();

    auto* scalars = vtkDoubleArray::SafeDownCast(m_suspPoly->GetCellData()->GetArray("Scalar"));
    const int n = m_vehicle.wheelCount();
    for (vtkIdType c = 0; scalars && c < static_cast<vtkIdType>(m_suspCellOwner.size()); ++c)
    {
        const int owner = m_suspCellOwner[c];
        double d;
        if (owner < n)
            d = m_analyzer.wheelDamage(owner);
        else
        {
            const int a = owner - n;
            d = std::max(m_analyzer.wheelDamage(2 * a), m_analyzer.wheelDamage(2 * a + 1));
        }
        scalars->SetValue(c, damageScalar(d));
    }
    if (scalars)
        scalars->Modified();
    m_suspPoly->Modified();
}

void MainWindow::updateArrowGeometry()
{
    const bool truck = (m_vehicle.type() == VehicleType::Truck);
    vtkPoints* pts = m_arrowPoly->GetPoints();
    auto* scalars = vtkDoubleArray::SafeDownCast(m_arrowPoly->GetCellData()->GetArray("Scalar"));

    for (int i = 0; i < m_vehicle.wheelCount(); ++i)
    {
        const WheelDef& w = m_vehicle.wheel(i);
        const double ratio = m_vehicle.loadRatio(i);
        const double x = w.x;
        const double y = w.side * (std::abs(w.y) + 0.5 * w.width + (truck ? 0.45 : 0.32));
        const double len = std::clamp(m_arrowScale * ratio, 0.03, 4.5 * m_arrowScale);
        const double z0 = 0.05;
        const double z1 = z0 + len;
        const double hw = truck ? 0.07 : 0.045;   // shaft half width
        const double hh = truck ? 0.20 : 0.13;    // head half width
        const double ht = truck ? 0.35 : 0.22;    // head height

        const Vec3 q[kArrowPoints] = {
            { x - hw, y - hw, z0 }, { x + hw, y - hw, z0 }, { x + hw, y + hw, z0 }, { x - hw, y + hw, z0 },
            { x - hw, y - hw, z1 }, { x + hw, y - hw, z1 }, { x + hw, y + hw, z1 }, { x - hw, y + hw, z1 },
            { x - hh, y - hh, z1 }, { x + hh, y - hh, z1 }, { x + hh, y + hh, z1 }, { x - hh, y + hh, z1 },
            { x, y, z1 + ht }
        };
        for (int k = 0; k < kArrowPoints; ++k)
            pts->SetPoint(i * kArrowPoints + k, q[k].data());
        for (int k = 0; scalars && k < kArrowCells; ++k)
            scalars->SetValue(i * kArrowCells + k, ratio);
    }
    pts->Modified();
    if (scalars)
        scalars->Modified();
    m_arrowPoly->Modified();
}

void MainWindow::resetTrail()
{
    m_trailPoints->Reset();
    m_trailPolys->Reset();
    m_trailScalars->Reset();
    m_trailStations = 0;

    double rear = 0.0;
    for (int i = 0; i < m_vehicle.wheelCount(); ++i)
        rear = std::min(rear, m_vehicle.wheel(i).x);
    m_trailS0 = m_s + rear - 1.0;

    // boundary 0
    const double z = 0.025;
    const RoadPath& path = m_road.course();
    const double o = m_trailOffset, hw = m_trailHalfWidth;
    m_trailPoints->InsertNextPoint(path.point(m_trailS0, -o - hw, z).data());
    m_trailPoints->InsertNextPoint(path.point(m_trailS0, -o + hw, z).data());
    m_trailPoints->InsertNextPoint(path.point(m_trailS0, o - hw, z).data());
    m_trailPoints->InsertNextPoint(path.point(m_trailS0, o + hw, z).data());

    // the road under the standing vehicle starts at the static load
    const int front = static_cast<int>(std::floor((m_s - m_trailS0) / kTrailStep));
    while (m_trailStations <= front)
        appendTrailStation(1.0);
}

void MainWindow::appendTrailStation(double initialValue)
{
    const int k = m_trailStations;
    const double s = m_trailS0 + (k + 1) * kTrailStep;
    const RoadPath& path = m_road.course();
    const double o = m_trailOffset, hw = m_trailHalfWidth;
    const double base = 0.025;
    const double zr = base + std::max(0.0, m_road.surfaceHeight(s, -o));
    const double zl = base + std::max(0.0, m_road.surfaceHeight(s, o));

    m_trailPoints->InsertNextPoint(path.point(s, -o - hw, zr).data());
    m_trailPoints->InsertNextPoint(path.point(s, -o + hw, zr).data());
    m_trailPoints->InsertNextPoint(path.point(s, o - hw, zl).data());
    m_trailPoints->InsertNextPoint(path.point(s, o + hw, zl).data());

    const vtkIdType a = 4 * k;
    const vtkIdType b = 4 * (k + 1);
    const vtkIdType right[4] = { a + 0, b + 0, b + 1, a + 1 };
    const vtkIdType left[4] = { a + 2, b + 2, b + 3, a + 3 };
    m_trailPolys->InsertNextCell(4, right);
    m_trailPolys->InsertNextCell(4, left);
    m_trailScalars->InsertNextValue(initialValue);
    m_trailScalars->InsertNextValue(initialValue);
    ++m_trailStations;
}

void MainWindow::updateTrailSubstep()
{
    for (int i = 0; i < m_vehicle.wheelCount(); ++i)
    {
        const WheelDef& w = m_vehicle.wheel(i);
        const int k = static_cast<int>(std::floor((m_s + w.x - m_trailS0) / kTrailStep));
        if (k < 0)
            continue;
        while (m_trailStations <= k)
            appendTrailStation(-1.0);

        const vtkIdType cell = 2 * k + (w.side > 0 ? 1 : 0);
        const double ratio = m_vehicle.loadRatio(i);
        if (ratio > m_trailScalars->GetValue(cell))
            m_trailScalars->SetValue(cell, ratio);
    }
}

void MainWindow::updateHazardLabels()
{
    // only around the vehicle, and not in a far-away overview (labels would pile up)
    double cam[3];
    m_renderer->GetActiveCamera()->GetPosition(cam);
    for (const auto& h : m_road.hazards())
    {
        auto it = m_hazardLabels.find(h.id);
        if (it == m_hazardLabels.end())
            continue;
        const double ds = h.s - m_s;
        const double* p = it->second->GetPosition();
        const double camDist = std::sqrt(vtkMath::Distance2BetweenPoints(cam, p));
        const bool near = ds > -90.0 && ds < 170.0 && camDist < 400.0;
        it->second->SetVisibility((m_showLabels && near) ? 1 : 0);
    }
}

void MainWindow::updateHud()
{
    const RoadZone& zone = m_road.zoneAt(m_s);

    std::string next = "-";
    const auto& hazards = m_road.hazards();
    for (size_t k = 0; k < hazards.size(); ++k)
    {
        const int st = m_analyzer.hazardState(static_cast<int>(k));
        if (st == LoadAnalyzer::Active)
        {
            next = hazards[k].label + "  << NOW";
            break;
        }
        if (st == LoadAnalyzer::Pending)
        {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "%s in %.0f m", hazards[k].label.c_str(),
                          hazards[k].s - hazards[k].halfExtent() - m_s);
            next = buf;
            break;
        }
    }

    const VehicleSpec& spec = m_vehicle.spec();
    char load[64];
    if (spec.type == VehicleType::Truck)
        std::snprintf(load, sizeof(load), "payload %.1f t  GVW %.1f t", m_vehicle.payload() / 1000.0, m_vehicle.totalMass() / 1000.0);
    else
        std::snprintf(load, sizeof(load), "load %.0f kg  total %.2f t", m_vehicle.payload(), m_vehicle.totalMass() / 1000.0);

    char mode[48];
    if (m_autoSpeed)
        std::snprintf(mode, sizeof(mode), "auto, limit %.0f", std::min(zone.speedLimitKmh, spec.maxSpeedKmh));
    else
        std::snprintf(mode, sizeof(mode), "set %d km/h", m_manualSpeedKmh);

    char text[720];
    std::snprintf(text, sizeof(text),
                  " %s | %s \n"
                  " %-13s limit %3.0f km/h \n"
                  " speed    %6.1f km/h  (%s)\n"
                  " distance %6.3f / %.3f km\n"
                  " %s\n"
                  " sim time %6.1f s   x%g\n"
                  " next: %s ",
                  RoadNetwork::mapName(m_road.type()), VehicleModel::typeName(m_vehicle.type()),
                  zone.name.c_str(), zone.speedLimitKmh,
                  m_v * 3.6, mode,
                  m_s / 1000.0, m_road.courseEnd() / 1000.0,
                  load,
                  m_simTime, m_timeScale,
                  next.c_str());
    m_hudActor->SetInput(text);

    if (m_alertUntilMs > 0 && m_clock.elapsed() > m_alertUntilMs)
    {
        m_alertActor->SetVisibility(0);
        m_alertUntilMs = 0;
    }
}

void MainWindow::showAlert(const std::string& text, const std::array<double, 3>& color, int durationMs)
{
    m_alertActor->SetInput(text.c_str());
    m_alertActor->GetTextProperty()->SetColor(color[0], color[1], color[2]);
    m_alertActor->SetVisibility(1);
    m_alertUntilMs = m_clock.elapsed() + durationMs;
}

void MainWindow::updateTables()
{
    if (!m_wheelTable)
        return;

    const int n = m_vehicle.wheelCount();
    for (int i = 0; i < n && i < m_wheelTable->rowCount(); ++i)
    {
        const double ratio = m_vehicle.loadRatio(i);
        const double peak = m_analyzer.wheelPeakRatio(i);
        const double dmg = m_analyzer.wheelDamage(i);

        m_wheelTable->item(i, 1)->setText(QString::number(m_vehicle.staticTireForce(i) / 1000.0, 'f', 1));
        m_wheelTable->item(i, 2)->setText(QString::number(m_vehicle.tireForce(i) / 1000.0, 'f', 1));

        QTableWidgetItem* r = m_wheelTable->item(i, 3);
        r->setText(QString::number(ratio, 'f', 2));
        const QColor rc = lutColor(m_loadLut, ratio);
        r->setBackground(rc);
        r->setForeground(readableText(rc));

        QTableWidgetItem* pk = m_wheelTable->item(i, 4);
        pk->setText(QString::number(peak, 'f', 2));
        const QColor pc = lutColor(m_loadLut, peak);
        pk->setBackground(pc);
        pk->setForeground(readableText(pc));

        QTableWidgetItem* d = m_wheelTable->item(i, 5);
        d->setText(fmtDamage(dmg));
        const QColor dc = lutColor(m_damageLut, damageScalar(dmg));
        d->setBackground(dc);
        d->setForeground(readableText(dc));
    }

    const double total = m_analyzer.totalDamage();
    const double hazard = m_analyzer.hazardDamage();
    const double rate = m_analyzer.backgroundRatePerKm();
    m_summaryLabel->setText(
        QString("Total pseudo-damage: <b>%1</b> &nbsp; (peak tyre load %2 kN)<br>"
                "from potholes / bumps: <b>%3</b> (%4 %)<br>"
                "from normal road roughness: %5 over %6 km (%7 per km)<br>"
                "hazard events: %8")
            .arg(fmtDamage(total))
            .arg(m_analyzer.peakForce() / 1000.0, 0, 'f', 1)
            .arg(fmtDamage(hazard))
            .arg(total > 0.0 ? 100.0 * hazard / total : 0.0, 0, 'f', 1)
            .arg(fmtDamage(m_analyzer.backgroundDamage()))
            .arg(m_analyzer.backgroundDistance() / 1000.0, 0, 'f', 2)
            .arg(rate > 0.0 ? fmtDamage(rate) : QString("-"))
            .arg(m_analyzer.finishedEventCount()));

    const RoadZone& zone = m_road.zoneAt(m_s);
    const double ay = m_v * m_v * m_road.course().curvature(m_s + 0.5 * m_vehicle.wheel(n - 1).x);
    int lifted = 0;
    double maxRatio = 0.0;
    for (int i = 0; i < n; ++i)
    {
        if (m_vehicle.tireForce(i) <= 0.0)
            ++lifted;
        maxRatio = std::max(maxRatio, m_vehicle.loadRatio(i));
    }
    m_liveLabel->setText(
        QString("<b>%1</b> on <b>%2</b><br>"
                "%3 (limit %4 km/h)<br>"
                "speed <b>%5 km/h</b> &nbsp; accel %6 m/s&sup2; &nbsp; lateral %7 m/s&sup2;<br>"
                "max wheel load %8 x static%9")
            .arg(VehicleModel::typeName(m_vehicle.type()))
            .arg(RoadNetwork::mapName(m_road.type()))
            .arg(QString::fromStdString(zone.name))
            .arg(zone.speedLimitKmh, 0, 'f', 0)
            .arg(m_v * 3.6, 0, 'f', 1)
            .arg(m_accel, 0, 'f', 2)
            .arg(ay, 0, 'f', 2)
            .arg(maxRatio, 0, 'f', 2)
            .arg(lifted > 0 ? QString(" &nbsp; <b style='color:#d33'>%1 wheel(s) airborne</b>").arg(lifted) : QString()));
}

// ===========================================================================
// cameras
// ===========================================================================

void MainWindow::updateFollowCamera()
{
    if (m_followCamera)
    {
        // keep the camera fixed relative to the vehicle frame, so orbiting / zooming by the
        // user is preserved while the vehicle drives (and turns) on
        vtkNew<vtkMatrix4x4> inv;
        vtkMatrix4x4::Invert(m_prevYawMatrix, inv);
        vtkNew<vtkMatrix4x4> delta;
        vtkMatrix4x4::Multiply4x4(m_yawMatrix, inv, delta);

        vtkCamera* cam = m_renderer->GetActiveCamera();
        double pos[3], fp[3], up[3], out[3];
        cam->GetPosition(pos);
        cam->GetFocalPoint(fp);
        cam->GetViewUp(up);
        transformPoint(delta, pos, out);
        cam->SetPosition(out);
        transformPoint(delta, fp, out);
        cam->SetFocalPoint(out);
        transformPoint(delta, up, out, 0.0);
        cam->SetViewUp(out);
    }
    m_prevYawMatrix->DeepCopy(m_yawMatrix);
    m_renderer->ResetCameraClippingRange();
}

void MainWindow::setCameraPreset(int preset)
{
    m_cameraPreset = preset;
    vtkCamera* cam = m_renderer->GetActiveCamera();
    cam->SetViewAngle(preset == 1 ? 55.0 : 35.0);

    if (preset == 4)
    {
        // whole map, almost top-down
        m_followAction->setChecked(false);
        const auto b = m_road.bounds();
        const double cx = 0.5 * (b[0] + b[1]);
        const double cy = 0.5 * (b[2] + b[3]);
        double aspect = m_renderer->GetTiledAspectRatio();
        if (aspect <= 0.0)
            aspect = 1.4;
        const double half = 0.5 * std::max(b[3] - b[2], (b[1] - b[0]) / aspect) * 0.92;
        const double dist = half / std::tan(vtkMath::RadiansFromDegrees(0.5 * cam->GetViewAngle()));
        cam->SetFocalPoint(cx, cy, 0.0);
        cam->SetPosition(cx, cy - 0.15 * dist, dist);
        cam->SetViewUp(0.0, 0.0, 1.0);
        cam->OrthogonalizeViewUp();
    }
    else
    {
        if (!m_followAction->isChecked())
            m_followAction->setChecked(true);

        // presets scale with the vehicle size
        const VehicleSpec& spec = m_vehicle.spec();
        const double length = spec.frontX - spec.rearX;
        const double sc = std::clamp(length / 7.7, 0.62, 1.0);
        const double midX = 0.5 * (spec.frontX + spec.rearX);
        const double halfW = 0.5 * spec.width;
        const double H = spec.height;

        double pos[3], fp[3], up[3] = { 0.0, 0.0, 1.0 };
        switch (preset)
        {
        case 1:  // wheels: right side, above the kerb / guard rail, looking down at the tyres
            pos[0] = midX; pos[1] = -(halfW + 3.4); pos[2] = std::max(3.2, 1.35 * H);
            fp[0] = midX;  fp[1] = -0.5 * halfW;    fp[2] = 0.12 * H;
            break;
        case 2:  // top
            pos[0] = midX; pos[1] = 0.0; pos[2] = 46.0 * sc;
            fp[0] = midX;  fp[1] = 0.0;  fp[2] = 0.0;
            up[0] = 1.0; up[2] = 0.0;
            break;
        case 3:  // front quarter
            pos[0] = spec.frontX + 13.5 * sc; pos[1] = -7.5 * sc; pos[2] = 3.2 * sc;
            fp[0] = midX + 1.5 * sc;           fp[1] = 0.0;       fp[2] = 0.35 * H;
            break;
        default:  // chase (stays clear of houses at the roadside)
            pos[0] = midX - 2.0 - 18.5 * sc; pos[1] = -4.0 * sc; pos[2] = 9.5 * sc;
            fp[0] = midX + 0.5;              fp[1] = 0.0;        fp[2] = 0.42 * H;
            break;
        }
        double wp[3], wf[3], wu[3];
        transformPoint(m_yawMatrix, pos, wp);
        transformPoint(m_yawMatrix, fp, wf);
        transformPoint(m_yawMatrix, up, wu, 0.0);
        cam->SetPosition(wp);
        cam->SetFocalPoint(wf);
        cam->SetViewUp(wu);
        cam->OrthogonalizeViewUp();
    }

    m_prevYawMatrix->DeepCopy(m_yawMatrix);
    m_renderer->ResetCameraClippingRange();
}

void MainWindow::renderAll()
{
    if (m_renderWindow)
        m_renderWindow->Render();
}

// ===========================================================================
// control slots
// ===========================================================================

void MainWindow::onMapIndexChanged(int index)
{
    loadMap(kMaps[std::clamp(index, 0, 2)]);
}

void MainWindow::onVehicleIndexChanged(int index)
{
    changeVehicle(kVehicles[std::clamp(index, 0, 3)]);
}

void MainWindow::onStartSectionChanged(int index)
{
    m_startSection = index;
    m_customStartS = -1.0;
    resetMotion();
}

void MainWindow::onSpeedSliderChanged(int value)
{
    setManualSpeed(value, true);
}

void MainWindow::onSpeedSpinChanged(int value)
{
    setManualSpeed(value, true);
}

void MainWindow::onAutoSpeedToggled(bool on)
{
    m_autoSpeed = on;
    statusBar()->showMessage(on ? "Auto speed: the driver follows the speed limits"
                                : QString("Fixed speed %1 km/h").arg(m_manualSpeedKmh));
    if (m_renderWindow)
    {
        updateHud();
        renderAll();
    }
}

void MainWindow::faster()
{
    const int base = m_autoSpeed ? static_cast<int>(std::round(m_v * 3.6 / 5.0) * 5.0) : m_manualSpeedKmh;
    setManualSpeed(base + 5, true);
}

void MainWindow::slower()
{
    const int base = m_autoSpeed ? static_cast<int>(std::round(m_v * 3.6 / 5.0) * 5.0) : m_manualSpeedKmh;
    setManualSpeed(base - 5, true);
}

void MainWindow::onPayloadChanged(int value)
{
    if (m_motionTimer && m_motionTimer->isActive())
        m_motionTimer->stop();
    recordRun();

    m_vehicle.setPayload(value * m_vehicle.spec().payloadStep);
    refreshPayloadControls();
    if (m_renderWindow)
    {
        resetSimulation();
        setCameraPreset(m_cameraPreset);
        renderAll();
    }
}

void MainWindow::onTimeScaleChanged(int index)
{
    m_timeScale = kTimeScales[std::clamp(index, 0, 5)];
}

void MainWindow::onExaggerationChanged(double value)
{
    m_exaggeration = value;
    updateVisuals();
    renderAll();
}

void MainWindow::onExponentChanged(double value)
{
    m_analyzer.setExponent(value);
    statusBar()->showMessage(QString("S-N slope k = %1 (applies to load cycles from now on)").arg(value));
}

void MainWindow::onFollowToggled(bool on)
{
    m_followCamera = on;
    m_prevYawMatrix->DeepCopy(m_yawMatrix);
}

void MainWindow::onHazardLabelsToggled(bool on)
{
    m_showLabels = on;
    if (m_renderWindow)
    {
        updateHazardLabels();
        renderAll();
    }
}

// ===========================================================================
// command-line helpers
// ===========================================================================

void MainWindow::setMap(MapType type)
{
    loadMap(type);
}

void MainWindow::setVehicle(VehicleType type)
{
    changeVehicle(type);
}

void MainWindow::setStartSection(int index)
{
    m_startCombo->setCurrentIndex(std::clamp(index, 0, m_startCombo->count() - 1));
}

void MainWindow::setStartPosition(double s)
{
    m_customStartS = std::clamp(s, 0.0, m_road.courseEnd() - 60.0);
    resetMotion();
}

void MainWindow::setTimeScale(double scale)
{
    int best = 0;
    for (int i = 0; i < 6; ++i)
    {
        if (std::abs(kTimeScales[i] - scale) < std::abs(kTimeScales[best] - scale))
            best = i;
    }
    m_timeScaleCombo->setCurrentIndex(best);
}

void MainWindow::setSpeedKmh(int kmh)
{
    setManualSpeed(kmh, true);
    resetMotion();  // flying start at the new speed
}

void MainWindow::setHazardKind(int kind)
{
    m_hazardKindCombo->setCurrentIndex(std::clamp(kind, 0, static_cast<int>(HazardKind::Count) - 1));
}

void MainWindow::setCameraView(int preset)
{
    setCameraPreset(preset);
    updateHazardLabels();
    renderAll();
}

bool MainWindow::saveScreenshot(const QString& path)
{
    const bool ok = grab().save(path);
    statusBar()->showMessage(ok ? QString("Screenshot saved: %1").arg(path)
                                : QString("Could not save screenshot: %1").arg(path));
    return ok;
}
