#pragma once

#include <QElapsedTimer>
#include <QMainWindow>
#include <QTimer>    // for animation

#include <array>
#include <map>
#include <string>
#include <vector>

#include <vtkSmartPointer.h>

#include "LoadAnalyzer.h"
#include "MeshBuilder.h"
#include "PidActorManager.h"
#include "RoadNetwork.h"
#include "VehicleModel.h"

class QAction;
class QActionGroup;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QSlider;
class QSpinBox;
class QTabWidget;
class QTableWidget;
class QTreeWidget;
class QTreeWidgetItem;
class QVTKOpenGLNativeWidget;

class vtkActor;
class vtkBillboardTextActor3D;
class vtkCallbackCommand;
class vtkCellArray;
class vtkCellPicker;
class vtkDoubleArray;
class vtkGenericOpenGLRenderWindow;
class vtkLight;
class vtkLookupTable;
class vtkMatrix4x4;
class vtkObject;
class vtkPoints;
class vtkPolyData;
class vtkPolyDataMapper;
class vtkRenderer;
class vtkScalarBarActor;
class vtkTextActor;
class vtkUnstructuredGrid;

class LoadPlotWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    // used by command-line options (main.cpp)
    void setMap(MapType type);
    void setVehicle(VehicleType type);
    void setStartSection(int index);
    void setStartPosition(double s);  // arbitrary course position [m]
    void setTimeScale(double scale);
    void setSpeedKmh(int kmh);        // fixed (manual) speed
    void setHazardKind(int kind);
    void setCameraView(int preset);
    void startSimulation() { startMotion(); }
    bool saveScreenshot(const QString& path);

public slots:
    void placeHazardAhead();

private slots:
    void setSurfaceMode();
    void setWireframeMode();
    void onTreeItemChanged(QTreeWidgetItem* item, int column);
    void onCurrentTreeItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);

    // motion control
    void startMotion();
    void stopMotion();
    void resetMotion();
    void updateMotion();

    // map / vehicle / speed
    void onMapIndexChanged(int index);
    void onVehicleIndexChanged(int index);
    void onStartSectionChanged(int index);
    void onSpeedSliderChanged(int value);
    void onSpeedSpinChanged(int value);
    void onAutoSpeedToggled(bool on);
    void faster();
    void slower();
    void onPayloadChanged(int value);
    void onTimeScaleChanged(int index);
    void onExaggerationChanged(double value);
    void onExponentChanged(double value);
    void onFollowToggled(bool on);
    void onHazardLabelsToggled(bool on);

    // hazards
    void onHazardKindChanged(int index);
    void undoHazard();
    void clearHazards();
    void restoreHazards();

private:
    // ---- UI ----
    void setupUi();
    void setupMenus();
    QWidget* createDrivePanel();
    QWidget* createLoadsPanel();
    QWidget* createAnalysisPanel();
    void populateTree();
    void refreshSectionCombo();
    void refreshPayloadControls();
    void refreshWheelWidgets();

    // ---- scene ----
    void setupVtk();
    std::map<int, PidActorManager::PidSpec> createSceneSpecs();
    void buildVehicleMeshes();
    void applyVehicleMeshes();
    void applyMapMeshes();
    void createAnalysisSpecs(std::map<int, PidActorManager::PidSpec>& specs);
    void createOverlays();
    void rebuildZoneSigns();
    void rebuildHazardOverlays();

    static void onLeftButtonPress(vtkObject* caller, unsigned long eid, void* clientData, void* callData);
    void handleLeftButtonPress();
    void selectTreeItemByPid(int pid);

    // front camera view (top-right inset viewport)
    void setupFrontCamera();
    void updateFrontCamera();

    // mini map (bottom-left inset viewport)
    void setupMiniMap();
    void rebuildMiniMap();
    void updateMiniMap();

    // ---- map / vehicle / hazards ----
    void loadMap(MapType type);
    void changeVehicle(VehicleType type);
    bool placeHazard(double s, double pointD);
    void onHazardsChanged(const QString& message);
    std::string hazardResultText(const Hazard& h, const HazardEvent& ev) const;
    void setManualSpeed(int kmh, bool switchToManual);

    // ---- simulation ----
    void resetSimulation();
    void simulateFrame(double frameDt);
    double targetSpeed() const;
    void handleFinishedEvent(const HazardEvent& ev);
    bool courseFinished() const;
    void recordRun();

    // ---- visual update ----
    void updateVisuals();
    void updateTireGeometry(const VehicleModel::WheelArray& zuVis);
    void updateSuspensionGeometry(const VehicleModel::WheelArray& zuVis);
    void updateArrowGeometry();
    void resetTrail();
    void appendTrailStation(double initialValue);
    void updateTrailSubstep();
    void updateHazardLabels();
    void updateHud();
    void updateTables();
    void updateFollowCamera();
    void setCameraPreset(int preset);
    void showAlert(const std::string& text, const std::array<double, 3>& color, int durationMs);
    void renderAll();

private:
    // ---- UI ----
    QVTKOpenGLNativeWidget* m_vtkWidget = nullptr;
    QTreeWidget* m_treeWidget = nullptr;
    QTabWidget* m_sideTabs = nullptr;
    QComboBox* m_mapCombo = nullptr;
    QComboBox* m_vehicleCombo = nullptr;
    QComboBox* m_startCombo = nullptr;
    QSlider* m_speedSlider = nullptr;
    QLabel* m_speedLabel = nullptr;
    QSpinBox* m_speedSpin = nullptr;
    QSlider* m_payloadSlider = nullptr;
    QLabel* m_payloadTitle = nullptr;
    QLabel* m_payloadLabel = nullptr;
    QComboBox* m_timeScaleCombo = nullptr;
    QDoubleSpinBox* m_exaggerationSpin = nullptr;
    QDoubleSpinBox* m_exponentSpin = nullptr;
    QComboBox* m_hazardKindCombo = nullptr;
    QComboBox* m_hazardLaneCombo = nullptr;
    QSpinBox* m_hazardDistanceSpin = nullptr;
    QLabel* m_liveLabel = nullptr;
    QTableWidget* m_wheelTable = nullptr;
    QLabel* m_summaryLabel = nullptr;
    QTableWidget* m_runTable = nullptr;
    QTableWidget* m_eventTable = nullptr;
    LoadPlotWidget* m_plot = nullptr;

    QAction* m_followAction = nullptr;
    QAction* m_labelsAction = nullptr;
    QAction* m_autoSpeedAction = nullptr;
    QAction* m_placeAheadAction = nullptr;
    QAction* m_placeClickAction = nullptr;
    QActionGroup* m_mapGroup = nullptr;
    QActionGroup* m_vehicleGroup = nullptr;
    QActionGroup* m_hazardKindGroup = nullptr;

    // ---- VTK ----
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;

    vtkSmartPointer<vtkCellPicker> m_cellPicker;
    vtkSmartPointer<vtkCallbackCommand> m_leftButtonPressCallback;

    PidActorManager* m_pidActorManager = nullptr;

    vtkSmartPointer<vtkLookupTable> m_loadLut;    // load / static load
    vtkSmartPointer<vtkLookupTable> m_damageLut;  // log10(pseudo-damage)

    vtkSmartPointer<vtkTextActor> m_hudActor;
    vtkSmartPointer<vtkTextActor> m_alertActor;
    vtkSmartPointer<vtkScalarBarActor> m_loadBar;
    vtkSmartPointer<vtkScalarBarActor> m_damageBar;
    std::map<int, vtkSmartPointer<vtkBillboardTextActor3D>> m_hazardLabels;  // by hazard id
    std::vector<vtkSmartPointer<vtkBillboardTextActor3D>> m_zoneSigns;
    QElapsedTimer m_clock;
    qint64 m_alertUntilMs = 0;

    // vehicle transforms
    vtkSmartPointer<vtkMatrix4x4> m_yawMatrix;       // world <- vehicle frame (position + heading)
    vtkSmartPointer<vtkMatrix4x4> m_bodyMatrix;      // world <- body frame (exaggerated heave/pitch/roll)
    vtkSmartPointer<vtkMatrix4x4> m_bodyRelMatrix;   // vehicle frame <- body frame (exaggerated)
    vtkSmartPointer<vtkMatrix4x4> m_bodyTrueMatrix;  // world <- body frame (true motion, for the dash cam)
    vtkSmartPointer<vtkMatrix4x4> m_prevYawMatrix;   // for the follow camera

    // front camera view
    vtkSmartPointer<vtkRenderer> m_frontCamRenderer;
    vtkSmartPointer<vtkLight> m_frontCamHeadlight;
    std::map<int, vtkSmartPointer<vtkActor>> m_frontCamActors;  // camera-only copies (no edges)
    double m_frontCamViewAngle = 60.0;  // vertical FOV [deg]

    // mini map
    vtkSmartPointer<vtkRenderer> m_miniRenderer;
    std::map<int, vtkSmartPointer<vtkActor>> m_miniActors;
    vtkSmartPointer<vtkActor> m_miniVehicle;
    vtkSmartPointer<vtkPolyDataMapper> m_miniVehicleMapper;
    vtkSmartPointer<vtkPolyData> m_miniHazards;
    vtkSmartPointer<vtkPolyDataMapper> m_miniHazardMapper;
    std::map<int, vtkIdType> m_miniHazardCells;  // hazard id -> first cell

    // dynamic vehicle geometry
    vtkSmartPointer<vtkUnstructuredGrid> m_cabGrid;
    vtkSmartPointer<vtkUnstructuredGrid> m_bodyGrid;
    vtkSmartPointer<vtkUnstructuredGrid> m_cameraGrid;
    vtkSmartPointer<vtkUnstructuredGrid> m_frameGrid;
    vtkSmartPointer<vtkPolyData> m_tirePoly;
    std::vector<Vec3> m_tireBase;          // tyre points relative to their wheel centre
    std::vector<int> m_tirePointWheel;
    std::vector<int> m_tireCellWheel;
    vtkSmartPointer<vtkPolyData> m_suspPoly;
    std::vector<int> m_suspCellOwner;      // < wheels: spring of wheel i, otherwise axle beam
    vtkSmartPointer<vtkPolyData> m_arrowPoly;
    double m_springTopZ = 0.95;            // body-frame height of the spring seats
    double m_arrowScale = 1.3;

    vtkSmartPointer<vtkPolyData> m_trailPoly;
    vtkSmartPointer<vtkPoints> m_trailPoints;
    vtkSmartPointer<vtkCellArray> m_trailPolys;
    vtkSmartPointer<vtkDoubleArray> m_trailScalars;
    double m_trailS0 = 0.0;
    int m_trailStations = 0;
    double m_trailOffset = 0.9;
    double m_trailHalfWidth = 0.22;

    // ---- simulation ----
    RoadNetwork m_road;
    VehicleModel m_vehicle;
    LoadAnalyzer m_analyzer;

    QTimer* m_motionTimer = nullptr;
    double m_motionDt = 0.02;   // 20 ms timer interval
    QElapsedTimer m_frameClock; // measures the real frame time
    double m_customStartS = -1.0;  // >= 0: start position from the command line [m]

    double m_s = 0.0;           // front axle position along the course [m]
    double m_v = 0.0;           // [m/s]
    double m_accel = 0.0;       // [m/s^2]
    double m_simTime = 0.0;     // [s]
    VehicleModel::WheelArray m_wheelSpin{};

    int m_startSection = 0;
    bool m_autoSpeed = true;
    int m_manualSpeedKmh = 60;
    double m_timeScale = 1.0;
    double m_exaggeration = 3.0;
    bool m_followCamera = true;
    bool m_showLabels = true;
    int m_cameraPreset = 0;
    unsigned m_plotDecimator = 0;
    unsigned m_frameCounter = 0;
    bool m_runRecorded = false;
    std::vector<int> m_userHazards;  // ids of placed hazards (for undo)
};
