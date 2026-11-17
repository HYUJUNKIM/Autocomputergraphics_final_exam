#pragma once

#include <QMainWindow>
#include <QTimer>    // for animation

#include <map>

#include <vtkSmartPointer.h>

class QTreeWidget;
class QTreeWidgetItem;
class QVTKOpenGLNativeWidget;
class vtkGenericOpenGLRenderWindow;
class vtkRenderer;
class vtkLight;
class vtkActor;
class vtkUnstructuredGrid;

class vtkObject;
class vtkCellPicker;
class vtkCallbackCommand;

class PidActorManager;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

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

private:
    void setupUi();
    void setupVtk();

    std::map<int, vtkSmartPointer<vtkUnstructuredGrid>> createSamplePidMeshes();
    vtkSmartPointer<vtkUnstructuredGrid> createHexGrid(
        const std::vector<std::array<double, 3>>& pts,
        double scalarValue);
    vtkSmartPointer<vtkUnstructuredGrid> createCylinderGroupGrid(
        const std::vector<std::array<double, 3>>& centers,
        double radius,
        double height,
        double scalarValue,
        char axis ='Y');
    vtkSmartPointer<vtkUnstructuredGrid> createPlaneGrid(
        const std::array<double, 3>& origin,
        const std::array<double, 3>& point1,
        const std::array<double, 3>& point2,
        double scalarValue);
	
	static void onLeftButtonPress(vtkObject* caller, unsigned long eid, void* clientData, void* callData);
    void handleLeftButtonPress();
    void selectTreeItemByPid(int pid);

    // front camera view (top-right inset viewport)
    void setupFrontCamera();
    void updateFrontCamera();

private:
    QVTKOpenGLNativeWidget* m_vtkWidget = nullptr;
    QTreeWidget* m_treeWidget = nullptr;

    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;
	
	vtkSmartPointer<vtkCellPicker> m_cellPicker;
    vtkSmartPointer<vtkCallbackCommand> m_leftButtonPressCallback;

    PidActorManager* m_pidActorManager = nullptr;

    // animation
    QTimer* m_motionTimer = nullptr;
    double m_motionDt = 0.02;   // 20 ms
    double m_motionVx = -10.0;  // -x direction, 10 m/s

    // Save initial position for reset
    std::map<int, std::array<double, 3>> m_initialPidPositions;

    // front camera view
    vtkSmartPointer<vtkRenderer> m_frontCamRenderer;
    vtkSmartPointer<vtkLight> m_frontCamHeadlight;
    std::map<int, vtkSmartPointer<vtkActor>> m_frontCamActors;  // camera-only copies (no edges)
    // lens position in model coordinates
    // (front face of PID 4 cylinder: x = 0.1 - 0.3/2 = -0.05, moved 0.15 further toward -x)
    std::array<double, 3> m_frontCamMount = { -0.20, 0.5, 1.1 };
    double m_frontCamViewAngle = 60.0;  // vertical FOV [deg]
    double m_frontCamPitch = 15.0;      // downward tilt toward the road [deg]
};