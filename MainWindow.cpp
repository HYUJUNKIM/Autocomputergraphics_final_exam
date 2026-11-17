#include "MainWindow.h"
#include "PidActorManager.h"

#include <array>
#include <cmath>
#include <vector>

#include <QAction>
#include <QDockWidget>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QTimer>  // for animation

#include <QVTKOpenGLNativeWidget.h>

#include <vtkCamera.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkHexahedron.h>
#include <vtkLight.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkUnstructuredGrid.h>

#include <vtkObject.h>
#include <vtkActor.h>

#include <vtkCallbackCommand.h>
#include <vtkCellPicker.h>
#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>

// for creating cylinder as tire
#include <vtkAppendFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkCylinderSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPolyData.h>

// for creating plane as ground
#include <vtkPlaneSource.h>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_pidActorManager(new PidActorManager())
{
    setupUi();
    setupVtk();

    statusBar()->showMessage("Drive-vehicle");
}

MainWindow::~MainWindow()
{
    delete m_pidActorManager;
}

void MainWindow::setupUi()
{
    setWindowTitle("Drive-vehicle");
    resize(1300, 850);

    m_vtkWidget = new QVTKOpenGLNativeWidget(this);
#ifdef Q_OS_MACOS
    // macOS trackpad delivers raw touches; VTK maps touch-down to LeftButtonPress,
    // so merely moving the cursor rotated the camera. Use normal mouse events instead.
    m_vtkWidget->setEnableTouchEventProcessing(false);
#endif
    setCentralWidget(m_vtkWidget);

    QMenu* fileMenu = menuBar()->addMenu("File");
    QAction* exitAction = new QAction("Exit", this);
    connect(exitAction, &QAction::triggered, this, &QMainWindow::close);
    fileMenu->addAction(exitAction);

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

    QToolBar* toolbar = addToolBar("View");
    toolbar->addAction(surfaceAction);
    toolbar->addAction(wireframeAction);
    toolbar->addSeparator();
    toolbar->addAction(startAction);
    toolbar->addAction(stopAction);
    toolbar->addAction(resetAction);

    QDockWidget* modelDock = new QDockWidget("PID / Parts", this);
    m_treeWidget = new QTreeWidget(modelDock);
    m_treeWidget->setHeaderLabel("Model");
    modelDock->setWidget(m_treeWidget);
    addDockWidget(Qt::LeftDockWidgetArea, modelDock);

    connect(m_treeWidget, &QTreeWidget::itemChanged,
            this, &MainWindow::onTreeItemChanged);
    
    connect(m_treeWidget, &QTreeWidget::currentItemChanged,
            this, &MainWindow::onCurrentTreeItemChanged);
}

void MainWindow::setupVtk()
{
    m_renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_renderer = vtkSmartPointer<vtkRenderer>::New();

    m_vtkWidget->setRenderWindow(m_renderWindow);
    m_renderWindow->AddRenderer(m_renderer);

    auto pidGrids = createSamplePidMeshes();
    m_pidActorManager->buildFromPidGrids(m_renderer, pidGrids);

    m_renderer->SetBackground(0.15, 0.18, 0.22);
    m_renderer->ResetCamera();

    for (int pid : m_pidActorManager->getPidList())
    {
        auto* item = new QTreeWidgetItem(m_treeWidget);
        item->setText(0, QString("PID %1").arg(pid));
        item->setCheckState(0, Qt::Checked);
        item->setData(0, Qt::UserRole, pid);
    }

    m_treeWidget->expandAll();
	
	m_cellPicker = vtkSmartPointer<vtkCellPicker>::New();
    m_cellPicker->SetTolerance(0.0005);

    m_leftButtonPressCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    m_leftButtonPressCallback->SetClientData(this);
    m_leftButtonPressCallback->SetCallback(MainWindow::onLeftButtonPress);

    m_renderWindow->GetInteractor()->AddObserver(
        vtkCommand::LeftButtonPressEvent,
        m_leftButtonPressCallback);
	
    // animation timer
    m_motionTimer = new QTimer(this);
    connect(m_motionTimer, &QTimer::timeout,
        this, &MainWindow::updateMotion);

    // PID 1,2,3,4 which represent a vehicle, initial positions
    for (int pid : {1, 2, 3, 4})
    {
        vtkActor* actor = m_pidActorManager->getActorByPid(pid);
        if (actor)
        {
            double p[3];
            actor->GetPosition(p);
            m_initialPidPositions[pid] = { p[0], p[1], p[2] };
        }
    }

    setupFrontCamera();

    statusBar()->showMessage(
        QString("Ready | velocity = (%1, 0, 0) m/s")
        .arg(m_motionVx));

    m_renderWindow->Render();
}

void MainWindow::setupFrontCamera()
{
    m_frontCamRenderer = vtkSmartPointer<vtkRenderer>::New();

    // top-right inset: 20% of width and height (normalized viewport coords)
    m_frontCamRenderer->SetViewport(0.8, 0.8, 1.0, 1.0);
    m_frontCamRenderer->SetBackground(0.05, 0.05, 0.08);

    // draw on top of the main renderer, but still clear its own background
    m_renderWindow->SetNumberOfLayers(2);
    m_frontCamRenderer->SetLayer(1);
    m_frontCamRenderer->PreserveColorBufferOff();

    // mouse interaction / picking stays on the main renderer
    m_frontCamRenderer->InteractiveOff();

    // camera-only actors: share the mapper (geometry), but use their own property without
    // mesh edges. Surface-with-edges breaks on cells with vertices behind the camera
    // (the road cell under the vehicle turned black). Position/visibility are synced in
    // updateFrontCamera(). No highlight actors: the camera sees the real scene only.
    m_frontCamActors.clear();
    for (int pid : m_pidActorManager->getPidList())
    {
        vtkActor* src = m_pidActorManager->getActorByPid(pid);
        if (!src)
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
    cam->SetClippingRange(0.02, 200.0);

    // headlight: positional spotlight at the lens, aimed at the camera focal point.
    // (the default directional headlight runs parallel to the road, so the road stays dark)
    m_frontCamHeadlight = vtkSmartPointer<vtkLight>::New();
    m_frontCamHeadlight->SetLightTypeToSceneLight();
    m_frontCamHeadlight->PositionalOn();
    m_frontCamHeadlight->SetConeAngle(50.0);      // half angle [deg]
    m_frontCamHeadlight->SetExponent(2.0);        // soft falloff toward the cone edge
    m_frontCamHeadlight->SetAttenuationValues(1.0, 0.0, 0.0);  // no distance falloff
    m_frontCamHeadlight->SetColor(1.0, 0.97, 0.9);
    m_frontCamHeadlight->SetIntensity(1.2);

    m_frontCamRenderer->AutomaticLightCreationOff();
    m_frontCamRenderer->AddLight(m_frontCamHeadlight);

    m_renderWindow->AddRenderer(m_frontCamRenderer);

    updateFrontCamera();
}

void MainWindow::updateFrontCamera()
{
    if (!m_frontCamRenderer)
        return;

    for (auto& [pid, camActor] : m_frontCamActors)
    {
        if (vtkActor* src = m_pidActorManager->getActorByPid(pid))
        {
            camActor->SetPosition(src->GetPosition());
            camActor->SetVisibility(src->GetVisibility());
        }
    }

    // follow the translation of PID 4 (camera module)
    double offset[3] = { 0.0, 0.0, 0.0 };
    if (vtkActor* camModule = m_pidActorManager->getActorByPid(4))
    {
        camModule->GetPosition(offset);
    }

    const double pos[3] = {
        m_frontCamMount[0] + offset[0],
        m_frontCamMount[1] + offset[1],
        m_frontCamMount[2] + offset[2]
    };

    // look toward -x, pitched down toward the road
    const double pitch = vtkMath::RadiansFromDegrees(m_frontCamPitch);
    const double dir[3] = { -std::cos(pitch), 0.0, -std::sin(pitch) };
    const double up[3]  = { -std::sin(pitch), 0.0,  std::cos(pitch) };  // orthogonal to dir

    vtkCamera* cam = m_frontCamRenderer->GetActiveCamera();
    cam->SetPosition(pos[0], pos[1], pos[2]);
    cam->SetFocalPoint(pos[0] + dir[0], pos[1] + dir[1], pos[2] + dir[2]);
    cam->SetViewUp(up[0], up[1], up[2]);
    cam->SetClippingRange(0.02, 200.0);

    // headlight shares the camera axis, so it is pitched by the same angle
    if (m_frontCamHeadlight)
    {
        double fp[3];
        cam->GetFocalPoint(fp);
        m_frontCamHeadlight->SetPosition(pos[0], pos[1], pos[2]);
        m_frontCamHeadlight->SetFocalPoint(fp[0], fp[1], fp[2]);
    }
}

vtkSmartPointer<vtkUnstructuredGrid> MainWindow::createHexGrid(
    const std::vector<std::array<double, 3>>& pts,
    double scalarValue)
{
    vtkNew<vtkPoints> points;
    for (const auto& p : pts)
    {
        points->InsertNextPoint(p[0], p[1], p[2]);
    }

    vtkNew<vtkHexahedron> hex;
    for (int i = 0; i < 8; ++i)
    {
        hex->GetPointIds()->SetId(i, i);
    }

    vtkNew<vtkUnstructuredGrid> grid;
    grid->SetPoints(points);
    grid->InsertNextCell(hex->GetCellType(), hex->GetPointIds());

    vtkNew<vtkDoubleArray> cellScalars;
    cellScalars->SetName("ElementScalar");
    cellScalars->SetNumberOfComponents(1);
    cellScalars->InsertNextValue(scalarValue);
    grid->GetCellData()->SetScalars(cellScalars);

    return grid;
}

vtkSmartPointer<vtkUnstructuredGrid> MainWindow::createCylinderGroupGrid(
    const std::vector<std::array<double, 3>>& centers,
    double radius,
    double height,
    double scalarValue,
    char  axis)
{
    vtkNew<vtkAppendPolyData> appendPolyData;

    for (const auto& c : centers)
    {
        vtkNew<vtkCylinderSource> cylinder;
        cylinder->SetRadius(radius);
        cylinder->SetHeight(height);
        cylinder->SetResolution(16);
        cylinder->CappingOn();
        cylinder->Update();

        
        vtkNew<vtkTransform> transform;

        transform->Translate(c[0], c[1], c[2]);
        
        // vtkCylinderSource center position (0,0,0), axis direction Y, (h/2, -h/2) height along Y

        // vtkCylinderSource axis direction Y
        if (axis == 'Z' || axis == 'z')
        {
            transform->RotateX(90.0);   // Y -> Z
        }
        else if (axis == 'X' || axis == 'x')
        {
            transform->RotateZ(-90.0);   // Y -> X
        }
        // If Y axis, no rotation needed
 
        vtkNew<vtkTransformPolyDataFilter> transformFilter;
        transformFilter->SetInputConnection(cylinder->GetOutputPort());
        transformFilter->SetTransform(transform);
        transformFilter->Update();

        appendPolyData->AddInputData(transformFilter->GetOutput());
    }

    appendPolyData->Update();

    // PolyData -> UnstructuredGrid 
    vtkNew<vtkAppendFilter> appendFilter;
    appendFilter->AddInputData(appendPolyData->GetOutput());
    appendFilter->Update();

    vtkSmartPointer<vtkUnstructuredGrid> grid =
        vtkSmartPointer<vtkUnstructuredGrid>::New();
    grid->ShallowCopy(appendFilter->GetOutput());

    // Assign scalar value to all cells
    vtkIdType numCells = grid->GetNumberOfCells();

    vtkNew<vtkDoubleArray> cellScalars;
    cellScalars->SetName("ElementScalar");
    cellScalars->SetNumberOfComponents(1);
    cellScalars->SetNumberOfTuples(numCells);

    for (vtkIdType i = 0; i < numCells; ++i)
    {
        cellScalars->SetValue(i, scalarValue);
    }

    grid->GetCellData()->SetScalars(cellScalars);

    return grid;
}

vtkSmartPointer<vtkUnstructuredGrid> MainWindow::createPlaneGrid(
    const std::array<double, 3>& origin,
    const std::array<double, 3>& point1,
    const std::array<double, 3>& point2,
    double scalarValue)
{
    vtkNew<vtkPlaneSource> plane;
    plane->SetOrigin(origin[0], origin[1], origin[2]);
    plane->SetPoint1(point1[0], point1[1], point1[2]);
    plane->SetPoint2(point2[0], point2[1], point2[2]);
    plane->SetXResolution(10);
    plane->SetYResolution(3);
    plane->Update();

    // PolyData -> UnstructuredGrid 변??
    vtkNew<vtkAppendFilter> appendFilter;
    appendFilter->AddInputData(plane->GetOutput());
    appendFilter->Update();

    vtkSmartPointer<vtkUnstructuredGrid> grid =
        vtkSmartPointer<vtkUnstructuredGrid>::New();
    grid->ShallowCopy(appendFilter->GetOutput());

    vtkIdType numCells = grid->GetNumberOfCells();

    vtkNew<vtkDoubleArray> cellScalars;
    cellScalars->SetName("ElementScalar");
    cellScalars->SetNumberOfComponents(1);
    cellScalars->SetNumberOfTuples(numCells);

    for (vtkIdType i = 0; i < numCells; ++i)
    {
        cellScalars->SetValue(i, scalarValue);
    }

    grid->GetCellData()->SetScalars(cellScalars);

    return grid;
}

std::map<int, vtkSmartPointer<vtkUnstructuredGrid>> MainWindow::createSamplePidMeshes()
{
    std::map<int, vtkSmartPointer<vtkUnstructuredGrid>> pidGrids;

    // PID 1
    std::vector<std::array<double, 3>> pts1 = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 1.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0},
        {1.0, 0.0, 1.0},
        {1.0, 1.0, 1.0},
        {0.0, 1.0, 1.0}
    };

    // PID 2
    std::vector<std::array<double, 3>> pts2 = {
        {1.0, 0.0, 0.0},
        {4.0, 0.0, 0.0},
        {4.0, 1.0, 0.0},
        {1.0, 1.0, 0.0},
        {1.0, 0.0, 1.0},
        {4.0, 0.0, 1.0},
        {4.0, 1.0, 1.0},
        {1.0, 1.0, 1.0}
    };

    pidGrids[1] = createHexGrid(pts1, 10.0);
    pidGrids[2] = createHexGrid(pts2, 15.0);

    // PID 3 : cylinder group for Tires
    std::vector<std::array<double, 3>> tireCylinderCenters = {
        {0.5,  0.0, 0.1},
        {0.5,  1.0, 0.1},
        {2.5,  0.0, 0.1},
        {2.5,  1.0, 0.1},
        {3.5,  0.0, 0.1},
        {3.5,  1.0, 0.1}
    };

    pidGrids[3] = createCylinderGroupGrid(
        tireCylinderCenters,
        0.2,   // radius
        0.1,   // height (Y direction)
        15.0,   // gray similar color value as scalar
		'Y'     // axis (cylinder axis direction, Y)
    );
    // PID 4 : cylinder group for Front Camera
    std::vector<std::array<double, 3>> frontCameraCylinderCenters = {
        {0.1,  0.5, 1.1}
    };

    pidGrids[4] = createCylinderGroupGrid(
        frontCameraCylinderCenters,
        0.1,   // radius
        0.3,   // height (Y direction)
        14.0,   // gray similar color value as scalar
		'X'     // axis (cylinder axis direction, Y)
    );
    // PID 5 : cylinder group for columns
    std::vector<std::array<double, 3>> columnCylinderCenters = {
        { 10.0,  -0.5, 0.9},
        { 10.0,   1.5, 0.9},
        {  0.0,  -0.5, 0.9},
        {  0.0,   1.5, 0.9},
        {-10.0,  -0.5, 0.9},
        {-10.0,   1.5, 0.9},
        {-20.0,  -0.5, 0.9},
        {-20.0,   1.5, 0.9},
        {-30.0,  -0.5, 0.9},
        {-30.0,   1.5, 0.9}
    };

    pidGrids[5] = createCylinderGroupGrid(
        columnCylinderCenters,
        0.1,   // radius
        2.0,   // height (Y direction)
        12.0,   // color scalar
		'Z'     // axis (cylinder axis direction, Z)
    );

	// PID 6  : plane for ground
    pidGrids[6] = createPlaneGrid(
        { 10.0, -0.6, -0.1 },   // origin
        { -35.0, -0.6, -0.1 },   // point1
        { 10.0,  1.6, -0.1 },   // point2
        16.                 // scalar value
    );



    return pidGrids;
}

void MainWindow::setSurfaceMode()
{
    m_pidActorManager->setSurfaceMode();
    m_renderWindow->Render();
    statusBar()->showMessage("Surface mode");
}

void MainWindow::setWireframeMode()
{
    m_pidActorManager->setWireframeMode();
    m_renderWindow->Render();
    statusBar()->showMessage("Wireframe mode");
}

void MainWindow::onTreeItemChanged(QTreeWidgetItem* item, int column)
{
    Q_UNUSED(column);

    if (!item)
        return;

    int pid = item->data(0, Qt::UserRole).toInt();
    bool visible = (item->checkState(0) == Qt::Checked);

    m_pidActorManager->setVisibility(pid, visible);
    updateFrontCamera();
    m_renderWindow->Render();

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

    if (!pickedActor || pickedCellId < 0)
    {
        m_pidActorManager->clearSelection();
        m_renderWindow->Render();
        statusBar()->showMessage("No cell picked");
        return;
    }

    int pid = m_pidActorManager->findPidByActor(pickedActor);
    if (pid < 0)
    {
        m_pidActorManager->clearSelection();
        m_renderWindow->Render();
        statusBar()->showMessage("Picked actor is not managed by PID manager");
        return;
    }

    m_pidActorManager->setSelectedPid(pid);
    selectTreeItemByPid(pid);

    m_renderWindow->Render();

    statusBar()->showMessage(
        QString("Picked PID %1, local cell id = %2")
            .arg(pid)
            .arg(static_cast<long long>(pickedCellId)));
}

void MainWindow::selectTreeItemByPid(int pid)
{
    if (!m_treeWidget)
        return;

    for (int i = 0; i < m_treeWidget->topLevelItemCount(); ++i)
    {
        QTreeWidgetItem* item = m_treeWidget->topLevelItem(i);
        if (!item)
            continue;

        int itemPid = item->data(0, Qt::UserRole).toInt();
        if (itemPid == pid)
        {
            m_treeWidget->setCurrentItem(item);
            return;
        }
    }
}

void MainWindow::onCurrentTreeItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous)
{
    Q_UNUSED(previous);

    if (!current)
    {
        m_pidActorManager->clearSelection();
        m_renderWindow->Render();
        statusBar()->showMessage("Selection cleared");
        return;
    }

    int pid = current->data(0, Qt::UserRole).toInt();
    m_pidActorManager->setSelectedPid(pid);
    m_renderWindow->Render();

    statusBar()->showMessage(QString("PID %1 selected").arg(pid));
}

void MainWindow::startMotion()
{
    if (!m_motionTimer)
        return;

    if (!m_motionTimer->isActive())
    {
        // Start the timer with the configured time step in milliseconds
        m_motionTimer->start(static_cast<int>(m_motionDt * 1000.0));

        statusBar()->showMessage(
            QString("Motion started | velocity = (%1, 0, 0) m/s")
            .arg(m_motionVx));
    }
}

void MainWindow::stopMotion()
{
    if (!m_motionTimer)
        return;

    if (m_motionTimer->isActive())
    {
        m_motionTimer->stop();

        statusBar()->showMessage(
            QString("Motion stopped | velocity = (%1, 0, 0) m/s")
            .arg(m_motionVx));
    }
}

void MainWindow::resetMotion()
{
    if (m_motionTimer && m_motionTimer->isActive())
    {
        m_motionTimer->stop();
    }

    for (int pid : {1, 2, 3, 4})
    {
        auto it = m_initialPidPositions.find(pid);
        if (it == m_initialPidPositions.end())
            continue;

        vtkActor* actor = m_pidActorManager->getActorByPid(pid);
        if (actor)
        {
            actor->SetPosition(it->second[0], it->second[1], it->second[2]);
        }

        vtkActor* highlightActor = m_pidActorManager->getHighlightActorByPid(pid);
        if (highlightActor)
        {
            highlightActor->SetPosition(it->second[0], it->second[1], it->second[2]);
        }
    }

    updateFrontCamera();
    m_renderWindow->Render();

    statusBar()->showMessage(
        QString("Motion reset | velocity = (%1, 0, 0) m/s")
        .arg(m_motionVx));
}

void MainWindow::updateMotion()
{
    const double dx = m_motionVx * m_motionDt;

    for (int pid : {1, 2, 3, 4})
    {
        vtkActor* actor = m_pidActorManager->getActorByPid(pid);
        if (actor)
        {
            actor->AddPosition(dx, 0.0, 0.0);
        }

        vtkActor* highlightActor = m_pidActorManager->getHighlightActorByPid(pid);
        if (highlightActor)
        {
            highlightActor->AddPosition(dx, 0.0, 0.0);
        }
    }

    updateFrontCamera();
    m_renderWindow->Render();

    statusBar()->showMessage(
        QString("Moving PID 1,2,3,4 | velocity = (%1, 0, 0) m/s")
        .arg(m_motionVx));
}