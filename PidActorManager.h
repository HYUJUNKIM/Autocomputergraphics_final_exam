#pragma once

#include <map>
#include <vector>

#include <vtkSmartPointer.h>

class vtkRenderer;
class vtkUnstructuredGrid;
class vtkActor;
class vtkDataSetMapper;

class PidActorManager
{
public:
    struct PidEntry
    {
        int pid = -1;

        vtkSmartPointer<vtkUnstructuredGrid> grid;

        vtkSmartPointer<vtkDataSetMapper> mapper;
        vtkSmartPointer<vtkActor> actor;

        vtkSmartPointer<vtkDataSetMapper> highlightMapper;
        vtkSmartPointer<vtkActor> highlightActor;

        bool visible = true;
    };

public:
    void buildFromPidGrids(
        vtkRenderer* renderer,
        const std::map<int, vtkSmartPointer<vtkUnstructuredGrid>>& pidGrids);

    void setVisibility(int pid, bool visible);
    bool isVisible(int pid) const;

    void setSurfaceMode();
    void setWireframeMode();

    void setSelectedPid(int pid);
    void clearSelection();

    int getSelectedPid() const;

    std::vector<int> getPidList() const;
	
	int findPidByActor(vtkActor* actor) const;

	// 추가 for animation
    vtkActor* getActorByPid(int pid) const;
    vtkActor* getHighlightActorByPid(int pid) const;

private:
    std::map<int, PidEntry> m_pidEntries;
    int m_selectedPid = -1;
};