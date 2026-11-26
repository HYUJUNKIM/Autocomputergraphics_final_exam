#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

#include <vtkSmartPointer.h>

class vtkRenderer;
class vtkDataSet;
class vtkActor;
class vtkDataSetMapper;
class vtkMatrix4x4;
class vtkScalarsToColors;

class PidActorManager
{
public:
    // 셀 색상 결정 방식
    enum class ColorMode
    {
        Solid,      // 단색 (spec.color)
        DirectRgb,  // 셀 배열(uchar RGB)을 그대로 색으로 사용 (도로, 차선, 표지판 등)
        Mapped      // 셀 스칼라 배열을 LUT로 매핑 (하중비, 손상도 등)
    };

    // PID 하나(부품 / 맵 레이어 / 해석 결과)의 정의
    struct PidSpec
    {
        std::string name;
        std::string group = "Model";           // tree group: Vehicle / Map / Analysis
        vtkSmartPointer<vtkDataSet> data;      // vtkUnstructuredGrid 또는 vtkPolyData

        ColorMode colorMode = ColorMode::Solid;
        std::array<double, 3> color{ 0.8, 0.8, 0.8 };
        std::string arrayName;                 // DirectRgb / Mapped 에서 사용할 셀 배열 이름
        vtkSmartPointer<vtkScalarsToColors> lut;
        std::array<double, 2> range{ 0.0, 1.0 };

        bool showEdges = false;                // Surface 모드에서 메쉬 엣지 표시
        bool inFrontCamera = true;             // 전방 카메라 뷰에 포함
        bool inMiniMap = false;                // 미니맵에 포함
    };

    struct PidEntry
    {
        int pid = -1;
        PidSpec spec;

        vtkSmartPointer<vtkDataSetMapper> mapper;
        vtkSmartPointer<vtkActor> actor;

        vtkSmartPointer<vtkDataSetMapper> highlightMapper;
        vtkSmartPointer<vtkActor> highlightActor;

        bool visible = true;
    };

public:
    void buildFromSpecs(vtkRenderer* renderer, const std::map<int, PidSpec>& specs);

    void setVisibility(int pid, bool visible);
    bool isVisible(int pid) const;

    void setSurfaceMode();
    void setWireframeMode();

    void setSelectedPid(int pid);
    void clearSelection();

    int getSelectedPid() const;

    std::vector<int> getPidList() const;

    int findPidByActor(vtkActor* actor) const;

    // for animation
    vtkActor* getActorByPid(int pid) const;
    vtkActor* getHighlightActorByPid(int pid) const;

    const PidSpec* getSpec(int pid) const;

    // 차량처럼 움직이는 PID: actor와 highlight actor에 같은 변환행렬 적용
    void setUserMatrix(int pid, vtkMatrix4x4* matrix);

    // 맵 / 차종 변경 시 PID의 형상만 교체 (actor, 색상 설정, 가시성은 유지)
    void setData(int pid, vtkDataSet* data);

    // apply the colouring of a spec to any mapper (used for camera / mini-map copies too)
    static void configureMapper(vtkDataSetMapper* mapper, const PidSpec& spec);

private:
    std::map<int, PidEntry> m_pidEntries;
    int m_selectedPid = -1;
};
