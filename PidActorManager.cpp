#include "PidActorManager.h"

#include <vtkActor.h>
#include <vtkDataSet.h>
#include <vtkDataSetMapper.h>
#include <vtkMatrix4x4.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkScalarsToColors.h>

void PidActorManager::configureMapper(vtkDataSetMapper* mapper, const PidSpec& spec)
{
    switch (spec.colorMode)
    {
    case ColorMode::Solid:
        mapper->ScalarVisibilityOff();
        break;

    case ColorMode::DirectRgb:
        // 셀 데이터의 RGB 배열을 그대로 색으로 사용
        mapper->ScalarVisibilityOn();
        mapper->SetScalarModeToUseCellFieldData();
        mapper->SelectColorArray(spec.arrayName.c_str());
        mapper->SetColorModeToDirectScalars();
        break;

    case ColorMode::Mapped:
        // 셀 스칼라를 LUT로 매핑
        mapper->ScalarVisibilityOn();
        mapper->SetScalarModeToUseCellFieldData();
        mapper->SelectColorArray(spec.arrayName.c_str());
        mapper->SetColorModeToMapScalars();
        if (spec.lut)
            mapper->SetLookupTable(spec.lut);
        mapper->SetScalarRange(spec.range[0], spec.range[1]);
        break;
    }
}

void PidActorManager::buildFromSpecs(
    vtkRenderer* renderer,
    const std::map<int, PidSpec>& specs)
{
    m_pidEntries.clear();
    m_selectedPid = -1;

    // specs의 각 원소(std::pair<const int, PidSpec>)를
    // 구조적 바인딩(C++17)으로 pid(first, 키)와 spec(second, 정의)으로 풀어서 순회한다.
    // std::map이므로 pid 오름차순(1, 2, 3, ...)으로 방문한다.
    for (const auto& [pid, spec] : specs)
    {
        PidEntry entry;
        entry.pid = pid;
        entry.spec = spec;
        entry.visible = true;

        // -----------------------------
        // 기본 표시 actor
        // -----------------------------
        entry.mapper = vtkSmartPointer<vtkDataSetMapper>::New();
        entry.actor = vtkSmartPointer<vtkActor>::New();

        entry.mapper->SetInputData(spec.data);
        configureMapper(entry.mapper, spec);

        entry.actor->SetMapper(entry.mapper);
        entry.actor->GetProperty()->SetColor(spec.color[0], spec.color[1], spec.color[2]);
        entry.actor->GetProperty()->SetRepresentationToSurface();
        entry.actor->GetProperty()->SetEdgeVisibility(spec.showEdges ? 1 : 0);
        entry.actor->GetProperty()->SetEdgeColor(0.08, 0.08, 0.10);

        renderer->AddActor(entry.actor);

        // -----------------------------
        // 하이라이트 actor
        // ParaView의 selection representation에 해당
        // -----------------------------
        entry.highlightMapper = vtkSmartPointer<vtkDataSetMapper>::New();
        entry.highlightActor  = vtkSmartPointer<vtkActor>::New();

        entry.highlightMapper->SetInputData(spec.data);
        entry.highlightMapper->ScalarVisibilityOff(); // 하이라이트는 단색 강조

        entry.highlightActor->SetMapper(entry.highlightMapper);
        entry.highlightActor->SetVisibility(0);
        entry.highlightActor->PickableOff();

        // 강조 속성: 노란색 wireframe overlay
        entry.highlightActor->GetProperty()->SetRepresentationToWireframe();
        entry.highlightActor->GetProperty()->SetColor(1.0, 1.0, 0.0);
        entry.highlightActor->GetProperty()->SetLineWidth(3.0);
        entry.highlightActor->GetProperty()->SetAmbient(1.0);
        entry.highlightActor->GetProperty()->SetDiffuse(0.0);
        entry.highlightActor->GetProperty()->SetEdgeVisibility(0);

        renderer->AddActor(entry.highlightActor);

        m_pidEntries[pid] = entry;
    }
}

void PidActorManager::setVisibility(int pid, bool visible)
{
    auto it = m_pidEntries.find(pid);
    if (it == m_pidEntries.end())
        return;

    it->second.visible = visible;

    if (it->second.actor)
    {
        it->second.actor->SetVisibility(visible ? 1 : 0);
    }

    // 선택된 PID인데 숨김 처리되면 하이라이트도 같이 꺼야 함
    if (it->second.highlightActor)
    {
        const bool highlightOn = visible && (pid == m_selectedPid);
        it->second.highlightActor->SetVisibility(highlightOn ? 1 : 0);
    }
}

bool PidActorManager::isVisible(int pid) const
{
    auto it = m_pidEntries.find(pid);
    if (it == m_pidEntries.end())
        return false;

    return it->second.visible;
}

void PidActorManager::setSurfaceMode()
{
    for (auto& [pid, entry] : m_pidEntries)
    {
        if (!entry.actor)
            continue;

        entry.actor->GetProperty()->SetRepresentationToSurface();
        entry.actor->GetProperty()->SetEdgeVisibility(entry.spec.showEdges ? 1 : 0);
    }
}

void PidActorManager::setWireframeMode()
{
    for (auto& [pid, entry] : m_pidEntries)
    {
        if (!entry.actor)
            continue;

        entry.actor->GetProperty()->SetRepresentationToWireframe();
        entry.actor->GetProperty()->SetEdgeVisibility(0);
    }
}

void PidActorManager::setSelectedPid(int pid)
{
    // 이전 선택 해제
    if (m_selectedPid != -1)
    {
        auto oldIt = m_pidEntries.find(m_selectedPid);
        if (oldIt != m_pidEntries.end() && oldIt->second.highlightActor)
        {
            oldIt->second.highlightActor->SetVisibility(0);
        }
    }

    m_selectedPid = pid;

    // 새 선택 표시
    auto it = m_pidEntries.find(m_selectedPid);
    if (it != m_pidEntries.end() && it->second.highlightActor)
    {
        const bool on = it->second.visible;
        it->second.highlightActor->SetVisibility(on ? 1 : 0);
    }
}

void PidActorManager::clearSelection()
{
    if (m_selectedPid != -1)
    {
        auto it = m_pidEntries.find(m_selectedPid);
        if (it != m_pidEntries.end() && it->second.highlightActor)
        {
            it->second.highlightActor->SetVisibility(0);
        }
    }
    m_selectedPid = -1;
}

std::vector<int> PidActorManager::getPidList() const
{
    std::vector<int> pids;
    pids.reserve(m_pidEntries.size());

    for (const auto& [pid, entry] : m_pidEntries)
    {
        pids.push_back(pid);
    }
    return pids;
}

int PidActorManager::getSelectedPid() const
{
    return m_selectedPid;
}

int PidActorManager::findPidByActor(vtkActor* actor) const
{
    for (const auto& [pid, entry] : m_pidEntries)
    {
        if (entry.actor == actor)
            return pid;
    }
    return -1;
}

vtkActor* PidActorManager::getActorByPid(int pid) const
{
    auto it = m_pidEntries.find(pid);
    if (it != m_pidEntries.end() && it->second.actor)
    {
        return it->second.actor.GetPointer();
    }
    return nullptr;
}

vtkActor* PidActorManager::getHighlightActorByPid(int pid) const
{
    auto it = m_pidEntries.find(pid);
    if (it != m_pidEntries.end() && it->second.highlightActor)
    {
        return it->second.highlightActor.GetPointer();
    }
    return nullptr;
}

const PidActorManager::PidSpec* PidActorManager::getSpec(int pid) const
{
    auto it = m_pidEntries.find(pid);
    return (it != m_pidEntries.end()) ? &it->second.spec : nullptr;
}

void PidActorManager::setData(int pid, vtkDataSet* data)
{
    auto it = m_pidEntries.find(pid);
    if (it == m_pidEntries.end() || !data)
        return;

    it->second.spec.data = data;
    if (it->second.mapper)
        it->second.mapper->SetInputData(data);
    if (it->second.highlightMapper)
        it->second.highlightMapper->SetInputData(data);
}

void PidActorManager::setUserMatrix(int pid, vtkMatrix4x4* matrix)
{
    auto it = m_pidEntries.find(pid);
    if (it == m_pidEntries.end())
        return;

    if (it->second.actor)
        it->second.actor->SetUserMatrix(matrix);
    if (it->second.highlightActor)
        it->second.highlightActor->SetUserMatrix(matrix);
}
