#include "PidActorManager.h"

#include <vtkActor.h>
#include <vtkCellData.h>
#include <vtkDataSetMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkUnstructuredGrid.h>

void PidActorManager::buildFromPidGrids(
    vtkRenderer* renderer,
    const std::map<int, vtkSmartPointer<vtkUnstructuredGrid>>& pidGrids)
{
    m_pidEntries.clear();
    m_selectedPid = -1;

    // pidGrids의 각 원소(std::pair<const int, vtkSmartPointer<vtkUnstructuredGrid>>)를
    // 구조적 바인딩(C++17)으로 pid(first, 키)와 grid(second, 메쉬)로 풀어서 순회한다.
    // std::map이므로 pid 오름차순(1, 2, 3, ...)으로 방문하며,
    // const auto& 로 받아 스마트 포인터 복사(참조 카운트 증감) 없이 읽기 전용으로 접근한다.
    for (const auto& [pid, grid] : pidGrids)
    {
        PidEntry entry;
        entry.pid = pid;
        entry.grid = grid;
        entry.visible = true;

        // -----------------------------
        // 기본 표시 actor
        // -----------------------------
        entry.mapper = vtkSmartPointer<vtkDataSetMapper>::New();
        entry.actor = vtkSmartPointer<vtkActor>::New();

        entry.mapper->SetInputData(grid);

        if (grid->GetCellData() && grid->GetCellData()->GetScalars())
        {
            entry.mapper->SetScalarModeToUseCellData();
            entry.mapper->ScalarVisibilityOn();
            entry.mapper->SetScalarRange(10.0, 25.0);
        }
        else
        {
            entry.mapper->ScalarVisibilityOff();
        }

        entry.actor->SetMapper(entry.mapper);
        entry.actor->GetProperty()->SetRepresentationToSurface();
        entry.actor->GetProperty()->SetEdgeVisibility(1);

        renderer->AddActor(entry.actor);

        // -----------------------------
        // 하이라이트 actor
        // ParaView의 selection representation에 해당
        // -----------------------------
        entry.highlightMapper = vtkSmartPointer<vtkDataSetMapper>::New();
        entry.highlightActor  = vtkSmartPointer<vtkActor>::New();

        entry.highlightMapper->SetInputData(grid);
        entry.highlightMapper->ScalarVisibilityOff(); // 하이라이트는 단색 강조

        entry.highlightActor->SetMapper(entry.highlightMapper);
        entry.highlightActor->SetVisibility(0);

        // 강조 속성: 노란색 wireframe overlay
        entry.highlightActor->GetProperty()->SetRepresentationToWireframe();
        entry.highlightActor->GetProperty()->SetColor(1.0, 1.0, 0.0);
        entry.highlightActor->GetProperty()->SetLineWidth(4.0);
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
        entry.actor->GetProperty()->SetEdgeVisibility(1);
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