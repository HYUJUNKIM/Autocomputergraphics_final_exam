#include "LoadAnalyzer.h"

#include "RoadNetwork.h"

#include <algorithm>
#include <cmath>

Severity severityOf(double peakLoadRatio)
{
    if (peakLoadRatio < 1.4)
        return Severity::Low;
    if (peakLoadRatio < 1.9)
        return Severity::Moderate;
    if (peakLoadRatio < 2.5)
        return Severity::High;
    return Severity::Severe;
}

const char* severityName(Severity s)
{
    switch (s)
    {
    case Severity::Low: return "LOW";
    case Severity::Moderate: return "MODERATE";
    case Severity::High: return "HIGH";
    case Severity::Severe: return "SEVERE";
    }
    return "";
}

void LoadAnalyzer::reset(const RoadNetwork& road, const VehicleModel& vehicle, double sStart)
{
    m_counters = {};
    m_damage.fill(0.0);
    m_peakRatio.fill(0.0);
    m_peakForce = 0.0;
    m_gate = std::max(20.0, 0.01 * vehicle.minStaticTireForce());

    m_ids.clear();
    m_state.clear();
    m_events.clear();
    m_finished.clear();
    m_finishedCount = 0;

    m_bgDamage = 0.0;
    m_bgDistance = 0.0;
    m_distance = 0.0;
    m_hazardDamage = 0.0;

    syncHazards(road, sStart);
}

void LoadAnalyzer::syncHazards(const RoadNetwork& road, double sFront)
{
    const auto& hazards = road.hazards();
    std::vector<int> ids(hazards.size());
    std::vector<int> state(hazards.size(), Pending);
    std::vector<HazardEvent> events(hazards.size());

    for (size_t k = 0; k < hazards.size(); ++k)
    {
        const Hazard& h = hazards[k];
        ids[k] = h.id;
        const auto it = std::find(m_ids.begin(), m_ids.end(), h.id);
        if (it != m_ids.end())
        {
            const size_t old = static_cast<size_t>(it - m_ids.begin());
            state[k] = m_state[old];
            events[k] = m_events[old];
        }
        else
        {
            events[k].hazardId = h.id;
            // hazards the front axle has already reached cannot be measured any more
            if (h.s - h.halfExtent() - 1.0 < sFront)
                state[k] = Skipped;
        }
    }

    m_ids.swap(ids);
    m_state.swap(state);
    m_events.swap(events);
}

double LoadAnalyzer::halfCycle(double range) const
{
    return 0.5 * std::pow(range / m_refRange, m_exponent);
}

double LoadAnalyzer::countReversal(Counter& c, double f) const
{
    if (!c.init)
    {
        c.init = true;
        c.lastTurn = f;
        c.extreme = f;
        c.dir = 0;
        return 0.0;
    }

    double inc = 0.0;
    if (c.dir == 0)
    {
        if (f > c.lastTurn + m_gate) { c.dir = 1; c.extreme = f; }
        else if (f < c.lastTurn - m_gate) { c.dir = -1; c.extreme = f; }
    }
    else if (c.dir > 0)
    {
        if (f > c.extreme)
            c.extreme = f;
        else if (f < c.extreme - m_gate)
        {
            inc = halfCycle(c.extreme - c.lastTurn);
            c.lastTurn = c.extreme;
            c.extreme = f;
            c.dir = -1;
        }
    }
    else
    {
        if (f < c.extreme)
            c.extreme = f;
        else if (f > c.extreme + m_gate)
        {
            inc = halfCycle(c.lastTurn - c.extreme);
            c.lastTurn = c.extreme;
            c.extreme = f;
            c.dir = 1;
        }
    }
    return inc;
}

void LoadAnalyzer::update(double t, double dt, double sFront, double speed,
                          const RoadNetwork& road, const VehicleModel& vehicle)
{
    const int n = vehicle.wheelCount();
    double increment = 0.0;
    double xFront = -1e9, xRear = 1e9;
    for (int i = 0; i < n; ++i)
    {
        const double f = vehicle.tireForce(i);
        const double inc = countReversal(m_counters[i], f);
        m_damage[i] += inc;
        increment += inc;
        m_peakRatio[i] = std::max(m_peakRatio[i], vehicle.loadRatio(i));
        m_peakForce = std::max(m_peakForce, f);
        xFront = std::max(xFront, vehicle.wheel(i).x);
        xRear = std::min(xRear, vehicle.wheel(i).x);
    }
    m_distance += speed * dt;

    const auto& hazards = road.hazards();
    int owner = -1;
    for (size_t k = 0; k < hazards.size() && k < m_state.size(); ++k)
    {
        const Hazard& h = hazards[k];
        HazardEvent& ev = m_events[k];

        if (m_state[k] == Pending && sFront + xFront >= h.s - h.halfExtent() - 1.0)
        {
            m_state[k] = Active;
            ev = HazardEvent{};
            ev.hazardId = h.id;
            ev.tStart = t;
            ev.speedKmh = speed * 3.6;
        }

        if (m_state[k] != Active)
            continue;

        if (owner < 0)
            owner = static_cast<int>(k);

        for (int i = 0; i < n; ++i)
        {
            const double f = vehicle.tireForce(i);
            const double r = vehicle.loadRatio(i);
            ev.peakForce = std::max(ev.peakForce, f);
            if (r > ev.peakRatio)
            {
                ev.peakRatio = r;
                ev.peakWheel = i;
            }
            ev.minRatio = std::min(ev.minRatio, r);
            if (f <= 0.0)
                ev.liftOff = true;
        }

        if (sFront + xRear >= h.s + h.halfExtent() + 8.0)
        {
            m_state[k] = Done;
            ev.tEnd = t;
            // damage of this sub-step goes to the event before it is closed
            if (owner == static_cast<int>(k))
            {
                ev.damage += increment;
                m_hazardDamage += increment;
                increment = 0.0;
                owner = -2;  // already booked
            }
            const double rate = backgroundRatePerKm();
            ev.equivalentKm = (rate > 0.0) ? ev.damage / rate : -1.0;
            m_finished.push_back(ev);
            ++m_finishedCount;
        }
    }

    if (owner >= 0)
    {
        m_events[owner].damage += increment;
        m_hazardDamage += increment;
    }
    else if (owner == -1)
    {
        m_bgDamage += increment;
        m_bgDistance += speed * dt;
    }
}

std::vector<HazardEvent> LoadAnalyzer::takeFinishedEvents()
{
    std::vector<HazardEvent> out;
    out.swap(m_finished);
    return out;
}

const HazardEvent* LoadAnalyzer::activeEvent() const
{
    for (size_t k = 0; k < m_state.size(); ++k)
    {
        if (m_state[k] == Active)
            return &m_events[k];
    }
    return nullptr;
}

double LoadAnalyzer::totalDamage() const
{
    double d = 0.0;
    for (double v : m_damage)
        d += v;
    return d;
}

double LoadAnalyzer::backgroundRatePerKm() const
{
    if (m_bgDistance < 300.0 || m_bgDamage <= 0.0)
        return -1.0;
    return m_bgDamage / (m_bgDistance / 1000.0);
}
