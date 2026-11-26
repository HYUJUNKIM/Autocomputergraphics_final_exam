#pragma once

#include "VehicleModel.h"

#include <array>
#include <vector>

class RoadNetwork;

enum class Severity { Low, Moderate, High, Severe };

Severity severityOf(double peakLoadRatio);
const char* severityName(Severity s);

// Result of driving over one hazard (pothole / bump / crosswalk).
struct HazardEvent
{
    int hazardId = -1;
    double tStart = 0.0;
    double tEnd = 0.0;
    double speedKmh = 0.0;     // speed when the front axle reached the hazard
    double peakForce = 0.0;    // largest tyre force [N]
    double peakRatio = 0.0;    // largest load / static load
    int peakWheel = -1;
    double minRatio = 1e9;
    bool liftOff = false;      // a tyre left the ground
    double damage = 0.0;       // pseudo-damage accumulated during the event
    double equivalentKm = -1.0;  // same damage expressed as km of hazard-free road
};

// Fatigue bookkeeping for the wheel stations.
//
// Pseudo-damage (Miner's rule on an S-N line with slope k) is computed from the vertical tyre
// force of every wheel. Load reversals are detected on the fly (with a small hysteresis gate) and
// every half cycle with range dF adds  0.5 * (dF / dFref)^k , dFref = 50 kN for every vehicle,
// so heavier vehicles (larger absolute loads) accumulate more damage.
//
// Each hazard opens an event window when the front axle approaches it and closes it when the
// last axle has passed (plus a few metres for the rebound). Damage outside any window is
// attributed to normal road roughness, which gives the "equivalent km" comparison.
class LoadAnalyzer
{
public:
    static constexpr int kMaxWheels = VehicleModel::kMaxWheels;

    enum HazardState { Pending = 0, Active = 1, Done = 2, Skipped = 3 };

    void reset(const RoadNetwork& road, const VehicleModel& vehicle, double sStart);
    // call after hazards were added / removed: keeps the state of existing hazards
    void syncHazards(const RoadNetwork& road, double sFront);

    void setExponent(double k) { m_exponent = k; }
    double exponent() const { return m_exponent; }
    double referenceRange() const { return m_refRange; }

    // call once per physics sub-step
    void update(double t, double dt, double sFront, double speed,
                const RoadNetwork& road, const VehicleModel& vehicle);

    std::vector<HazardEvent> takeFinishedEvents();
    const HazardEvent* activeEvent() const;
    int hazardState(int index) const { return m_state[index]; }  // index into road.hazards()
    const HazardEvent& hazardResult(int index) const { return m_events[index]; }

    double wheelDamage(int i) const { return m_damage[i]; }
    double wheelPeakRatio(int i) const { return m_peakRatio[i]; }
    double peakForce() const { return m_peakForce; }
    double totalDamage() const;
    double hazardDamage() const { return m_hazardDamage; }
    double backgroundDamage() const { return m_bgDamage; }
    double backgroundDistance() const { return m_bgDistance; }
    double distance() const { return m_distance; }
    double backgroundRatePerKm() const;  // < 0 when not enough distance yet
    int finishedEventCount() const { return m_finishedCount; }

private:
    struct Counter
    {
        bool init = false;
        int dir = 0;
        double lastTurn = 0.0;
        double extreme = 0.0;
    };
    double countReversal(Counter& c, double force) const;
    double halfCycle(double range) const;

    std::array<Counter, kMaxWheels> m_counters{};
    std::array<double, kMaxWheels> m_damage{};
    std::array<double, kMaxWheels> m_peakRatio{};
    double m_peakForce = 0.0;

    // parallel to road.hazards()
    std::vector<int> m_ids;
    std::vector<int> m_state;
    std::vector<HazardEvent> m_events;
    std::vector<HazardEvent> m_finished;
    int m_finishedCount = 0;

    double m_exponent = 5.0;
    double m_refRange = 50e3;  // [N]
    double m_gate = 300.0;     // reversal hysteresis [N], 1 % of the static wheel load

    double m_bgDamage = 0.0;
    double m_bgDistance = 0.0;
    double m_distance = 0.0;
    double m_hazardDamage = 0.0;
};
