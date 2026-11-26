#pragma once

#include "MeshBuilder.h"

#include <array>
#include <string>
#include <vector>

#include <vtkSmartPointer.h>

class vtkUnstructuredGrid;

struct Pose2D
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;  // [rad], 0 = +x
};

// Planar centre line made of straight and circular-arc segments, parameterised by arc length s.
class RoadPath
{
public:
    RoadPath() = default;
    RoadPath(double x0, double y0, double headingRad);

    void addStraight(double length);
    void addArc(double radius, double angleDeg);  // + = left turn, - = right turn

    double length() const { return m_length; }

    // Beyond both ends the path is extended as a straight line.
    Pose2D pose(double s) const;
    double curvature(double s) const;             // [1/m], + = turning left
    Vec3 point(double s, double d, double z = 0.0) const;  // d = lateral offset, + = left

private:
    struct Segment
    {
        double s0 = 0.0;
        double length = 0.0;
        double kappa = 0.0;
        Pose2D start;
    };

    Pose2D m_start;
    Pose2D m_end;
    double m_length = 0.0;
    std::vector<Segment> m_segments;
};

// The three selectable maps
enum class MapType { Highway = 0, City = 1, National = 2 };

enum class RoadClass { Highway, City, SchoolZone, National, Village };

struct RoadZone
{
    double s0 = 0.0;
    double s1 = 0.0;
    RoadClass roadClass = RoadClass::Highway;
    std::string name;
    double speedLimitKmh = 50.0;
    double roughnessG0 = 16e-6;  // ISO 8608 displacement PSD Gd(n0), n0 = 0.1 cycle/m [m^3]
};

// "Start at" positions offered for a map
struct MapSection
{
    std::string name;
    double s = 0.0;
};

enum class HazardType { Pothole, SpeedBump, RaisedCrosswalk };

// Catalogue of hazards the user can place
enum class HazardKind
{
    PotholeSmall,      // 4 cm deep
    PotholeMedium,     // 7 cm deep
    PotholeLarge,      // 10 cm deep
    SpeedBump,         // circular hump 10 cm x 3.6 m (Korean standard)
    SpeedBumpShort,    // 7.5 cm x 2.0 m
    RubberHump,        // modular rubber hump 5 cm x 0.9 m
    RaisedCrosswalk,   // 10 cm plateau, 2 m ramps
    Count
};

// lateral placement of a pothole (speed bumps always span the carriageway)
enum class HazardLane { RightTrack, LeftTrack, BothTracks, AtPoint };

struct Hazard
{
    int id = 0;                // unique, stable while the map is loaded
    HazardType type = HazardType::Pothole;
    HazardKind kind = HazardKind::PotholeMedium;
    bool userPlaced = false;
    std::string label;   // long label, e.g. "POTHOLE #3  7 cm deep"
    std::string tag;     // short tag, e.g. "P3"
    std::string brief;   // compact table text, e.g. "P3  7 cm pothole"

    double s = 0.0;      // centre along the course [m]
    double d = 0.0;      // lateral centre (relative to the truck's lane centre) [m]
    double length = 0.0; // extent along the road [m]
    double width = 0.0;  // extent across the road [m]
    double height = 0.0; // bump height or pothole depth (positive) [m]
    double ramp = 0.0;   // raised crosswalk ramp length [m]

    double halfExtent() const { return 0.5 * length; }
    // geometric surface height at (u along, v across) relative to the hazard centre
    double profile(double u, double v) const;
};

// One drivable map: course (the truck's lane), speed zones, hazards, road input for the
// physics and the VTK meshes of the scenery.
class RoadNetwork
{
public:
    explicit RoadNetwork(MapType type = MapType::Highway);

    MapType type() const { return m_type; }
    static const char* mapName(MapType type);

    const RoadPath& course() const { return m_course; }
    double courseEnd() const { return m_sEnd; }
    const std::vector<MapSection>& sections() const { return m_sections; }

    const std::vector<RoadZone>& zones() const { return m_zones; }
    const RoadZone& zoneAt(double s) const;

    // ---- hazards ----
    const std::vector<Hazard>& hazards() const { return m_hazards; }
    const Hazard* findHazard(int id) const;
    static const char* hazardKindName(HazardKind kind);
    static bool isPotholeKind(HazardKind kind);
    // returns the new hazard id, or -1 (error message in *error)
    int addHazard(HazardKind kind, double s, HazardLane lane, double pointD, std::string* error);
    bool removeHazard(int id);
    void clearHazards();
    void restoreDefaultHazards();

    // ---- physics ----
    // vertical input at a wheel centre (tyre envelope of hazards + random roughness), [m]
    double wheelInput(double s, double d, double tireRadius) const;
    double hazardInput(double s, double d, double tireRadius) const;
    double roughness(double s, double d) const;
    // Speed the driver wants at s [m/s]: capped by capMs, by the speed limits (if obeyLimits),
    // by the curvature (comfortable lateral acceleration) and braking for everything ahead.
    double plannedSpeed(double s, double decel, double capMs, bool obeyLimits) const;
    // lowest comfortable corner speed anywhere in [s0, s1] (the whole vehicle must be out of a bend)
    double cornerSpeed(double s0, double s1) const;

    // closest course position for a world point (for click-to-place)
    bool project(double x, double y, double& s, double& d) const;

    // ---- visual helpers ----
    double surfaceHeight(double s, double d) const;  // raw hazard geometry
    std::array<double, 4> bounds() const { return m_bounds; }  // xmin, xmax, ymin, ymax
    double signOffset() const { return -3.6; }       // lateral position of zone signs

    vtkSmartPointer<vtkUnstructuredGrid> buildTerrain() const;
    vtkSmartPointer<vtkUnstructuredGrid> buildRoadSurface() const;  // asphalt + markings
    vtkSmartPointer<vtkUnstructuredGrid> buildCurbs() const;        // median / sidewalks / city blocks
    vtkSmartPointer<vtkUnstructuredGrid> buildRoadside() const;     // guard rails, lamps, signals, signs
    vtkSmartPointer<vtkUnstructuredGrid> buildScenery() const;      // trees, houses, buildings, fields
    vtkSmartPointer<vtkUnstructuredGrid> buildPotholes() const;
    vtkSmartPointer<vtkUnstructuredGrid> buildBumps() const;

private:
    struct DefaultHazard
    {
        HazardKind kind;
        double s;
        HazardLane lane;
    };
    struct Street  // straight city street with its crossings (arc length along the street)
    {
        RoadPath path;
        std::vector<double> crossings;
    };
    struct Block   // city block between streets
    {
        double x0, y0, x1, y1;
        int ix, iy;
    };

    void setupHighway();
    void setupCity();
    void setupNational();
    void buildRoughness();
    void computeBounds();
    double roughnessG0At(double s) const;
    double rigidWheelEnvelope(double s, double d, double R) const;
    void carriageway(double& center, double& width) const;
    Hazard makeHazard(HazardKind kind, double s, HazardLane lane, double pointD) const;
    void finalizeHazard(Hazard& h, bool userPlaced);
    void sortHazards();

    std::vector<Block> cityBlocks() const;
    void buildCityMarkings(MeshBuilder& mb) const;
    void buildCityBlocks(MeshBuilder& mb) const;
    void buildCityRoadside(MeshBuilder& mb) const;
    void buildCityScenery(MeshBuilder& mb) const;

    MapType m_type = MapType::Highway;
    RoadPath m_course;       // centre line of the truck's lane
    double m_sEnd = 0.0;

    std::vector<RoadZone> m_zones;
    std::vector<MapSection> m_sections;
    std::vector<DefaultHazard> m_defaults;
    std::vector<Hazard> m_hazards;
    int m_nextId = 1;
    int m_nPothole = 0;
    int m_nBump = 0;
    int m_nHump = 0;
    int m_nCross = 0;

    // national road: village section
    double m_sVillage0 = 0.0;
    double m_sVillage1 = 0.0;
    // city: grid
    std::vector<double> m_cityX;   // vertical street centre lines
    std::vector<double> m_cityY;   // horizontal street centre lines (ascending)
    std::array<double, 4> m_cityBox{};  // xmin, xmax, ymin, ymax
    std::vector<Street> m_streets;
    double m_schoolS0 = 0.0;
    double m_schoolS1 = 0.0;

    struct RoughTerm
    {
        double k = 0.0;       // 2*pi*n [rad/m]
        double amp = 0.0;     // amplitude for Gd(n0) = 1
        double phaseL = 0.0;  // left wheel track
        double phaseR = 0.0;  // right wheel track
    };
    std::vector<RoughTerm> m_rough;

    std::array<double, 4> m_bounds{};
};
