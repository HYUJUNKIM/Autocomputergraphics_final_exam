#include "RoadNetwork.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include <vtkCellType.h>
#include <vtkUnstructuredGrid.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTrackOffset = 0.92;    // wheel track offset from the lane centre [m]
constexpr double kAyMax = 1.8;           // comfortable lateral acceleration, loaded truck [m/s^2]

// city geometry
constexpr double kCityHw = 6.4;          // carriageway half width (4 lanes x 3.2 m)
constexpr double kCitySidewalk = 3.5;
constexpr double kCityCornerR = 9.0;     // kerb radius at the block corners
constexpr double kCurbHeight = 0.15;

inline double sq(double v) { return v * v; }

// ---- palette ----
const Rgb kAsphalt     = makeRgb(64, 65, 69);
const Rgb kShoulder    = makeRgb(100, 100, 102);
const Rgb kMedian      = makeRgb(118, 118, 114);
const Rgb kWhite       = makeRgb(236, 236, 236);
const Rgb kYellow      = makeRgb(240, 186, 36);
const Rgb kConcrete    = makeRgb(184, 182, 174);
const Rgb kSteel       = makeRgb(172, 178, 184);
const Rgb kSidewalk    = makeRgb(178, 168, 152);
const Rgb kGrass       = makeRgb(98, 140, 70);
const Rgb kLawn        = makeRgb(92, 150, 72);
const Rgb kSignGreen   = makeRgb(18, 112, 66);
const Rgb kSignBlue    = makeRgb(24, 70, 150);
const Rgb kLamp        = makeRgb(252, 246, 205);
const Rgb kPole        = makeRgb(150, 150, 150);
const Rgb kTrunk       = makeRgb(98, 72, 50);
const Rgb kSchoolRed   = makeRgb(176, 58, 48);

const Rgb kLeaves[] = { makeRgb(52, 110, 50), makeRgb(64, 124, 56), makeRgb(44, 96, 46), makeRgb(78, 132, 60) };

Pose2D advance(const Pose2D& p, double ds, double kappa)
{
    Pose2D r;
    if (std::abs(kappa) < 1e-12)
    {
        r.x = p.x + ds * std::cos(p.heading);
        r.y = p.y + ds * std::sin(p.heading);
        r.heading = p.heading;
    }
    else
    {
        const double h1 = p.heading + kappa * ds;
        r.x = p.x + (std::sin(h1) - std::sin(p.heading)) / kappa;
        r.y = p.y - (std::cos(h1) - std::cos(p.heading)) / kappa;
        r.heading = h1;
    }
    return r;
}

// ---------------------------------------------------------------------------
// mesh helpers (all geometry follows the path, so it bends with the curves)
// ---------------------------------------------------------------------------

// flat strip between lateral offsets d0 < d1
void ribbon(MeshBuilder& mb, const RoadPath& path, double s0, double s1,
            double d0, double d1, double z, double step = 4.0)
{
    if (s1 <= s0)
        return;
    const int n = std::max(1, static_cast<int>(std::ceil((s1 - s0) / step)));
    vtkIdType prevA = -1;
    vtkIdType prevB = -1;
    for (int i = 0; i <= n; ++i)
    {
        const double s = s0 + (s1 - s0) * i / n;
        const vtkIdType a = mb.addPoint(path.point(s, d0, z));
        const vtkIdType b = mb.addPoint(path.point(s, d1, z));
        if (i > 0)
            mb.addCell(VTK_QUAD, { prevA, a, b, prevB });
        prevA = a;
        prevB = b;
    }
}

void dashed(MeshBuilder& mb, const RoadPath& path, double s0, double s1,
            double d, double w, double z, double on, double off)
{
    for (double s = s0; s < s1; s += on + off)
        ribbon(mb, path, s, std::min(s + on, s1), d - 0.5 * w, d + 0.5 * w, z, on);
}

// solid block centred at lateral offset d with full width w
void wall(MeshBuilder& mb, const RoadPath& path, double s0, double s1,
          double d, double w, double z0, double z1, double step = 4.0)
{
    if (s1 <= s0)
        return;
    const int n = std::max(1, static_cast<int>(std::ceil((s1 - s0) / step)));
    const double dl = d - 0.5 * w;
    const double dr = d + 0.5 * w;
    for (int i = 0; i < n; ++i)
    {
        const double sa = s0 + (s1 - s0) * i / n;
        const double sb = s0 + (s1 - s0) * (i + 1) / n;
        mb.addHex({ path.point(sa, dl, z0), path.point(sb, dl, z0), path.point(sb, dr, z0), path.point(sa, dr, z0),
                    path.point(sa, dl, z1), path.point(sb, dl, z1), path.point(sb, dr, z1), path.point(sa, dr, z1) });
    }
}

void pathBox(MeshBuilder& mb, const RoadPath& path, double s, double d,
             double halfLen, double halfWid, double z0, double z1)
{
    wall(mb, path, s - halfLen, s + halfLen, d, 2.0 * halfWid, z0, z1, 2.0 * halfLen + 1.0);
}

void guardrail(MeshBuilder& mb, const RoadPath& path, double s0, double s1, double d, double towardRoad)
{
    if (s1 <= s0)
        return;
    mb.setColor(kSteel);
    for (double s = s0; s <= s1; s += 4.0)
        pathBox(mb, path, s, d, 0.08, 0.08, 0.0, 0.78);
    wall(mb, path, s0, s1, d + 0.12 * towardRoad, 0.06, 0.48, 0.80, 4.0);
}

// street lamp: pole at d, arms reaching armL to the left (+d) and armR to the right (-d)
void lampPole(MeshBuilder& mb, const RoadPath& path, double s, double d, double z0,
              double height, double armL, double armR)
{
    mb.setColor(kPole);
    pathBox(mb, path, s, d, 0.12, 0.12, z0, height);
    pathBox(mb, path, s, d + 0.5 * (armL - armR), 0.06, 0.5 * (armL + armR), height - 0.4, height - 0.25);
    mb.setColor(kLamp);
    if (armL > 0.0)
        pathBox(mb, path, s, d + armL - 0.45, 0.22, 0.45, height - 0.55, height - 0.4);
    if (armR > 0.0)
        pathBox(mb, path, s, d - armR + 0.45, 0.22, 0.45, height - 0.55, height - 0.4);
}

void gantry(MeshBuilder& mb, const RoadPath& path, double s, double dRight, double dLeft,
            double signD0, double signD1, const Rgb& signColor)
{
    mb.setColor(kPole);
    pathBox(mb, path, s, dRight, 0.2, 0.2, 0.0, 6.8);
    pathBox(mb, path, s, dLeft, 0.2, 0.2, 0.0, 6.8);
    pathBox(mb, path, s, 0.5 * (dRight + dLeft), 0.15, 0.5 * (dLeft - dRight), 6.3, 6.6);
    mb.setColor(signColor);
    pathBox(mb, path, s - 0.25, 0.5 * (signD0 + signD1), 0.06, 0.5 * (signD1 - signD0), 5.6, 8.1);
}

void tree(MeshBuilder& mb, double x, double y, double z, double canopyR, double canopyH, const Rgb& leaf)
{
    mb.setColor(kTrunk);
    mb.addBox({ x - 0.15, y - 0.15, z - 0.15 }, { x + 0.15, y + 0.15, z + 2.0 });
    mb.setColor(leaf);
    mb.addCone({ x, y, z + 1.6 }, canopyR, canopyH, 10);
}

// rounded rectangle outline, counter-clockwise seen from +z
std::vector<Vec3> roundedRect(double x0, double y0, double x1, double y1, double r, double z, int segs = 6)
{
    r = std::min({ r, 0.5 * (x1 - x0), 0.5 * (y1 - y0) });
    const double cx[4] = { x1 - r, x0 + r, x0 + r, x1 - r };
    const double cy[4] = { y1 - r, y1 - r, y0 + r, y0 + r };
    std::vector<Vec3> pts;
    for (int c = 0; c < 4; ++c)
    {
        for (int k = 0; k <= segs; ++k)
        {
            const double t = 0.5 * kPi * (c + static_cast<double>(k) / segs);
            pts.push_back({ cx[c] + r * std::cos(t), cy[c] + r * std::sin(t), z });
        }
    }
    return pts;
}

// raised slab with vertical sides (outline must be convex)
void slab(MeshBuilder& mb, const std::vector<Vec3>& outline, double z0, double z1, const Rgb& top, const Rgb& side)
{
    Vec3 c{ 0.0, 0.0, z1 };
    std::vector<Vec3> ring;
    for (const auto& p : outline)
    {
        c[0] += p[0] / outline.size();
        c[1] += p[1] / outline.size();
        ring.push_back({ p[0], p[1], z1 });
    }
    mb.setColor(top);
    mb.addTriangleFan(c, ring);
    mb.setColor(side);
    for (size_t i = 0; i < outline.size(); ++i)
    {
        const Vec3& a = outline[i];
        const Vec3& b = outline[(i + 1) % outline.size()];
        mb.addQuad({ a[0], a[1], z0 }, { b[0], b[1], z0 }, { b[0], b[1], z1 }, { a[0], a[1], z1 });
    }
}
} // namespace

// ===========================================================================
// RoadPath
// ===========================================================================

RoadPath::RoadPath(double x0, double y0, double headingRad)
{
    m_start = { x0, y0, headingRad };
    m_end = m_start;
}

void RoadPath::addStraight(double length)
{
    Segment seg;
    seg.s0 = m_length;
    seg.length = length;
    seg.kappa = 0.0;
    seg.start = m_end;
    m_segments.push_back(seg);
    m_end = advance(m_end, length, 0.0);
    m_length += length;
}

void RoadPath::addArc(double radius, double angleDeg)
{
    const double angle = angleDeg * kPi / 180.0;
    Segment seg;
    seg.s0 = m_length;
    seg.length = radius * std::abs(angle);
    seg.kappa = (angle >= 0.0 ? 1.0 : -1.0) / radius;
    seg.start = m_end;
    m_segments.push_back(seg);
    m_end = advance(m_end, seg.length, seg.kappa);
    m_length += seg.length;
}

Pose2D RoadPath::pose(double s) const
{
    if (m_segments.empty() || s <= 0.0)
        return advance(m_start, s, 0.0);
    if (s >= m_length)
        return advance(m_end, s - m_length, 0.0);

    for (const auto& seg : m_segments)
    {
        if (s < seg.s0 + seg.length)
            return advance(seg.start, s - seg.s0, seg.kappa);
    }
    return m_end;
}

double RoadPath::curvature(double s) const
{
    if (s <= 0.0 || s >= m_length)
        return 0.0;
    for (const auto& seg : m_segments)
    {
        if (s < seg.s0 + seg.length)
            return seg.kappa;
    }
    return 0.0;
}

Vec3 RoadPath::point(double s, double d, double z) const
{
    const Pose2D p = pose(s);
    return { p.x - d * std::sin(p.heading), p.y + d * std::cos(p.heading), z };
}

// ===========================================================================
// Hazard
// ===========================================================================

double Hazard::profile(double u, double v) const
{
    if (std::abs(v) > 0.5 * width)
        return 0.0;

    switch (type)
    {
    case HazardType::Pothole:
    {
        // elliptic hole with a flat bottom and sharp edges
        const double e = 2.0 * v / width;
        const double halfLen = 0.5 * length * std::sqrt(std::max(0.0, 1.0 - e * e));
        return (std::abs(u) < halfLen) ? -height : 0.0;
    }
    case HazardType::SpeedBump:
    {
        // circular-arc hump
        if (std::abs(u) > 0.5 * length)
            return 0.0;
        const double rc = (0.25 * length * length + height * height) / (2.0 * height);
        return std::sqrt(rc * rc - u * u) - (rc - height);
    }
    case HazardType::RaisedCrosswalk:
    {
        // trapezoid: ramp - flat top - ramp
        const double top = 0.5 * length - ramp;
        const double au = std::abs(u);
        if (au <= top)
            return height;
        if (au <= 0.5 * length)
            return height * (0.5 * length - au) / ramp;
        return 0.0;
    }
    }
    return 0.0;
}

// ===========================================================================
// RoadNetwork: maps
// ===========================================================================

const char* RoadNetwork::mapName(MapType type)
{
    switch (type)
    {
    case MapType::Highway: return "Highway";
    case MapType::City: return "City driving";
    case MapType::National: return "National road";
    }
    return "";
}

RoadNetwork::RoadNetwork(MapType type)
    : m_type(type)
{
    buildRoughness();
    switch (type)
    {
    case MapType::Highway: setupHighway(); break;
    case MapType::City: setupCity(); break;
    case MapType::National: setupNational(); break;
    }
    restoreDefaultHazards();
    computeBounds();
}

void RoadNetwork::setupHighway()
{
    // 2 x 2 lane motorway, truck in the right lane, two long curves
    m_course = RoadPath(0.0, 0.0, 0.0);
    m_course.addStraight(700.0);
    m_course.addArc(900.0, 20.0);
    m_course.addStraight(600.0);
    m_course.addArc(1200.0, -30.0);
    m_course.addStraight(800.0);
    m_sEnd = m_course.length();

    // cars 100 km/h (trucks are capped at 90 km/h by their own vehicle limit); ISO 8608 class A
    m_zones = { { -1e9, 1e9, RoadClass::Highway, "HIGHWAY", 100.0, 16e-6 } };
    m_sections = { { "Start (km 0)", 0.0 }, { "km 1.0", 1000.0 }, { "km 2.0", 2000.0 } };

    m_defaults = {
        { HazardKind::PotholeSmall,  300.0, HazardLane::RightTrack },
        { HazardKind::PotholeMedium, 560.0, HazardLane::LeftTrack },
        { HazardKind::PotholeSmall,  850.0, HazardLane::BothTracks },
        { HazardKind::PotholeLarge, 1250.0, HazardLane::RightTrack },
        { HazardKind::PotholeMedium, 1750.0, HazardLane::LeftTrack },
        { HazardKind::PotholeLarge, 2350.0, HazardLane::RightTrack },
        { HazardKind::PotholeSmall, 2750.0, HazardLane::BothTracks },
    };
}

void RoadNetwork::setupNational()
{
    // two-lane rural road with a village (speed bumps, raised crosswalk)
    m_course = RoadPath(0.0, 0.0, 0.0);
    m_course.addStraight(350.0);
    m_course.addArc(300.0, 30.0);
    m_sVillage0 = m_course.length();
    m_course.addStraight(420.0);
    m_sVillage1 = m_course.length();
    m_course.addArc(200.0, -25.0);
    m_course.addStraight(350.0);
    m_course.addArc(250.0, 40.0);
    m_course.addStraight(400.0);
    m_sEnd = m_course.length();

    m_zones = {
        { -1e9,        m_sVillage0, RoadClass::National, "NATIONAL ROAD", 70.0, 64e-6 },
        { m_sVillage0, m_sVillage1, RoadClass::Village,  "VILLAGE ZONE",  30.0, 96e-6 },
        { m_sVillage1, 1e9,         RoadClass::National, "NATIONAL ROAD", 70.0, 64e-6 },
    };
    m_sections = { { "Start", 0.0 },
                   { "Village (speed bumps)", m_sVillage0 - 90.0 },
                   { "After the village", m_sVillage1 + 20.0 } };

    const double v0 = m_sVillage0;
    const double v1 = m_sVillage1;
    m_defaults = {
        { HazardKind::PotholeMedium, 150.0, HazardLane::RightTrack },
        { HazardKind::PotholeLarge, 420.0, HazardLane::LeftTrack },
        { HazardKind::SpeedBump, v0 + 60.0, HazardLane::BothTracks },
        { HazardKind::RaisedCrosswalk, v0 + 170.0, HazardLane::BothTracks },
        { HazardKind::SpeedBump, v0 + 270.0, HazardLane::BothTracks },
        { HazardKind::PotholeSmall, v0 + 330.0, HazardLane::RightTrack },
        { HazardKind::SpeedBumpShort, v0 + 385.0, HazardLane::BothTracks },
        { HazardKind::PotholeMedium, v1 + 150.0, HazardLane::LeftTrack },
        { HazardKind::PotholeLarge, v1 + 420.0, HazardLane::RightTrack },
    };
}

void RoadNetwork::setupCity()
{
    // street grid: 4-lane streets, 130 m blocks. The course is a clockwise loop around
    // six blocks (right turns only), truck in the right-hand (kerb) lane.
    m_cityX = { 0.0, 130.0, 260.0, 390.0 };
    m_cityY = { -260.0, -130.0, 0.0 };
    m_cityBox = { -140.0, 530.0, -400.0, 140.0 };

    const double laneOff = 4.8;                    // kerb lane centre from the street centre line
    const double R = kCityCornerR + 1.6;           // turn radius of the lane centre
    const double cornerC = kCityHw + kCityCornerR; // kerb-corner centre offset from the crossing

    // start on the north street heading east
    const double xStart = 20.0;
    m_course = RoadPath(xStart, m_cityY[2] - laneOff, 0.0);
    const double l1 = (m_cityX[3] - cornerC) - xStart;
    const double l2 = (m_cityY[2] - cornerC) - (m_cityY[0] + cornerC);
    const double l3 = (m_cityX[3] - cornerC) - (m_cityX[0] + cornerC);
    const double arc = 0.5 * kPi * R;

    m_course.addStraight(l1);
    m_course.addArc(R, -90.0);
    const double s2 = m_course.length();
    m_course.addStraight(l2);
    m_course.addArc(R, -90.0);
    const double s3 = m_course.length();
    m_course.addStraight(l3);
    m_course.addArc(R, -90.0);
    const double s4 = m_course.length();
    m_course.addStraight(l2);
    m_course.addArc(R, -90.0);
    m_course.addStraight((xStart - (m_cityX[0] + cornerC)) + 60.0);  // past the start line
    m_sEnd = m_course.length();
    (void)arc;

    // south street = school zone in front of the school block
    m_schoolS0 = s3 + 30.0;
    m_schoolS1 = s3 + l3 - 30.0;
    m_zones = {
        { -1e9,       m_schoolS0, RoadClass::City,       "CITY ROAD",   50.0, 64e-6 },
        { m_schoolS0, m_schoolS1, RoadClass::SchoolZone, "SCHOOL ZONE", 30.0, 64e-6 },
        { m_schoolS1, 1e9,        RoadClass::City,       "CITY ROAD",   50.0, 64e-6 },
    };
    m_sections = { { "North street (start)", 0.0 },
                   { "East street", s2 + 5.0 },
                   { "School zone (south)", s3 + 5.0 },
                   { "West street", s4 + 5.0 } };

    m_defaults = {
        { HazardKind::PotholeMedium, 165.0, HazardLane::RightTrack },
        { HazardKind::PotholeSmall, 300.0, HazardLane::LeftTrack },
        { HazardKind::RaisedCrosswalk, s2 + 55.0, HazardLane::BothTracks },
        { HazardKind::PotholeLarge, s2 + 175.0, HazardLane::RightTrack },
        { HazardKind::SpeedBump, s3 + 55.0, HazardLane::BothTracks },
        { HazardKind::RaisedCrosswalk, s3 + 180.0, HazardLane::BothTracks },
        { HazardKind::SpeedBump, s3 + 290.0, HazardLane::BothTracks },
        { HazardKind::RubberHump, s3 + 318.0, HazardLane::BothTracks },
        { HazardKind::PotholeMedium, s4 + 60.0, HazardLane::LeftTrack },
        { HazardKind::PotholeLarge, s4 + 170.0, HazardLane::RightTrack },
    };

    // straight streets for the visual model
    m_streets.clear();
    for (double y : m_cityY)
    {
        Street st;
        st.path = RoadPath(m_cityBox[0], y, 0.0);
        st.path.addStraight(m_cityBox[1] - m_cityBox[0]);
        for (double x : m_cityX)
            st.crossings.push_back(x - m_cityBox[0]);
        m_streets.push_back(st);
    }
    for (double x : m_cityX)
    {
        Street st;
        st.path = RoadPath(x, m_cityBox[2], 0.5 * kPi);
        st.path.addStraight(m_cityBox[3] - m_cityBox[2]);
        for (double y : m_cityY)
            st.crossings.push_back(y - m_cityBox[2]);
        m_streets.push_back(st);
    }
}

void RoadNetwork::buildRoughness()
{
    // Random road profile with an ISO 8608 PSD Gd(n) = Gd(n0) * (n/n0)^-2 (sum of sinusoids).
    // Long wavelengths are identical left/right, short ones become uncorrelated.
    std::mt19937 rng(20261117u);
    std::uniform_real_distribution<double> phase(0.0, 2.0 * kPi);

    const int count = 56;
    const double nMin = 0.02;  // 50 m wavelength
    const double nMax = 2.0;   // 0.5 m wavelength
    const double logStep = std::log(nMax / nMin) / (count - 1);

    m_rough.clear();
    for (int i = 0; i < count; ++i)
    {
        const double n = nMin * std::exp(logStep * i);
        const double dn = n * logStep;
        const double psd = std::pow(n / 0.1, -2.0);  // for Gd(n0) = 1
        RoughTerm t;
        t.k = 2.0 * kPi * n;
        t.amp = std::sqrt(2.0 * psd * dn);
        t.phaseL = phase(rng);
        const double decorrelation = std::clamp((n - 0.05) / 0.3, 0.0, 1.0);
        t.phaseR = t.phaseL + decorrelation * phase(rng);
        m_rough.push_back(t);
    }
}

void RoadNetwork::computeBounds()
{
    if (m_type == MapType::City)
    {
        const double m = 150.0;
        m_bounds = { m_cityBox[0] - m, m_cityBox[1] + m, m_cityBox[2] - m, m_cityBox[3] + m };
        return;
    }

    double xmin = 1e18, xmax = -1e18, ymin = 1e18, ymax = -1e18;
    for (double s = -260.0; s <= m_sEnd + 200.0; s += 10.0)
    {
        const Vec3 q = m_course.point(s, 0.0);
        xmin = std::min(xmin, q[0]); xmax = std::max(xmax, q[0]);
        ymin = std::min(ymin, q[1]); ymax = std::max(ymax, q[1]);
    }
    const double margin = 260.0;
    m_bounds = { xmin - margin, xmax + margin, ymin - margin, ymax + margin };
}

const RoadZone& RoadNetwork::zoneAt(double s) const
{
    for (const auto& z : m_zones)
    {
        if (s >= z.s0 && s < z.s1)
            return z;
    }
    return m_zones.back();
}

// ===========================================================================
// hazards
// ===========================================================================

const char* RoadNetwork::hazardKindName(HazardKind kind)
{
    switch (kind)
    {
    case HazardKind::PotholeSmall: return "Pothole - small (4 cm deep)";
    case HazardKind::PotholeMedium: return "Pothole - medium (7 cm deep)";
    case HazardKind::PotholeLarge: return "Pothole - large (10 cm deep)";
    case HazardKind::SpeedBump: return "Speed bump - standard (10 cm x 3.6 m)";
    case HazardKind::SpeedBumpShort: return "Speed bump - short (7.5 cm x 2.0 m)";
    case HazardKind::RubberHump: return "Rubber speed hump (5 cm x 0.9 m)";
    case HazardKind::RaisedCrosswalk: return "Raised crosswalk (10 cm)";
    case HazardKind::Count: break;
    }
    return "";
}

bool RoadNetwork::isPotholeKind(HazardKind kind)
{
    return kind == HazardKind::PotholeSmall || kind == HazardKind::PotholeMedium || kind == HazardKind::PotholeLarge;
}

const Hazard* RoadNetwork::findHazard(int id) const
{
    for (const auto& h : m_hazards)
    {
        if (h.id == id)
            return &h;
    }
    return nullptr;
}

void RoadNetwork::carriageway(double& center, double& width) const
{
    // lateral extent of the carriageway a speed bump spans (relative to the truck's lane centre)
    switch (m_type)
    {
    case MapType::Highway: center = 1.8; width = 7.2; break;    // the truck's two lanes
    case MapType::City: center = 4.8; width = 12.8; break;      // all four lanes
    case MapType::National: center = 1.75; width = 7.0; break;  // both lanes
    }
}

Hazard RoadNetwork::makeHazard(HazardKind kind, double s, HazardLane lane, double pointD) const
{
    Hazard h;
    h.kind = kind;
    h.s = s;
    switch (kind)
    {
    case HazardKind::PotholeSmall:
        h.type = HazardType::Pothole; h.length = 0.6; h.width = 0.8; h.height = 0.04; break;
    case HazardKind::PotholeMedium:
        h.type = HazardType::Pothole; h.length = 0.9; h.width = 1.0; h.height = 0.07; break;
    case HazardKind::PotholeLarge:
        h.type = HazardType::Pothole; h.length = 1.2; h.width = 1.1; h.height = 0.10; break;
    case HazardKind::SpeedBump:
        h.type = HazardType::SpeedBump; h.length = 3.6; h.height = 0.10; break;
    case HazardKind::SpeedBumpShort:
        h.type = HazardType::SpeedBump; h.length = 2.0; h.height = 0.075; break;
    case HazardKind::RubberHump:
        h.type = HazardType::SpeedBump; h.length = 0.9; h.height = 0.05; break;
    case HazardKind::RaisedCrosswalk:
        h.type = HazardType::RaisedCrosswalk; h.ramp = 2.0; h.length = 4.0 + 2.0 * h.ramp; h.height = 0.10; break;
    case HazardKind::Count:
        break;
    }

    if (h.type == HazardType::Pothole)
    {
        switch (lane)
        {
        case HazardLane::RightTrack: h.d = -kTrackOffset; break;
        case HazardLane::LeftTrack: h.d = kTrackOffset; break;
        case HazardLane::BothTracks: h.d = 0.0; h.width = 2.6; break;
        case HazardLane::AtPoint: h.d = std::clamp(pointD, -3.0, 3.0); break;
        }
    }
    else
    {
        carriageway(h.d, h.width);
    }
    return h;
}

void RoadNetwork::finalizeHazard(Hazard& h, bool userPlaced)
{
    h.id = m_nextId++;
    h.userPlaced = userPlaced;
    char buf[128];
    const char* user = userPlaced ? "  (placed)" : "";
    switch (h.kind)
    {
    case HazardKind::PotholeSmall:
    case HazardKind::PotholeMedium:
    case HazardKind::PotholeLarge:
        ++m_nPothole;
        std::snprintf(buf, sizeof(buf), "POTHOLE #%d  %.0f cm deep%s", m_nPothole, h.height * 100.0, user);
        h.label = buf;
        std::snprintf(buf, sizeof(buf), "P%d", m_nPothole);
        h.tag = buf;
        std::snprintf(buf, sizeof(buf), "P%d  %.0f cm pothole", m_nPothole, h.height * 100.0);
        h.brief = buf;
        break;
    case HazardKind::SpeedBump:
    case HazardKind::SpeedBumpShort:
        ++m_nBump;
        std::snprintf(buf, sizeof(buf), "SPEED BUMP #%d  %.1f cm x %.1f m%s", m_nBump, h.height * 100.0, h.length, user);
        h.label = buf;
        std::snprintf(buf, sizeof(buf), "B%d", m_nBump);
        h.tag = buf;
        std::snprintf(buf, sizeof(buf), "B%d  %.1f cm bump", m_nBump, h.height * 100.0);
        h.brief = buf;
        break;
    case HazardKind::RubberHump:
        ++m_nHump;
        std::snprintf(buf, sizeof(buf), "RUBBER HUMP #%d  %.0f cm x %.1f m%s", m_nHump, h.height * 100.0, h.length, user);
        h.label = buf;
        std::snprintf(buf, sizeof(buf), "H%d", m_nHump);
        h.tag = buf;
        std::snprintf(buf, sizeof(buf), "H%d  %.0f cm rubber hump", m_nHump, h.height * 100.0);
        h.brief = buf;
        break;
    case HazardKind::RaisedCrosswalk:
        ++m_nCross;
        std::snprintf(buf, sizeof(buf), "RAISED CROSSWALK #%d  %.0f cm%s", m_nCross, h.height * 100.0, user);
        h.label = buf;
        std::snprintf(buf, sizeof(buf), "X%d", m_nCross);
        h.tag = buf;
        std::snprintf(buf, sizeof(buf), "X%d  %.0f cm crosswalk", m_nCross, h.height * 100.0);
        h.brief = buf;
        break;
    case HazardKind::Count:
        break;
    }
}

void RoadNetwork::sortHazards()
{
    std::sort(m_hazards.begin(), m_hazards.end(),
              [](const Hazard& a, const Hazard& b) { return a.s < b.s; });
}

int RoadNetwork::addHazard(HazardKind kind, double s, HazardLane lane, double pointD, std::string* error)
{
    if (s < 15.0 || s > m_sEnd - 30.0)
    {
        if (error)
            *error = "outside the drivable part of the course";
        return -1;
    }

    Hazard h = makeHazard(kind, s, lane, pointD);
    for (const auto& e : m_hazards)
    {
        const bool alongOverlap = std::abs(e.s - h.s) < e.halfExtent() + h.halfExtent() + 1.0;
        const bool acrossOverlap = std::abs(e.d - h.d) < 0.5 * (e.width + h.width);
        if (alongOverlap && acrossOverlap)
        {
            if (error)
                *error = "too close to " + e.label;
            return -1;
        }
    }

    finalizeHazard(h, true);
    m_hazards.push_back(h);
    sortHazards();
    return h.id;
}

bool RoadNetwork::removeHazard(int id)
{
    const auto it = std::find_if(m_hazards.begin(), m_hazards.end(), [id](const Hazard& h) { return h.id == id; });
    if (it == m_hazards.end())
        return false;
    m_hazards.erase(it);
    return true;
}

void RoadNetwork::clearHazards()
{
    m_hazards.clear();
    m_nPothole = m_nBump = m_nHump = m_nCross = 0;
}

void RoadNetwork::restoreDefaultHazards()
{
    clearHazards();
    std::vector<DefaultHazard> defaults = m_defaults;
    std::sort(defaults.begin(), defaults.end(),
              [](const DefaultHazard& a, const DefaultHazard& b) { return a.s < b.s; });
    for (const auto& d : defaults)
    {
        Hazard h = makeHazard(d.kind, d.s, d.lane, 0.0);
        finalizeHazard(h, false);
        m_hazards.push_back(h);
    }
    sortHazards();
}

// ===========================================================================
// physics
// ===========================================================================

double RoadNetwork::roughnessG0At(double s) const
{
    const double blend = 15.0;
    for (size_t i = 0; i < m_zones.size(); ++i)
    {
        const RoadZone& z = m_zones[i];
        if (s < z.s0 || s >= z.s1)
            continue;

        if (i > 0 && s < z.s0 + blend)
        {
            const double t = (s - (z.s0 - blend)) / (2.0 * blend);
            return m_zones[i - 1].roughnessG0 + t * (z.roughnessG0 - m_zones[i - 1].roughnessG0);
        }
        if (i + 1 < m_zones.size() && s > z.s1 - blend)
        {
            const double t = (s - (z.s1 - blend)) / (2.0 * blend);
            return z.roughnessG0 + t * (m_zones[i + 1].roughnessG0 - z.roughnessG0);
        }
        return z.roughnessG0;
    }
    return m_zones.back().roughnessG0;
}

double RoadNetwork::roughness(double s, double d) const
{
    const bool left = d >= 0.0;
    double sum = 0.0;
    for (const auto& t : m_rough)
        sum += t.amp * std::sin(t.k * s + (left ? t.phaseL : t.phaseR));
    return std::sqrt(roughnessG0At(s)) * sum;
}

double RoadNetwork::hazardInput(double s, double d, double R) const
{
    // A tyre is not a rigid disc: it wraps around sharp edges over its contact patch.
    // Approximate this by averaging the rigid-wheel envelope over a ~25 cm patch.
    bool near = false;
    for (const auto& h : m_hazards)
    {
        if (std::abs(s - h.s) <= h.halfExtent() + R + 0.15 && std::abs(d - h.d) <= 0.5 * h.width)
        {
            near = true;
            break;
        }
    }
    if (!near)
        return 0.0;

    static const double offsets[5] = { -0.12, -0.06, 0.0, 0.06, 0.12 };
    static const double weights[5] = { 1.0 / 9.0, 2.0 / 9.0, 3.0 / 9.0, 2.0 / 9.0, 1.0 / 9.0 };
    double z = 0.0;
    for (int k = 0; k < 5; ++k)
        z += weights[k] * rigidWheelEnvelope(s + offsets[k], d, R);
    return z;
}

double RoadNetwork::rigidWheelEnvelope(double s, double d, double R) const
{
    // A rigid wheel of radius R rolling over the hazard: the wheel centre height (minus R) is
    // the upper envelope of the surface seen through a circle (the wheel bridges short holes
    // and hits the far edge).
    double z = 0.0;
    for (const auto& h : m_hazards)
    {
        const double u = s - h.s;
        if (std::abs(u) > h.halfExtent() + R)
            continue;
        const double v = d - h.d;
        if (std::abs(v) > 0.5 * h.width)
            continue;

        if (h.type == HazardType::Pothole)
        {
            const double e = 2.0 * v / h.width;
            const double l = h.halfExtent() * std::sqrt(std::max(0.0, 1.0 - e * e));
            if (std::abs(u) >= l)
                continue;  // contact point still on intact asphalt

            // support from the hole bottom or from one of the two edges
            double best = -h.height;
            const double toNear = u + l;  // distance to the edge behind the wheel centre
            const double toFar = l - u;   // distance to the edge in front
            if (toNear < R)
                best = std::max(best, std::sqrt(R * R - toNear * toNear) - R);
            if (toFar < R)
                best = std::max(best, std::sqrt(R * R - toFar * toFar) - R);
            z += best;
        }
        else
        {
            // smooth humps: sample the envelope
            const int K = 24;
            double best = 0.0;
            for (int k = -K; k <= K; ++k)
            {
                const double du = R * k / K;
                const double hgt = h.profile(u + du, v) + std::sqrt(std::max(0.0, R * R - du * du)) - R;
                best = std::max(best, hgt);
            }
            z += best;
        }
    }
    return z;
}

double RoadNetwork::wheelInput(double s, double d, double tireRadius) const
{
    return hazardInput(s, d, tireRadius) + roughness(s, d);
}

double RoadNetwork::plannedSpeed(double s, double decel, double capMs, bool obeyLimits) const
{
    auto limitAt = [&](double q)
    {
        double v = capMs;
        if (obeyLimits)
            v = std::min(v, zoneAt(q).speedLimitKmh / 3.6);
        const double k = std::abs(m_course.curvature(q));
        if (k > 1e-6)
            v = std::min(v, std::sqrt(kAyMax / k));
        return v;
    };

    double v = limitAt(s);
    for (double ds = 4.0; ds <= 400.0; ds += 4.0)
        v = std::min(v, std::sqrt(sq(limitAt(s + ds)) + 2.0 * decel * ds));

    const double sStop = m_sEnd - 15.0;
    v = std::min(v, std::sqrt(2.0 * decel * std::max(0.0, sStop - s)));
    return v;
}

double RoadNetwork::cornerSpeed(double s0, double s1) const
{
    double v = 1e9;
    for (double q = s0; q <= s1; q += 1.0)
    {
        const double k = std::abs(m_course.curvature(q));
        if (k > 1e-6)
            v = std::min(v, std::sqrt(kAyMax / k));
    }
    return v;
}

bool RoadNetwork::project(double x, double y, double& s, double& d) const
{
    double best = 1e18;
    double bestS = 0.0;
    for (double q = 0.0; q <= m_sEnd; q += 2.0)
    {
        const Vec3 p = m_course.point(q, 0.0);
        const double dist = sq(p[0] - x) + sq(p[1] - y);
        if (dist < best)
        {
            best = dist;
            bestS = q;
        }
    }
    s = bestS;
    for (double q = bestS - 2.0; q <= bestS + 2.0; q += 0.05)
    {
        const Vec3 p = m_course.point(q, 0.0);
        const double dist = sq(p[0] - x) + sq(p[1] - y);
        if (dist < best)
        {
            best = dist;
            s = q;
        }
    }
    if (best >= 1e17)
        return false;

    const Pose2D p = m_course.pose(s);
    d = -(x - p.x) * std::sin(p.heading) + (y - p.y) * std::cos(p.heading);
    return true;
}

double RoadNetwork::surfaceHeight(double s, double d) const
{
    double z = 0.0;
    for (const auto& h : m_hazards)
    {
        const double u = s - h.s;
        if (std::abs(u) <= h.halfExtent())
            z += h.profile(u, d - h.d);
    }
    return z;
}

// ===========================================================================
// meshes: common
// ===========================================================================

vtkSmartPointer<vtkUnstructuredGrid> RoadNetwork::buildTerrain() const
{
    // well beyond the map bounds, so no sky shows below the horizon in overview shots
    MeshBuilder mb;
    mb.setColor(kGrass);
    const double z = -0.15;
    const double m = 1500.0;
    mb.addQuad({ m_bounds[0] - m, m_bounds[2] - m, z }, { m_bounds[1] + m, m_bounds[2] - m, z },
               { m_bounds[1] + m, m_bounds[3] + m, z }, { m_bounds[0] - m, m_bounds[3] + m, z });
    return mb.buildGrid();
}

vtkSmartPointer<vtkUnstructuredGrid> RoadNetwork::buildRoadSurface() const
{
    MeshBuilder mb;
    const double zm = 0.012;  // paint

    if (m_type == MapType::Highway)
    {
        // cross-section relative to the truck's lane centre (right-hand traffic, truck in the right lane)
        const double s0 = -260.0, s1 = m_sEnd + 200.0;
        const RoadPath& p = m_course;
        mb.setColor(kShoulder); ribbon(mb, p, s0, s1, -4.8, -1.8, 0.0);
        mb.setColor(kAsphalt);  ribbon(mb, p, s0, s1, -1.8, 5.4, 0.0);
        mb.setColor(kMedian);   ribbon(mb, p, s0, s1, 5.4, 7.4, 0.0);
        mb.setColor(kAsphalt);  ribbon(mb, p, s0, s1, 7.4, 14.6, 0.0);
        mb.setColor(kShoulder); ribbon(mb, p, s0, s1, 14.6, 17.6, 0.0);
        mb.setColor(kWhite);
        ribbon(mb, p, s0, s1, -1.9, -1.7, zm);
        dashed(mb, p, s0, s1, 1.8, 0.15, zm, 10.0, 10.0);
        dashed(mb, p, s0, s1, 11.0, 0.15, zm, 10.0, 10.0);
        ribbon(mb, p, s0, s1, 14.5, 14.7, zm);
        mb.setColor(kYellow);
        ribbon(mb, p, s0, s1, 5.3, 5.5, zm);
        ribbon(mb, p, s0, s1, 7.3, 7.5, zm);
    }
    else if (m_type == MapType::National)
    {
        // two lanes: truck lane [-1.75, 1.75], opposing lane [1.75, 5.25]
        const double s0 = -150.0, s1 = m_sEnd + 150.0;
        const RoadPath& p = m_course;
        mb.setColor(kShoulder); ribbon(mb, p, s0, s1, -2.75, -1.75, 0.0);
        mb.setColor(kAsphalt);  ribbon(mb, p, s0, s1, -1.75, 5.25, 0.0);
        mb.setColor(kShoulder); ribbon(mb, p, s0, s1, 5.25, 6.25, 0.0);
        mb.setColor(kWhite);
        ribbon(mb, p, s0, s1, -1.82, -1.68, zm);
        ribbon(mb, p, s0, s1, 5.18, 5.32, zm);
        mb.setColor(kYellow);  // double yellow centre line
        ribbon(mb, p, s0, s1, 1.57, 1.69, zm);
        ribbon(mb, p, s0, s1, 1.81, 1.93, zm);
    }
    else
    {
        // city: one asphalt sheet under the whole grid, blocks sit on top of it
        mb.setColor(kAsphalt);
        mb.addQuad({ m_cityBox[0], m_cityBox[2], 0.0 }, { m_cityBox[1], m_cityBox[2], 0.0 },
                   { m_cityBox[1], m_cityBox[3], 0.0 }, { m_cityBox[0], m_cityBox[3], 0.0 });
        // red paint in the school zone (all four lanes)
        mb.setColor(kSchoolRed);
        ribbon(mb, m_course, m_schoolS0, m_schoolS1, -1.6, 11.2, 0.004);
        buildCityMarkings(mb);
    }
    return mb.buildGrid();
}

vtkSmartPointer<vtkUnstructuredGrid> RoadNetwork::buildCurbs() const
{
    MeshBuilder mb;
    if (m_type == MapType::Highway)
    {
        mb.setColor(kConcrete);  // median barrier
        wall(mb, m_course, -260.0, m_sEnd + 200.0, 6.4, 0.55, 0.0, 0.85, 6.0);
    }
    else if (m_type == MapType::National)
    {
        mb.setColor(kSidewalk);  // village sidewalks (raised 15 cm)
        wall(mb, m_course, m_sVillage0, m_sVillage1, -3.75, 2.0, 0.0, kCurbHeight);
        wall(mb, m_course, m_sVillage0, m_sVillage1, 7.25, 2.0, 0.0, kCurbHeight);
    }
    else
    {
        buildCityBlocks(mb);
    }
    return mb.buildGrid();
}

vtkSmartPointer<vtkUnstructuredGrid> RoadNetwork::buildRoadside() const
{
    MeshBuilder mb;
    if (m_type == MapType::Highway)
    {
        const double s0 = -260.0, s1 = m_sEnd + 200.0;
        guardrail(mb, m_course, s0, s1, -5.1, +1.0);
        guardrail(mb, m_course, s0, s1, 17.9, -1.0);
        for (double s = -240.0; s < s1; s += 70.0)
            lampPole(mb, m_course, s, 6.4, 0.85, 10.0, 3.2, 3.2);
        gantry(mb, m_course, 140.0, -5.9, 6.4, -1.6, 4.4, kSignGreen);
        gantry(mb, m_course, 1500.0, -5.9, 6.4, -1.6, 4.4, kSignGreen);
        gantry(mb, m_course, m_sEnd - 400.0, -5.9, 6.4, -2.6, 3.4, kSignGreen);
    }
    else if (m_type == MapType::National)
    {
        gantry(mb, m_course, 40.0, -3.4, 6.6, -1.5, 3.5, kSignBlue);
        // concrete utility poles along the rural parts
        for (double s = 15.0; s < m_sEnd + 120.0; s += 45.0)
        {
            if (s > m_sVillage0 - 5.0 && s < m_sVillage1 + 5.0)
                continue;
            mb.setColor(kConcrete);
            mb.addCylinder(m_course.point(s, -3.6, 4.5), 'Z', 0.15, 9.0, 10);
            mb.setColor(kPole);
            pathBox(mb, m_course, s, -3.6, 0.08, 1.0, 8.2, 8.35);
        }
        // village street lamps on the sidewalk, arm reaching over the road
        for (double s = m_sVillage0 + 15.0; s < m_sVillage1; s += 35.0)
            lampPole(mb, m_course, s, -4.3, kCurbHeight, 8.0, 2.4, 0.0);
    }
    else
    {
        buildCityRoadside(mb);
    }
    return mb.buildGrid();
}

vtkSmartPointer<vtkUnstructuredGrid> RoadNetwork::buildScenery() const
{
    MeshBuilder mb;
    if (m_type == MapType::City)
    {
        buildCityScenery(mb);
        return mb.buildGrid();
    }

    std::mt19937 rng(m_type == MapType::Highway ? 7u : 11u);
    auto U = [&rng](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };

    // keep scenery off the road: circles (x, y, r) along the carriageway
    std::vector<std::array<double, 3>> blockers;
    const double roadCenter = (m_type == MapType::Highway) ? 6.4 : 1.75;
    const double roadHalf = (m_type == MapType::Highway) ? 13.5 : 6.0;
    for (double s = -260.0; s <= m_sEnd + 200.0; s += 6.0)
    {
        const Vec3 q = m_course.point(s, roadCenter);
        blockers.push_back({ q[0], q[1], roadHalf });
    }
    auto isFree = [&](const Vec3& q, double margin)
    {
        for (const auto& b : blockers)
        {
            if (sq(q[0] - b[0]) + sq(q[1] - b[1]) < sq(b[2] + margin))
                return false;
        }
        return true;
    };
    auto plant = [&](double s, double d)
    {
        const double r = U(1.5, 2.4);
        const Vec3 q = m_course.point(s, d);
        if (isFree(q, r))
            tree(mb, q[0], q[1], 0.0, r, U(3.5, 6.0), kLeaves[static_cast<int>(U(0.0, 3.99))]);
    };

    if (m_type == MapType::Highway)
    {
        for (double s = -240.0; s < m_sEnd + 180.0; s += 24.0)
        {
            plant(s + U(-5.0, 5.0), -U(10.0, 18.0));
            plant(s + U(-5.0, 5.0), U(23.0, 31.0));
        }
    }
    else
    {
        for (double s = -130.0; s < m_sEnd + 130.0; s += 18.0)
        {
            if (s > m_sVillage0 - 10.0 && s < m_sVillage1 + 10.0)
                continue;
            plant(s + U(-4.0, 4.0), -U(8.0, 14.0));
            plant(s + U(-4.0, 4.0), U(11.5, 17.0));
        }

        // village houses / shops behind the sidewalks
        const Rgb walls[] = { makeRgb(214, 196, 168), makeRgb(196, 120, 96), makeRgb(170, 184, 196),
                              makeRgb(226, 220, 206), makeRgb(150, 158, 130), makeRgb(204, 168, 120) };
        for (int side = 0; side < 2; ++side)
        {
            double s = m_sVillage0 + 8.0 + U(0.0, 6.0);
            while (s < m_sVillage1 - 10.0)
            {
                const double len = U(8.0, 14.0);
                const double depth = U(7.0, 11.0);
                const double height = U(3.5, 11.0);
                const double d = (side == 0) ? -(5.6 + 0.5 * depth) : (9.1 + 0.5 * depth);
                mb.setColor(walls[static_cast<int>(U(0.0, 5.99))]);
                pathBox(mb, m_course, s + 0.5 * len, d, 0.5 * len, 0.5 * depth, 0.0, height);
                mb.setColor(makeRgb(90, 70, 64));  // roof slab
                pathBox(mb, m_course, s + 0.5 * len, d, 0.5 * len + 0.3, 0.5 * depth + 0.3, height, height + 0.25);
                s += len + U(3.0, 9.0);
            }
        }
    }

    // farm fields
    const Rgb fields[] = { makeRgb(118, 162, 74), makeRgb(136, 172, 82), makeRgb(168, 146, 96), makeRgb(108, 150, 66) };
    const double dNear = (m_type == MapType::Highway) ? 36.0 : 26.0;
    int fieldIndex = 0;
    for (double s = 40.0; s < m_sEnd; s += 75.0)
    {
        const double side = (fieldIndex % 2 == 0) ? -1.0 : 1.0;
        const double d0 = side * dNear + (side > 0 && m_type == MapType::Highway ? 10.0 : 0.0);
        const double d1 = d0 + side * U(30.0, 60.0);
        const double sa = s;
        const double sb = s + U(50.0, 65.0);
        const bool village = (m_type == MapType::National) && sb > m_sVillage0 - 10.0 && sa < m_sVillage1 + 10.0;
        const Vec3 c[4] = { m_course.point(sa, d0), m_course.point(sb, d0), m_course.point(sb, d1), m_course.point(sa, d1) };
        bool ok = !village;
        for (const auto& q : c)
            ok = ok && isFree(q, 0.0);
        if (ok)
        {
            mb.setColor(fields[fieldIndex % 4]);
            ribbon(mb, m_course, sa, sb, std::min(d0, d1), std::max(d0, d1), -0.12, 15.0);
        }
        ++fieldIndex;
    }
    return mb.buildGrid();
}

vtkSmartPointer<vtkUnstructuredGrid> RoadNetwork::buildPotholes() const
{
    MeshBuilder mb;
    for (const auto& h : m_hazards)
    {
        if (h.type != HazardType::Pothole)
            continue;

        std::mt19937 rng(100u + static_cast<unsigned>(h.id));
        std::uniform_real_distribution<double> jitter(-0.12, 0.12);

        struct Layer { double scale; double dz; Rgb color; };
        const Layer layers[3] = {
            { 1.22, 0.006, makeRgb(116, 110, 102) },  // crumbled rim
            { 1.00, 0.009, makeRgb(34, 31, 29) },     // hole
            { 0.60, 0.011, makeRgb(18, 16, 16) },     // deepest part
        };
        for (const auto& layer : layers)
        {
            const int n = 26;
            std::vector<Vec3> ring;
            for (int k = 0; k < n; ++k)
            {
                const double t = 2.0 * kPi * k / n;
                const double r = layer.scale * (1.0 + jitter(rng));
                ring.push_back(m_course.point(h.s + 0.5 * h.length * r * std::cos(t),
                                              h.d + 0.5 * h.width * r * std::sin(t), layer.dz));
            }
            mb.setColor(layer.color);
            mb.addTriangleFan(m_course.point(h.s, h.d, layer.dz), ring);
        }
    }
    return mb.buildGrid();
}

vtkSmartPointer<vtkUnstructuredGrid> RoadNetwork::buildBumps() const
{
    MeshBuilder mb;
    const Rgb bumpYellow = makeRgb(244, 200, 40);
    const Rgb bumpWhite = makeRgb(240, 240, 236);
    const Rgb rubberBlack = makeRgb(40, 40, 42);
    const Rgb crossDark = makeRgb(84, 84, 88);

    for (const auto& h : m_hazards)
    {
        if (h.type == HazardType::Pothole)
            continue;

        const double z0 = 0.003;
        std::vector<double> us;
        if (h.type == HazardType::SpeedBump)
        {
            const int n = 14;
            for (int i = 0; i <= n; ++i)
                us.push_back(-0.5 * h.length + h.length * i / n);
        }
        else
        {
            const double top = 0.5 * h.length - h.ramp;
            us = { -0.5 * h.length, -top, top, 0.5 * h.length };
        }

        const int nv = std::max(2, static_cast<int>(std::round(h.width / 0.5)));
        auto P = [&](double u, double v)
        {
            return m_course.point(h.s + u, h.d + v, z0 + h.profile(u, 0.0));
        };

        const bool rubber = (h.kind == HazardKind::RubberHump);
        for (size_t i = 0; i + 1 < us.size(); ++i)
        {
            for (int j = 0; j < nv; ++j)
            {
                const double v0 = -0.5 * h.width + h.width * j / nv;
                const double v1 = -0.5 * h.width + h.width * (j + 1) / nv;
                const bool crossTop = (h.type == HazardType::RaisedCrosswalk && i == 1);
                if (crossTop)
                    mb.setColor((j % 2 == 0) ? kWhite : crossDark);  // zebra stripes on the plateau
                else if (rubber)
                    mb.setColor((j % 2 == 0) ? bumpYellow : rubberBlack);
                else
                    mb.setColor((j % 2 == 0) ? bumpYellow : bumpWhite);
                mb.addQuad(P(us[i], v0), P(us[i + 1], v0), P(us[i + 1], v1), P(us[i], v1));
            }
        }

        // side faces of the hump
        mb.setColor(rubber ? rubberBlack : bumpYellow);
        for (int side = 0; side < 2; ++side)
        {
            const double v = (side == 0) ? -0.5 * h.width : 0.5 * h.width;
            std::vector<Vec3> ring;
            for (double u : us)
                ring.push_back(P(u, v));
            mb.addTriangleFan(m_course.point(h.s, h.d + v, z0), ring);
        }
    }
    return mb.buildGrid();
}

// ===========================================================================
// meshes: city
// ===========================================================================

std::vector<RoadNetwork::Block> RoadNetwork::cityBlocks() const
{
    // intervals between the streets (and the outer edge of the city)
    auto intervals = [](double lo, double hi, const std::vector<double>& lines)
    {
        std::vector<std::pair<double, double>> out;
        double a = lo;
        for (double c : lines)
        {
            out.push_back({ a, c - kCityHw });
            a = c + kCityHw;
        }
        out.push_back({ a, hi });
        return out;
    };
    const auto xs = intervals(m_cityBox[0], m_cityBox[1], m_cityX);
    const auto ys = intervals(m_cityBox[2], m_cityBox[3], m_cityY);

    std::vector<Block> blocks;
    for (size_t ix = 0; ix < xs.size(); ++ix)
    {
        for (size_t iy = 0; iy < ys.size(); ++iy)
        {
            blocks.push_back({ xs[ix].first, ys[iy].first, xs[ix].second, ys[iy].second,
                               static_cast<int>(ix), static_cast<int>(iy) });
        }
    }
    return blocks;
}

void RoadNetwork::buildCityBlocks(MeshBuilder& mb) const
{
    // every block is a raised sidewalk island with rounded kerb corners
    for (const auto& b : cityBlocks())
        slab(mb, roundedRect(b.x0, b.y0, b.x1, b.y1, kCityCornerR, 0.0), 0.0, kCurbHeight, kSidewalk, kConcrete);
}

void RoadNetwork::buildCityMarkings(MeshBuilder& mb) const
{
    const double zm = 0.012;
    for (const auto& st : m_streets)
    {
        const RoadPath& p = st.path;
        const double L = p.length();

        // centre line and lane lines between the crossings
        std::vector<std::pair<double, double>> segments;
        double a = 0.0;
        for (double c : st.crossings)
        {
            segments.push_back({ a, c - kCityHw });
            a = c + kCityHw;
        }
        segments.push_back({ a, L });
        for (const auto& [sa, sb] : segments)
        {
            const double a0 = (sa > 0.0) ? sa + 7.0 : sa;
            const double b0 = (sb < L) ? sb - 7.0 : sb;
            mb.setColor(kYellow);
            ribbon(mb, p, a0, b0, -0.18, -0.06, zm, 8.0);
            ribbon(mb, p, a0, b0, 0.06, 0.18, zm, 8.0);
            mb.setColor(kWhite);
            dashed(mb, p, a0, b0, -3.2, 0.12, zm, 3.0, 5.0);
            dashed(mb, p, a0, b0, 3.2, 0.12, zm, 3.0, 5.0);
        }

        // zebra crossings and stop lines on both sides of every intersection
        for (double c : st.crossings)
        {
            for (int side = -1; side <= 1; side += 2)
            {
                const double e = c + side * kCityHw;               // edge of the intersection
                const double z0 = e + side * 1.5;
                const double z1 = e + side * 5.5;
                mb.setColor(kWhite);
                for (double v = -6.2; v < 6.2; v += 1.0)
                    ribbon(mb, p, std::min(z0, z1), std::max(z0, z1), v, v + 0.5, zm, 10.0);
                // stop line for the traffic approaching this side of the crossing
                const double l0 = e + side * 6.1;
                const double l1 = e + side * 6.6;
                if (side < 0)
                    ribbon(mb, p, std::min(l0, l1), std::max(l0, l1), -6.3, -0.25, zm, 10.0);
                else
                    ribbon(mb, p, std::min(l0, l1), std::max(l0, l1), 0.25, 6.3, zm, 10.0);
            }
        }
    }
}

void RoadNetwork::buildCityRoadside(MeshBuilder& mb) const
{
    // traffic signals at two diagonal corners of every intersection
    const double inset = kCityHw + kCityCornerR - 7.5 / std::sqrt(2.0);  // on the rounded sidewalk corner
    for (double x : m_cityX)
    {
        for (double y : m_cityY)
        {
            for (int corner = 0; corner < 2; ++corner)
            {
                const double sgn = (corner == 0) ? -1.0 : 1.0;
                const double px = x + sgn * inset;
                const double py = y + sgn * inset;
                mb.setColor(makeRgb(60, 62, 66));
                mb.addBox({ px - 0.12, py - 0.12, kCurbHeight }, { px + 0.12, py + 0.12, 6.0 });
                // arm over the approach lanes (towards the street centre line)
                const double yEnd = y - sgn * 1.0;
                mb.addBox({ px - 0.08, std::min(py, yEnd), 5.6 }, { px + 0.08, std::max(py, yEnd), 5.78 });
                // signal head with a lit lamp
                const double hy = 0.5 * (py + yEnd);
                mb.setColor(makeRgb(30, 30, 32));
                mb.addBox({ px - 0.2, hy - 0.65, 5.05 }, { px + 0.2, hy + 0.65, 5.55 });
                mb.setColor(corner == 0 ? makeRgb(60, 230, 110) : makeRgb(240, 60, 50));
                mb.addBox({ px - 0.24, hy - 0.18, 5.15 }, { px + 0.24, hy + 0.18, 5.45 });
            }
        }
    }

    // street lamps on both sidewalks, staggered. Every lamp is checked on its own: never inside a
    // crossing and never on the rounded block corners (there the kerb curves away and the lamp
    // would stand on the asphalt).
    const double clearCrossing = kCityHw + kCityCornerR + 4.0;
    const double clearEnd = kCityCornerR + 3.0;
    for (const auto& st : m_streets)
    {
        const double L = st.path.length();
        auto onStraightSidewalk = [&](double s)
        {
            if (s < clearEnd || s > L - clearEnd)
                return false;
            for (double c : st.crossings)
            {
                if (std::abs(s - c) < clearCrossing)
                    return false;
            }
            return true;
        };
        for (double s = 18.0; s < L; s += 35.0)
        {
            if (onStraightSidewalk(s))
                lampPole(mb, st.path, s, -(kCityHw + 0.9), kCurbHeight, 8.0, 2.2, 0.0);
            if (onStraightSidewalk(s + 17.0))
                lampPole(mb, st.path, s + 17.0, kCityHw + 0.9, kCurbHeight, 8.0, 0.0, 2.2);
        }
    }
}

void RoadNetwork::buildCityScenery(MeshBuilder& mb) const
{
    std::mt19937 rng(23u);
    auto U = [&rng](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };
    const Rgb facades[] = { makeRgb(150, 160, 172), makeRgb(182, 176, 166), makeRgb(120, 132, 150),
                            makeRgb(200, 190, 170), makeRgb(164, 118, 96),  makeRgb(96, 110, 130),
                            makeRgb(210, 206, 198), makeRgb(140, 150, 140) };
    const double zb = kCurbHeight;
    const double setback = kCitySidewalk + 1.0;

    for (const auto& b : cityBlocks())
    {
        const double x0 = b.x0 + setback, x1 = b.x1 - setback;
        const double y0 = b.y0 + setback, y1 = b.y1 - setback;

        // park in the middle of the north side of the loop
        if (b.ix == 2 && b.iy == 2)
        {
            slab(mb, roundedRect(x0, y0, x1, y1, 6.0, zb), zb, zb + 0.05, kLawn, kLawn);
            for (int k = 0; k < 45; ++k)
            {
                const double tx = U(x0 + 4.0, x1 - 4.0);
                const double ty = U(y0 + 4.0, y1 - 4.0);
                tree(mb, tx, ty, zb + 0.05, U(1.8, 3.0), U(4.0, 7.0), kLeaves[k % 4]);
            }
            continue;
        }

        // school facing the school-zone street, with its sports ground
        if (b.ix == 2 && b.iy == 1)
        {
            mb.setColor(makeRgb(232, 224, 200));
            mb.addBox({ x0 + 10.0, y0 + 2.0, zb }, { x1 - 10.0, y0 + 20.0, 15.0 });
            mb.setColor(makeRgb(120, 70, 60));
            mb.addBox({ x0 + 9.5, y0 + 1.5, 15.0 }, { x1 - 9.5, y0 + 20.5, 15.4 });
            mb.setColor(makeRgb(178, 140, 98));
            mb.addQuad({ x0 + 6.0, y0 + 30.0, zb + 0.02 }, { x1 - 6.0, y0 + 30.0, zb + 0.02 },
                       { x1 - 6.0, y1 - 6.0, zb + 0.02 }, { x0 + 6.0, y1 - 6.0, zb + 0.02 });
            continue;
        }

        // ordinary block: buildings around the perimeter, courtyard inside
        const int nx = std::max(1, static_cast<int>(std::round((x1 - x0) / 30.0)));
        const int ny = std::max(1, static_cast<int>(std::round((y1 - y0) / 30.0)));
        const double cw = (x1 - x0) / nx;
        const double ch = (y1 - y0) / ny;
        for (int i = 0; i < nx; ++i)
        {
            for (int j = 0; j < ny; ++j)
            {
                if (nx > 2 && ny > 2 && i > 0 && i < nx - 1 && j > 0 && j < ny - 1)
                    continue;  // courtyard
                const double bx0 = x0 + i * cw + 1.5;
                const double by0 = y0 + j * ch + 1.5;
                const double bx1 = x0 + (i + 1) * cw - 1.5;
                const double by1 = y0 + (j + 1) * ch - 1.5;
                const double height = U(9.0, 46.0);
                mb.setColor(facades[static_cast<int>(U(0.0, 7.99))]);
                mb.addBox({ bx0, by0, zb }, { bx1, by1, height });
                mb.setColor(makeRgb(76, 78, 84));  // roof
                mb.addBox({ bx0 + 0.4, by0 + 0.4, height }, { bx1 - 0.4, by1 - 0.4, height + 0.6 });
            }
        }
    }
}
