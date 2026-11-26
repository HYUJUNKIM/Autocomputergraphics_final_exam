#include "VehicleModel.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace
{
// Gaussian elimination with partial pivoting for a 3x3 system.
bool solve3(const double A[3][3], const double b[3], double x[3])
{
    double M[3][4];
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
            M[i][j] = A[i][j];
        M[i][3] = b[i];
    }

    for (int c = 0; c < 3; ++c)
    {
        int piv = c;
        for (int r = c + 1; r < 3; ++r)
        {
            if (std::abs(M[r][c]) > std::abs(M[piv][c]))
                piv = r;
        }
        if (std::abs(M[piv][c]) < 1e-12)
            return false;
        if (piv != c)
        {
            for (int k = 0; k < 4; ++k)
                std::swap(M[c][k], M[piv][k]);
        }
        for (int r = 0; r < 3; ++r)
        {
            if (r == c)
                continue;
            const double f = M[r][c] / M[c][c];
            for (int k = c; k < 4; ++k)
                M[r][k] -= f * M[c][k];
        }
    }

    for (int i = 0; i < 3; ++i)
        x[i] = M[i][3] / M[i][i];
    return true;
}

// passenger car corner (independent suspension)
WheelDef carWheel(const char* name, int axle, int side, double x, double track, double radius, double width,
                  double mu, double kt, double ks, double cs, double travel)
{
    WheelDef w;
    w.name = name;
    w.axle = axle;
    w.side = side;
    w.x = x;
    w.y = side * 0.5 * track;
    w.radius = radius;
    w.width = width;
    w.dual = false;
    w.unsprungMass = mu;
    w.tireStiffness = kt;
    w.springStiffness = ks;
    w.damping = cs;
    w.travelCompression = travel;
    w.travelRebound = travel + 0.01;
    return w;
}

WheelDef truckWheel(const char* name, int axle, int side, double x, double y, bool dual)
{
    WheelDef w;
    w.name = name;
    w.axle = axle;
    w.side = side;
    w.x = x;
    w.y = y;
    w.radius = 0.48;
    w.dual = dual;
    if (!dual)
    {
        // steer axle, single tyre
        w.width = 0.30;
        w.unsprungMass = 380.0;
        w.tireStiffness = 0.95e6;
        w.springStiffness = 320e3;
        w.damping = 14e3;
        w.travelCompression = 0.09;
        w.travelRebound = 0.10;
    }
    else
    {
        // tandem drive axles, twin tyres
        w.width = 0.56;
        w.unsprungMass = 550.0;
        w.tireStiffness = 1.9e6;
        w.springStiffness = 480e3;
        w.damping = 20e3;
        w.travelCompression = 0.10;
        w.travelRebound = 0.10;
    }
    return w;
}
} // namespace

const char* VehicleModel::typeName(VehicleType type)
{
    switch (type)
    {
    case VehicleType::Compact: return "Compact car";
    case VehicleType::Sedan: return "Sedan";
    case VehicleType::SUV: return "SUV";
    case VehicleType::Truck: return "Truck";
    }
    return "";
}

VehicleSpec VehicleModel::makeSpec(VehicleType type)
{
    VehicleSpec s;
    s.type = type;
    s.name = typeName(type);

    switch (type)
    {
    case VehicleType::Compact:
    {
        // ~1.0 t city car (light car class), wheelbase 2.4 m
        const double wb = 2.40, track = 1.40, r = 0.29, w = 0.165;
        const double mu = 28.0, kt = 170e3, ks = 16.5e3, cs = 1.15e3, travel = 0.07;
        s.wheels = { carWheel("FL", 0, +1, 0.0, track, r, w, mu, kt, ks, cs, travel),
                     carWheel("FR", 0, -1, 0.0, track, r, w, mu, kt, ks, cs, travel),
                     carWheel("RL", 1, +1, -wb, track, r, w, mu, kt, 0.9 * ks, cs, travel),
                     carWheel("RR", 1, -1, -wb, track, r, w, mu, kt, 0.9 * ks, cs, travel) };
        s.axleCount = 2;
        s.sprungMass = 860.0;  s.cgX = -0.96; s.cgZ = 0.56;
        s.kyBody = 1.05; s.kxBody = 0.50;
        s.payloadDefault = 150.0; s.payloadMax = 400.0; s.payloadStep = 25.0;
        s.payloadCgX = -1.30; s.payloadCgZ = 0.70; s.kyPayload = 0.7; s.kxPayload = 0.45;
        s.pitchCentreZ = 0.20; s.rollCentreZ = 0.25;
        s.bumpStopStiffness = 1.7e5;
        s.maxSpeedKmh = 130.0;
        s.frontX = 0.62; s.rearX = -3.0; s.width = 1.60; s.height = 1.49;
        s.cameraMount = { -0.70, 0.0, 1.54 };
        break;
    }
    case VehicleType::Sedan:
    {
        // ~1.65 t mid-size sedan, wheelbase 2.84 m
        const double wb = 2.84, track = 1.60, r = 0.335, w = 0.215;
        const double mu = 40.0, kt = 220e3, ks = 23.5e3, cs = 1.75e3, travel = 0.08;
        s.wheels = { carWheel("FL", 0, +1, 0.0, track, r, w, mu, kt, ks, cs, travel),
                     carWheel("FR", 0, -1, 0.0, track, r, w, mu, kt, ks, cs, travel),
                     carWheel("RL", 1, +1, -wb, track, r, w, mu, kt, 0.95 * ks, cs, travel),
                     carWheel("RR", 1, -1, -wb, track, r, w, mu, kt, 0.95 * ks, cs, travel) };
        s.axleCount = 2;
        s.sprungMass = 1420.0; s.cgX = -1.19; s.cgZ = 0.55;
        s.kyBody = 1.25; s.kxBody = 0.55;
        s.payloadDefault = 150.0; s.payloadMax = 500.0; s.payloadStep = 25.0;
        s.payloadCgX = -1.55; s.payloadCgZ = 0.68; s.kyPayload = 0.9; s.kxPayload = 0.5;
        s.pitchCentreZ = 0.20; s.rollCentreZ = 0.25;
        s.bumpStopStiffness = 2.4e5;
        s.maxSpeedKmh = 130.0;
        s.frontX = 0.95; s.rearX = -3.95; s.width = 1.86; s.height = 1.45;
        s.cameraMount = { -0.90, 0.0, 1.50 };
        break;
    }
    case VehicleType::SUV:
    {
        // ~2.1 t mid-size SUV, wheelbase 2.8 m, high centre of gravity
        const double wb = 2.80, track = 1.65, r = 0.37, w = 0.235;
        const double mu = 50.0, kt = 260e3, ks = 34.5e3, cs = 2.4e3, travel = 0.10;
        s.wheels = { carWheel("FL", 0, +1, 0.0, track, r, w, mu, kt, ks, cs, travel),
                     carWheel("FR", 0, -1, 0.0, track, r, w, mu, kt, ks, cs, travel),
                     carWheel("RL", 1, +1, -wb, track, r, w, mu, kt, ks, cs, travel),
                     carWheel("RR", 1, -1, -wb, track, r, w, mu, kt, ks, cs, travel) };
        s.axleCount = 2;
        s.sprungMass = 1800.0; s.cgX = -1.26; s.cgZ = 0.72;
        s.kyBody = 1.30; s.kxBody = 0.60;
        s.payloadDefault = 150.0; s.payloadMax = 600.0; s.payloadStep = 25.0;
        s.payloadCgX = -1.70; s.payloadCgZ = 0.90; s.kyPayload = 1.0; s.kxPayload = 0.55;
        s.pitchCentreZ = 0.30; s.rollCentreZ = 0.35;
        s.bumpStopStiffness = 3.5e5;
        s.maxSpeedKmh = 130.0;
        s.frontX = 0.95; s.rearX = -3.90; s.width = 1.90; s.height = 1.72;
        s.cameraMount = { -1.00, 0.0, 1.77 };
        break;
    }
    case VehicleType::Truck:
    {
        // 3-axle cab-over truck: steer axle at x = 0, tandem axles at x = -3.85 / -5.15
        s.wheels = { truckWheel("FL",  0, +1,  0.00,  0.95, false),
                     truckWheel("FR",  0, -1,  0.00, -0.95, false),
                     truckWheel("R1L", 1, +1, -3.85,  0.85, true),
                     truckWheel("R1R", 1, -1, -3.85, -0.85, true),
                     truckWheel("R2L", 2, +1, -5.15,  0.85, true),
                     truckWheel("R2R", 2, -1, -5.15, -0.85, true) };
        s.axleCount = 3;
        s.sprungMass = 6800.0; s.cgX = -1.7; s.cgZ = 1.15;   // cab + chassis + empty body
        s.kyBody = 1.7; s.kxBody = 0.65;
        s.payloadDefault = 10000.0; s.payloadMax = 16000.0; s.payloadStep = 500.0;
        s.payloadCgX = -3.65; s.payloadCgZ = 1.80; s.kyPayload = 1.56; s.kxPayload = 0.83;
        s.pitchCentreZ = 0.6; s.rollCentreZ = 0.8;
        s.bumpStopStiffness = 4.0e6;
        s.maxSpeedKmh = 90.0;   // truck limit on Korean motorways
        s.frontX = 1.36; s.rearX = -6.35; s.width = 2.40; s.height = 3.40;
        s.cameraMount = { 1.32, 0.0, 3.15 };
        break;
    }
    }
    return s;
}

VehicleModel::VehicleModel(VehicleType type)
{
    setType(type);
}

void VehicleModel::setType(VehicleType type)
{
    m_spec = makeSpec(type);
    setPayload(m_spec.payloadDefault);
}

double VehicleModel::totalMass() const
{
    double m = m_mass;
    for (const auto& w : m_spec.wheels)
        m += w.unsprungMass;
    return m;
}

double VehicleModel::minStaticTireForce() const
{
    double f = 1e18;
    for (int i = 0; i < wheelCount(); ++i)
        f = std::min(f, m_ft0[i]);
    return f;
}

void VehicleModel::setPayload(double kg)
{
    m_payload = std::clamp(kg, 0.0, m_spec.payloadMax);
    computeMassProperties();
    computeStatics();
    resetDynamics();
}

void VehicleModel::computeMassProperties()
{
    const VehicleSpec& s = m_spec;
    const double mc = s.sprungMass;
    const double mp = m_payload;
    m_mass = mc + mp;
    m_cgX = (mc * s.cgX + mp * s.payloadCgX) / m_mass;
    m_cgZ = (mc * s.cgZ + mp * s.payloadCgZ) / m_mass;

    m_iPitch = mc * (s.kyBody * s.kyBody + std::pow(s.cgX - m_cgX, 2) + std::pow(s.cgZ - m_cgZ, 2))
             + mp * (s.kyPayload * s.kyPayload + std::pow(s.payloadCgX - m_cgX, 2) + std::pow(s.payloadCgZ - m_cgZ, 2));
    m_iRoll = mc * (s.kxBody * s.kxBody + std::pow(s.cgZ - m_cgZ, 2))
            + mp * (s.kxPayload * s.kxPayload + std::pow(s.payloadCgZ - m_cgZ, 2));

    m_a.fill(0.0);
    m_b.fill(0.0);
    for (int i = 0; i < wheelCount(); ++i)
    {
        m_a[i] = s.wheels[i].x - m_cgX;
        m_b[i] = s.wheels[i].y;
    }
}

void VehicleModel::computeStatics()
{
    // Spring and tyre in series. With body deflection db_i = z + a_i px + b_i py (from the
    // unloaded state) the spring force is  Fs_i = -keff_i * db_i - c_i,
    //   keff = ks*kt/(ks+kt),  c = ks * mu * g / (ks + kt)  (unsprung weight on the tyre).
    // Body equilibrium: sum Fs = M g, sum a Fs = 0, sum b Fs = 0  ->  3x3 system.
    double A[3][3] = { { 0.0 } };
    double rhs[3] = { -m_mass * kGravity, 0.0, 0.0 };
    WheelArray keff{}, cst{};

    for (int i = 0; i < wheelCount(); ++i)
    {
        const WheelDef& w = m_spec.wheels[i];
        keff[i] = w.springStiffness * w.tireStiffness / (w.springStiffness + w.tireStiffness);
        cst[i] = w.springStiffness * w.unsprungMass * kGravity / (w.springStiffness + w.tireStiffness);

        const double g[3] = { 1.0, m_a[i], m_b[i] };
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
                A[r][c] += keff[i] * g[r] * g[c];
            rhs[r] -= g[r] * cst[i];
        }
    }

    double q[3] = { 0.0, 0.0, 0.0 };
    solve3(A, rhs, q);

    m_fs0.fill(0.0);
    m_ft0.fill(1.0);
    for (int i = 0; i < wheelCount(); ++i)
    {
        const double db = q[0] + m_a[i] * q[1] + m_b[i] * q[2];
        m_fs0[i] = -keff[i] * db - cst[i];
        m_ft0[i] = m_fs0[i] + m_spec.wheels[i].unsprungMass * kGravity;
    }
}

void VehicleModel::resetDynamics()
{
    m_z = m_zd = 0.0;
    m_px = m_pxd = 0.0;
    m_py = m_pyd = 0.0;
    m_zu.fill(0.0);
    m_zud.fill(0.0);
    m_defl.fill(0.0);
    m_zr.fill(0.0);
    m_ft = m_ft0;
}

void VehicleModel::step(double dt, const WheelArray& roadZ, double ax, double ay)
{
    double sumF = 0.0, sumA = 0.0, sumB = 0.0;
    WheelArray zudd{};
    const double kBump = m_spec.bumpStopStiffness;

    for (int i = 0; i < wheelCount(); ++i)
    {
        const WheelDef& w = m_spec.wheels[i];

        const double zb = m_z + m_a[i] * m_px + m_b[i] * m_py;
        const double zbd = m_zd + m_a[i] * m_pxd + m_b[i] * m_pyd;

        // suspension (dynamic part), + pushes the body up
        const double defl = m_zu[i] - zb;
        double fs = w.springStiffness * defl + w.damping * (m_zud[i] - zbd);
        if (defl > w.travelCompression)
            fs += kBump * (defl - w.travelCompression);
        else if (defl < -w.travelRebound)
            fs += kBump * (defl + w.travelRebound);

        // tyre: point follower on the enveloped road, no tension (lift-off)
        double ft = m_ft0[i] + w.tireStiffness * (roadZ[i] - m_zu[i]);
        if (ft < 0.0)
            ft = 0.0;

        zudd[i] = (ft - m_ft0[i] - fs) / w.unsprungMass;

        m_ft[i] = ft;
        m_defl[i] = defl;
        m_zr[i] = roadZ[i];

        sumF += fs;
        sumA += m_a[i] * fs;
        sumB += m_b[i] * fs;
    }

    // inertial load transfer: braking pitches the nose down, a left turn rolls the body right
    const double zdd = sumF / m_mass;
    const double pxdd = (sumA + m_mass * ax * (m_cgZ - m_spec.pitchCentreZ)) / m_iPitch;
    const double pydd = (sumB + m_mass * ay * (m_cgZ - m_spec.rollCentreZ)) / m_iRoll;

    // semi-implicit Euler
    m_zd += zdd * dt;
    m_pxd += pxdd * dt;
    m_pyd += pydd * dt;
    m_z += m_zd * dt;
    m_px += m_pxd * dt;
    m_py += m_pyd * dt;

    for (int i = 0; i < wheelCount(); ++i)
    {
        m_zud[i] += zudd[i] * dt;
        m_zu[i] += m_zud[i] * dt;
    }
}

void VehicleModel::roadPlane(double& z0, double& px, double& py) const
{
    double A[3][3] = { { 0.0 } };
    double rhs[3] = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < wheelCount(); ++i)
    {
        const double g[3] = { 1.0, m_a[i], m_b[i] };
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
                A[r][c] += g[r] * g[c];
            rhs[r] += g[r] * m_zr[i];
        }
    }
    double q[3] = { 0.0, 0.0, 0.0 };
    solve3(A, rhs, q);
    z0 = q[0];
    px = q[1];
    py = q[2];
}
