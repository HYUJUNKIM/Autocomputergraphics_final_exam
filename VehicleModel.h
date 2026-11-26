#pragma once

#include <array>
#include <string>
#include <vector>

// One wheel station (tyre + unsprung mass + suspension).
struct WheelDef
{
    std::string name;
    int axle = 0;
    int side = 1;                  // +1 left, -1 right
    double x = 0.0;                // longitudinal position [m]; vehicle frame: origin = front axle, +x forward
    double y = 0.0;                // lateral position [m], + = left
    double radius = 0.48;          // [m]
    double width = 0.30;           // visual width [m]
    bool dual = false;             // twin tyres
    double unsprungMass = 400.0;   // [kg]
    double tireStiffness = 1.0e6;  // [N/m]
    double springStiffness = 3e5;  // wheel rate [N/m]
    double damping = 1.5e4;        // [N s/m]
    double travelCompression = 0.09;  // free travel before the bump stop [m]
    double travelRebound = 0.10;      // free travel before the rebound stop [m]
};

enum class VehicleType { Compact = 0, Sedan = 1, SUV = 2, Truck = 3 };

// Everything that differs between the selectable vehicles.
struct VehicleSpec
{
    VehicleType type = VehicleType::Truck;
    std::string name;
    std::vector<WheelDef> wheels;   // 4 (cars) or 6 (truck), ordered axle by axle: left, right
    int axleCount = 2;

    // sprung body without payload
    double sprungMass = 1500.0;
    double cgX = -1.2, cgZ = 0.55;
    double kyBody = 1.2, kxBody = 0.55;  // radii of gyration (pitch, roll) [m]

    // payload (people + luggage, or cargo)
    double payloadDefault = 150.0;
    double payloadMax = 500.0;
    double payloadStep = 50.0;
    double payloadCgX = -1.5, payloadCgZ = 0.7;
    double kyPayload = 0.8, kxPayload = 0.5;

    double pitchCentreZ = 0.25;
    double rollCentreZ = 0.30;
    double bumpStopStiffness = 2.5e5;  // [N/m]
    double maxSpeedKmh = 120.0;        // legal / practical top speed of this vehicle class

    // outer dimensions for the visuals and cameras (vehicle frame)
    double frontX = 0.9;    // front bumper
    double rearX = -3.9;    // rear bumper
    double width = 1.8;
    double height = 1.45;
    std::array<double, 3> cameraMount{ 0.0, 0.0, 1.4 };  // dash-cam lens position
};

// Vertical multi-body model of a 2- or 3-axle vehicle:
//   sprung body: heave z, pitch slope px (= dz/dx), roll slope py (= dz/dy)
//   one unsprung mass per wheel: vertical displacement zu
// All displacements are deviations from the static equilibrium. Tyres can lift off
// (no tension), suspensions hit progressive bump / rebound stops.
class VehicleModel
{
public:
    static constexpr int kMaxWheels = 6;
    static constexpr double kGravity = 9.81;
    using WheelArray = std::array<double, kMaxWheels>;

    static VehicleSpec makeSpec(VehicleType type);
    static const char* typeName(VehicleType type);

    explicit VehicleModel(VehicleType type = VehicleType::Truck);

    void setType(VehicleType type);
    VehicleType type() const { return m_spec.type; }
    const VehicleSpec& spec() const { return m_spec; }
    int wheelCount() const { return static_cast<int>(m_spec.wheels.size()); }
    int axleCount() const { return m_spec.axleCount; }

    void setPayload(double kg);
    double payload() const { return m_payload; }
    double sprungMass() const { return m_mass; }
    double totalMass() const;
    double cgX() const { return m_cgX; }
    double cgZ() const { return m_cgZ; }

    void resetDynamics();

    // roadZ: vertical road input under every wheel [m]; ax, ay: longitudinal / lateral acceleration [m/s^2]
    void step(double dt, const WheelArray& roadZ, double ax, double ay);

    const WheelDef& wheel(int i) const { return m_spec.wheels[i]; }
    double tireForce(int i) const { return m_ft[i]; }           // total vertical tyre force [N]
    double staticTireForce(int i) const { return m_ft0[i]; }    // [N]
    double loadRatio(int i) const { return m_ft[i] / m_ft0[i]; }  // dynamic load coefficient
    double suspensionDeflection(int i) const { return m_defl[i]; }  // + = compression [m]
    double wheelZ(int i) const { return m_zu[i]; }
    double roadInput(int i) const { return m_zr[i]; }
    double minStaticTireForce() const;

    double heave() const { return m_z; }
    double pitchSlope() const { return m_px; }
    double rollSlope() const { return m_py; }

    // least-squares plane z = z0 + px*(x - cgX) + py*y through the current road inputs
    void roadPlane(double& z0, double& px, double& py) const;

private:
    void computeMassProperties();
    void computeStatics();

    VehicleSpec m_spec;
    double m_payload = 0.0;

    double m_mass = 0.0;
    double m_cgX = 0.0;
    double m_cgZ = 0.0;
    double m_iPitch = 0.0;
    double m_iRoll = 0.0;

    WheelArray m_a{};    // wheel x relative to CG
    WheelArray m_b{};    // wheel y
    WheelArray m_fs0{};  // static spring force
    WheelArray m_ft0{};  // static tyre force

    // state
    double m_z = 0.0, m_zd = 0.0;
    double m_px = 0.0, m_pxd = 0.0;
    double m_py = 0.0, m_pyd = 0.0;
    WheelArray m_zu{};
    WheelArray m_zud{};

    // outputs
    WheelArray m_ft{};
    WheelArray m_defl{};
    WheelArray m_zr{};
};
