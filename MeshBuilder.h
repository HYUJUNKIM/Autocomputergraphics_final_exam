#pragma once

#include <array>
#include <vector>

#include <vtkSmartPointer.h>
#include <vtkType.h>

class vtkPolyData;
class vtkUnstructuredGrid;

using Vec3 = std::array<double, 3>;
using Rgb  = std::array<unsigned char, 3>;

constexpr Rgb makeRgb(int r, int g, int b)
{
    return Rgb{ static_cast<unsigned char>(r), static_cast<unsigned char>(g), static_cast<unsigned char>(b) };
}

// Accumulates points and cells and turns them into a vtkUnstructuredGrid / vtkPolyData.
// Every cell carries two cell-data values:
//   "Color"  : unsigned char RGB  (used by PIDs in DirectRgb mode)
//   "Scalar" : double             (used by PIDs mapped through a lookup table)
class MeshBuilder
{
public:
    void setColor(const Rgb& c) { m_color = c; }
    void setScalar(double s) { m_scalar = s; }

    vtkIdType addPoint(const Vec3& p);
    vtkIdType numberOfPoints() const { return static_cast<vtkIdType>(m_points.size()); }
    vtkIdType numberOfCells() const { return static_cast<vtkIdType>(m_types.size()); }
    const std::vector<Vec3>& points() const { return m_points; }

    void addCell(int cellType, const std::vector<vtkIdType>& ids);

    void addTriangle(const Vec3& a, const Vec3& b, const Vec3& c);
    void addQuad(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d);
    void addTriangleFan(const Vec3& center, const std::vector<Vec3>& ring);

    // Hexahedron in VTK order: bottom face counter-clockwise seen from +z, then top face.
    // asFaces = true emits the 6 faces as quads (needed for vtkPolyData output).
    void addHex(const std::array<Vec3, 8>& p, bool asFaces = false);
    void addBox(const Vec3& mn, const Vec3& mx, bool asFaces = false);

    // Closed cylinder centred at 'center' with its axis along 'X', 'Y' or 'Z'.
    void addCylinder(const Vec3& center, char axis, double radius, double length, int resolution);
    // Cone standing on the xy-plane at baseCenter, apex pointing +z.
    void addCone(const Vec3& baseCenter, double radius, double height, int resolution);

    vtkSmartPointer<vtkUnstructuredGrid> buildGrid() const;
    // Only 2D cells (triangle / quad / polygon) are emitted.
    vtkSmartPointer<vtkPolyData> buildPolyData() const;

private:
    std::vector<Vec3> m_points;
    std::vector<int> m_types;
    std::vector<vtkIdType> m_offsets{ 0 };
    std::vector<vtkIdType> m_connectivity;
    std::vector<Rgb> m_colors;
    std::vector<double> m_scalars;

    Rgb m_color = makeRgb(200, 200, 200);
    double m_scalar = 0.0;
};
