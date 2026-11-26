#include "MeshBuilder.h"

#include <cmath>

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCellType.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnstructuredGrid.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

bool is2DCell(int type)
{
    return type == VTK_TRIANGLE || type == VTK_QUAD || type == VTK_POLYGON;
}

vtkSmartPointer<vtkPoints> makePoints(const std::vector<Vec3>& pts)
{
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->SetDataTypeToDouble();
    points->SetNumberOfPoints(static_cast<vtkIdType>(pts.size()));
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(pts.size()); ++i)
    {
        points->SetPoint(i, pts[i].data());
    }
    return points;
}

void attachArrays(vtkDataSet* ds, const std::vector<Rgb>& colors, const std::vector<double>& scalars)
{
    auto colorArray = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colorArray->SetName("Color");
    colorArray->SetNumberOfComponents(3);
    colorArray->SetNumberOfTuples(static_cast<vtkIdType>(colors.size()));
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(colors.size()); ++i)
    {
        colorArray->SetTypedTuple(i, colors[i].data());
    }

    auto scalarArray = vtkSmartPointer<vtkDoubleArray>::New();
    scalarArray->SetName("Scalar");
    scalarArray->SetNumberOfComponents(1);
    scalarArray->SetNumberOfTuples(static_cast<vtkIdType>(scalars.size()));
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(scalars.size()); ++i)
    {
        scalarArray->SetValue(i, scalars[i]);
    }

    ds->GetCellData()->AddArray(colorArray);
    ds->GetCellData()->AddArray(scalarArray);
}
} // namespace

vtkIdType MeshBuilder::addPoint(const Vec3& p)
{
    m_points.push_back(p);
    return static_cast<vtkIdType>(m_points.size()) - 1;
}

void MeshBuilder::addCell(int cellType, const std::vector<vtkIdType>& ids)
{
    m_types.push_back(cellType);
    m_connectivity.insert(m_connectivity.end(), ids.begin(), ids.end());
    m_offsets.push_back(static_cast<vtkIdType>(m_connectivity.size()));
    m_colors.push_back(m_color);
    m_scalars.push_back(m_scalar);
}

void MeshBuilder::addTriangle(const Vec3& a, const Vec3& b, const Vec3& c)
{
    const vtkIdType ia = addPoint(a);
    const vtkIdType ib = addPoint(b);
    const vtkIdType ic = addPoint(c);
    addCell(VTK_TRIANGLE, { ia, ib, ic });
}

void MeshBuilder::addQuad(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d)
{
    const vtkIdType ia = addPoint(a);
    const vtkIdType ib = addPoint(b);
    const vtkIdType ic = addPoint(c);
    const vtkIdType id = addPoint(d);
    addCell(VTK_QUAD, { ia, ib, ic, id });
}

void MeshBuilder::addTriangleFan(const Vec3& center, const std::vector<Vec3>& ring)
{
    if (ring.size() < 3)
        return;

    const vtkIdType ic = addPoint(center);
    const vtkIdType first = numberOfPoints();
    for (const auto& p : ring)
        addPoint(p);

    const vtkIdType n = static_cast<vtkIdType>(ring.size());
    for (vtkIdType i = 0; i < n; ++i)
    {
        addCell(VTK_TRIANGLE, { ic, first + i, first + (i + 1) % n });
    }
}

void MeshBuilder::addHex(const std::array<Vec3, 8>& p, bool asFaces)
{
    const vtkIdType id0 = numberOfPoints();
    for (const auto& q : p)
        addPoint(q);

    if (!asFaces)
    {
        addCell(VTK_HEXAHEDRON, { id0, id0 + 1, id0 + 2, id0 + 3, id0 + 4, id0 + 5, id0 + 6, id0 + 7 });
        return;
    }

    // outward-facing quads
    static const int faces[6][4] = {
        { 0, 3, 2, 1 }, { 4, 5, 6, 7 }, { 0, 1, 5, 4 },
        { 1, 2, 6, 5 }, { 2, 3, 7, 6 }, { 3, 0, 4, 7 }
    };
    for (const auto& f : faces)
    {
        addCell(VTK_QUAD, { id0 + f[0], id0 + f[1], id0 + f[2], id0 + f[3] });
    }
}

void MeshBuilder::addBox(const Vec3& mn, const Vec3& mx, bool asFaces)
{
    addHex({ Vec3{ mn[0], mn[1], mn[2] }, Vec3{ mx[0], mn[1], mn[2] },
             Vec3{ mx[0], mx[1], mn[2] }, Vec3{ mn[0], mx[1], mn[2] },
             Vec3{ mn[0], mn[1], mx[2] }, Vec3{ mx[0], mn[1], mx[2] },
             Vec3{ mx[0], mx[1], mx[2] }, Vec3{ mn[0], mx[1], mx[2] } },
           asFaces);
}

void MeshBuilder::addCylinder(const Vec3& c, char axis, double radius, double length, int resolution)
{
    // u: coordinate along the axis, (a, b): coordinates in the circle plane
    auto map = [&](double u, double a, double b) -> Vec3
    {
        switch (axis)
        {
        case 'X': case 'x': return { c[0] + u, c[1] + a, c[2] + b };
        case 'Y': case 'y': return { c[0] + a, c[1] + u, c[2] + b };
        default:            return { c[0] + a, c[1] + b, c[2] + u };
        }
    };

    const vtkIdType id0 = numberOfPoints();
    for (int i = 0; i < resolution; ++i)
    {
        const double t = 2.0 * kPi * i / resolution;
        const double a = radius * std::cos(t);
        const double b = radius * std::sin(t);
        addPoint(map(-0.5 * length, a, b));
        addPoint(map(+0.5 * length, a, b));
    }

    for (int i = 0; i < resolution; ++i)
    {
        const int j = (i + 1) % resolution;
        addCell(VTK_QUAD, { id0 + 2 * i, id0 + 2 * j, id0 + 2 * j + 1, id0 + 2 * i + 1 });
    }

    std::vector<vtkIdType> capA, capB;
    for (int i = 0; i < resolution; ++i)
    {
        capA.push_back(id0 + 2 * (resolution - 1 - i));
        capB.push_back(id0 + 2 * i + 1);
    }
    addCell(VTK_POLYGON, capA);
    addCell(VTK_POLYGON, capB);
}

void MeshBuilder::addCone(const Vec3& base, double radius, double height, int resolution)
{
    const vtkIdType id0 = numberOfPoints();
    for (int i = 0; i < resolution; ++i)
    {
        const double t = 2.0 * kPi * i / resolution;
        addPoint({ base[0] + radius * std::cos(t), base[1] + radius * std::sin(t), base[2] });
    }
    const vtkIdType apex = addPoint({ base[0], base[1], base[2] + height });

    for (int i = 0; i < resolution; ++i)
    {
        addCell(VTK_TRIANGLE, { id0 + i, id0 + (i + 1) % resolution, apex });
    }

    std::vector<vtkIdType> cap;
    for (int i = resolution - 1; i >= 0; --i)
        cap.push_back(id0 + i);
    addCell(VTK_POLYGON, cap);
}

vtkSmartPointer<vtkUnstructuredGrid> MeshBuilder::buildGrid() const
{
    auto grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    grid->SetPoints(makePoints(m_points));
    grid->AllocateExact(static_cast<vtkIdType>(m_types.size()),
                        static_cast<vtkIdType>(m_connectivity.size()));

    for (size_t c = 0; c < m_types.size(); ++c)
    {
        const vtkIdType off = m_offsets[c];
        const vtkIdType n = m_offsets[c + 1] - off;
        grid->InsertNextCell(m_types[c], n, m_connectivity.data() + off);
    }

    attachArrays(grid, m_colors, m_scalars);
    return grid;
}

vtkSmartPointer<vtkPolyData> MeshBuilder::buildPolyData() const
{
    auto polys = vtkSmartPointer<vtkCellArray>::New();
    std::vector<Rgb> colors;
    std::vector<double> scalars;

    for (size_t c = 0; c < m_types.size(); ++c)
    {
        if (!is2DCell(m_types[c]))
            continue;

        const vtkIdType off = m_offsets[c];
        const vtkIdType n = m_offsets[c + 1] - off;
        polys->InsertNextCell(n, m_connectivity.data() + off);
        colors.push_back(m_colors[c]);
        scalars.push_back(m_scalars[c]);
    }

    auto poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(makePoints(m_points));
    poly->SetPolys(polys);
    attachArrays(poly, colors, scalars);
    return poly;
}
