#pragma once

#include <vector>
#include <vtkVector.h>
#include <vtkSmartPointer.h>
#include <optional>


class vtkOBBTree;
class vtkPolyData;

namespace RayCastUtil
{
    struct Ray
    {
        vtkVector3d StartPos;
        vtkVector3d EndPos;
    };

    struct MeshDRRInfo
    {
        vtkSmartPointer<vtkPolyData> Mesh;
        std::vector<vtkVector3d> InputPoints;
        vtkVector3d FocalPoint;
        double AttenuationCoefficient = 1.0;
    };

    vtkSmartPointer<vtkOBBTree> GetOBBTree(vtkPolyData* polyData);
    double GetIntegral(const double meshBounds[6], vtkOBBTree* obbTree, const Ray& ray, double attenuationCoefficient);
    std::vector<double> GetIntegral(const MeshDRRInfo& info);
}
