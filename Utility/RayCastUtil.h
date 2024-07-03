#pragma once

#include <vector>
#include <vtkVector.h>
#include <vtkSmartPointer.h>


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
        vtkSmartPointer<vtkOBBTree> ObbTree;
        std::vector<vtkVector3d> InputPoints;
        vtkVector3d FocalPoint;
        double AttenuationCoefficient = 1.0;
    };

    vtkSmartPointer<vtkOBBTree> GetOBBTree(vtkPolyData* polyData);
    std::optional<std::vector<vtkVector3d>> GetRayMeshIntersectionPoints(vtkOBBTree* obbTree, const Ray& ray);
    std::vector<double> GetIntegral(const MeshDRRInfo& info);
}
