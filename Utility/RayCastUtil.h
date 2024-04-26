#pragma once

#include <vector>
#include <vtkVector.h>


class vtkOBBTree;

class vtkPolyData;
namespace RayCastUtil
{
    struct Ray
    {
        vtkVector3d startPos;
        vtkVector3d endPos;
    };

    vtkSmartPointer<vtkOBBTree> GetOBBTree(vtkPolyData* polyData);
    std::vector<vtkVector3d> GetRayMeshIntersectionPoints(vtkOBBTree* obbTree, const Ray& ray);
    double IntegrateEnergy(vtkOBBTree* obbTree, const Ray& ray, double attenuationCoefficient);
}
