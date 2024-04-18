#pragma once

#include <vtkVector.h>
#include <vtkSmartPointer.h>
#include <vtkOBBTree.h>

class vtkPolyData;

namespace Algorithm
{
    struct Ray
    {
        vtkVector3d startPos;
        vtkVector3d endPos;
    };

    class MeshRayCastHelper
    {
    public:
        void SetInputPolyData(vtkPolyData* polyData);
        void SetRay(const Ray& ray);
        void SetAttenuationCoefficient(double coefficient);
        double IntegrateAboveThreshold(double threshold);

    private:
        vtkPolyData* mPolyData = nullptr;
        bool mPolyDataChanged = false;
        Ray mRay;
        vtkSmartPointer<vtkOBBTree> mObbTree = nullptr;
        double mAttenuationCoefficient = 1.0;

        void BuildOBBTree();

        std::vector<vtkVector3d> GetRayMeshIntersectionPoints();
    };
}
