#include "MeshRayCastHelper.h"

#include <vtkPolyData.h>
#include <vtkVectorOperators.h>


namespace Algorithm
{
    void MeshRayCastHelper::SetInputPolyData(vtkPolyData* polyData)
    {
        mPolyData = polyData;
        mPolyDataChanged = true;
    }

    void MeshRayCastHelper::SetRay(const Ray& ray)
    {
        mRay = ray;
    }

    void MeshRayCastHelper::SetAttenuationCoefficient(double coefficient)
    {
        mAttenuationCoefficient = coefficient;
    }

    void MeshRayCastHelper::BuildOBBTree()
    {
        mObbTree = vtkSmartPointer<vtkOBBTree>::New();
        mObbTree->SetDataSet(mPolyData);
        mObbTree->BuildLocator();
    }

    std::vector<vtkVector3d> MeshRayCastHelper::GetRayMeshIntersectionPoints()
    {
        if(mPolyDataChanged)
        {
            BuildOBBTree();
            mPolyDataChanged = false;
        }

        auto intersectPoints = vtkSmartPointer<vtkPoints>::New();
        const int result = mObbTree->IntersectWithLine(mRay.startPos.GetData(), mRay.endPos.GetData(), intersectPoints, nullptr);

        if(result == 0)
        {
            //no intersection
            return {};
        }

        const auto pntsCnt = intersectPoints->GetNumberOfPoints();
        std::vector<vtkVector3d> intersectedPntsVec(pntsCnt);
        for(int i = 0; i < pntsCnt; i++)
        {
            intersectedPntsVec.push_back(vtkVector3d(intersectPoints->GetPoint(i)));
        }

        return intersectedPntsVec;
    }

    double MeshRayCastHelper::IntegrateAboveThreshold(const double threshold)
    {
        double attenuationSum = 0.0;

        auto intersectionPnts = GetRayMeshIntersectionPoints();
        if(!intersectionPnts.empty() && intersectionPnts.size() % 2 == 0)
        {
            for(auto i = 0; i < intersectionPnts.size() - 1; ++i)
            {
                // Calculate the distance to the next intersection point
                double distance = (intersectionPnts[i + 1] - intersectionPnts[i]).Norm();

                double attenuation = distance * mAttenuationCoefficient;
                attenuationSum += attenuation;
            }
        }

        return attenuationSum;
    }
}
