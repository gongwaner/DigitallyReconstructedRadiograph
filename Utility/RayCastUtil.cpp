#include <vtkPoints.h>
#include "RayCastUtil.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkOBBTree.h>
#include <vtkVectorOperators.h>

namespace RayCastUtil
{
    vtkSmartPointer<vtkOBBTree> GetOBBTree(vtkPolyData* polyData)
    {
        auto obbTree = vtkSmartPointer<vtkOBBTree>::New();
        obbTree = vtkSmartPointer<vtkOBBTree>::New();
        obbTree->SetDataSet(polyData);
        obbTree->BuildLocator();

        return obbTree;
    }

    std::vector<vtkVector3d> GetRayMeshIntersectionPoints(vtkOBBTree* obbTree, const Ray& ray)
    {
        auto intersectPoints = vtkSmartPointer<vtkPoints>::New();
        const int result = obbTree->IntersectWithLine(ray.startPos.GetData(), ray.endPos.GetData(), intersectPoints, nullptr);

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

    double IntegrateEnergy(vtkOBBTree* obbTree, const Ray& ray, const double attenuationCoefficient)
    {
        double attenuationSum = 0.0;

        auto intersectionPnts = GetRayMeshIntersectionPoints(obbTree, ray);
        if(!intersectionPnts.empty() && intersectionPnts.size() % 2 == 0)
        {
            for(auto i = 0; i < intersectionPnts.size() - 1; ++i)
            {
                // Calculate the distance to the next intersection point
                double distance = (intersectionPnts[i + 1] - intersectionPnts[i]).Norm();

                double attenuation = distance * attenuationCoefficient;
                attenuationSum += attenuation;
            }
        }

        return attenuationSum;
    }
};
