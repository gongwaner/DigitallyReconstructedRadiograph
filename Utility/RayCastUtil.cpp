#include "RayCastUtil.h"

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkOBBTree.h>
#include <vtkVectorOperators.h>
#include <vtkPoints.h>


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

    std::optional<std::vector<vtkVector3d>> GetRayMeshIntersectionPoints(vtkOBBTree* obbTree, const Ray& ray)
    {
        auto intersectPoints = vtkSmartPointer<vtkPoints>::New();
        const int result = obbTree->IntersectWithLine(ray.StartPos.GetData(), ray.EndPos.GetData(), intersectPoints, nullptr);

        if(result == 0) //no intersection
            return std::nullopt;

        const auto pntsCnt = intersectPoints->GetNumberOfPoints();
        std::vector<vtkVector3d> intersectedPntsVec(pntsCnt);
        for(int i = 0; i < pntsCnt; i++)
        {
            intersectedPntsVec.push_back(vtkVector3d(intersectPoints->GetPoint(i)));
        }

        return intersectedPntsVec;
    }

    double GetIntegral(vtkOBBTree* obbTree, const Ray& ray, const double attenuationCoefficient)
    {
        double attenuationSum = 0.0;
        auto intersectionResult = GetRayMeshIntersectionPoints(obbTree, ray);
        if(intersectionResult)
        {
            const auto& intersectionPnts = *intersectionResult;
            if(intersectionPnts.size() % 2 == 0)
            {
                for(auto i = 0; i < intersectionPnts.size() - 1; ++i)
                {
                    // Calculate the distance to the next intersection point
                    const auto distance = (intersectionPnts[i + 1] - intersectionPnts[i]).Norm();
                    attenuationSum += distance * attenuationCoefficient;
                }
            }
        }

        return attenuationSum;
    }

    std::vector<double> GetIntegral(const MeshDRRInfo& info)
    {
        if(info.InputPoints.empty())
            throw std::runtime_error("RayCastUtil::GetIntegral(). Input points vector is empty!");

        std::vector<double> attenuationSumVec(info.InputPoints.size(), 0.0);

        for(int pID = 0; pID < info.InputPoints.size(); ++pID)
        {
            const auto intersectionResult = GetRayMeshIntersectionPoints(info.ObbTree, {info.FocalPoint, info.InputPoints[pID]});
            if(intersectionResult)
            {
                const auto& intersectionPnts = *intersectionResult;
                if(intersectionPnts.size() % 2 == 0)
                {
                    for(auto i = 0; i < intersectionPnts.size() - 1; ++i)
                    {
                        //calculate the distance to the next intersection point
                        const double distance = (intersectionPnts[i + 1] - intersectionPnts[i]).Norm();
                        attenuationSumVec[pID] += distance * info.AttenuationCoefficient;
                    }
                }
            }
        }

        return attenuationSumVec;
    }
};
