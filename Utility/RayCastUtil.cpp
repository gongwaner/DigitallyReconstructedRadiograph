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

    bool RayIntersectsAABB(const Ray& ray, const double bounds[6])
    {
        const auto dir = ray.EndPos - ray.StartPos;
        const auto invDir = vtkVector3d(1.0 / dir[0], 1.0 / dir[1], 1.0 / dir[2]);

        double t1 = (bounds[0] - ray.StartPos[0]) * invDir[0];//xin
        double t2 = (bounds[1] - ray.StartPos[0]) * invDir[0];//xmax
        double t3 = (bounds[2] - ray.StartPos[1]) * invDir[1];//ymin
        double t4 = (bounds[3] - ray.StartPos[1]) * invDir[1];//ymax
        double t5 = (bounds[4] - ray.StartPos[2]) * invDir[2];//zmin
        double t6 = (bounds[5] - ray.StartPos[2]) * invDir[2];//zmax

        double tMin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
        double tMax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

        //ray (line) is intersecting AABB, but the whole AABB is behind us
        if(tMax < 0)
            return false;

        if(tMin > tMax)//ray doesn't intersect AABB
            return false;

        return true;
    }

    std::optional<std::vector<vtkVector3d>> GetRayMeshIntersectionPoints(const double meshBounds[6], vtkOBBTree* obbTree, const Ray& ray)
    {
        if(!RayIntersectsAABB(ray, meshBounds))
            return std::nullopt;

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

    double GetIntegral(const double meshBounds[6], vtkOBBTree* obbTree, const Ray& ray, const double attenuationCoefficient)
    {
        double attenuationSum = 0.0;
        auto intersectionResult = GetRayMeshIntersectionPoints(meshBounds, obbTree, ray);
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

        auto meshBounds = info.Mesh->GetBounds();
        auto obbTree = GetOBBTree(info.Mesh);

        std::vector<double> attenuationSumVec(info.InputPoints.size(), 0.0);

        for(int pID = 0; pID < info.InputPoints.size(); ++pID)
        {
            const auto intersectionResult = GetRayMeshIntersectionPoints(meshBounds, obbTree, {info.FocalPoint, info.InputPoints[pID]});
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
