#include "MeshDRR.h"

#include <execution>

#include <vtkPolyData.h>
#include <vtkImageData.h>
#include <vtkMatrix4x4.h>
#include <vtkVectorOperators.h>
#include <vtkImageIterator.h>
#include <vtkOBBTree.h>

//submodule files
#include "TransformUtil.h"
#include "MeshUtil.h"
#include "CollisionDetectionUtil.h"


namespace
{
    constexpr bool DEBUG = true;
}

namespace Algorithm
{
    void MeshDRR::SetInputPolyData(vtkPolyData* polyData)
    {
        mPolyData = polyData;
        mPolyDataCenter = vtkVector3d(mPolyData->GetCenter());
    }

    void MeshDRR::SetAttenuationCoefficient(const double coefficient)
    {
        mAttenuationCoefficient = coefficient;
    }

    void MeshDRR::SetTranslation(double transX, double transY, double transZ)
    {
        mTranslation[0] = transX;
        mTranslation[1] = transY;
        mTranslation[2] = transZ;
    }

    void MeshDRR::SetRotation(double rotX, double rotY, double rotZ)
    {
        mRotation[0] = rotX;
        mRotation[1] = rotY;
        mRotation[2] = rotZ;
    }

    void MeshDRR::SetTransform(vtkMatrix4x4* transform)
    {
        mTransform = transform;
    }

    void MeshDRR::SetSourceToMeshDistance(const float distance)
    {
        mSourceToMeshDistance = distance;
    }

    void MeshDRR::SetDefaultPixelValue(const double value)
    {
        mDefaultPixelValue = value;
    }

    void MeshDRR::SetOutputImageDimension(const vtkVector3i& dim)
    {
        mOutputDimension = dim;
    }

    void MeshDRR::SetOutputImageDimension(const int dimX, const int dimY, const int dimZ)
    {
        mOutputDimension[0] = dimX;
        mOutputDimension[1] = dimY;
        mOutputDimension[2] = dimZ;
    }

    void MeshDRR::SetOutputSpacing(const vtkVector3d& spacing)
    {
        mOutputSpacing = spacing;
    }

    void MeshDRR::SetOutputSpacing(const double spacingX, const double spacingY, const double spacingZ)
    {
        mOutputSpacing[0] = spacingX;
        mOutputSpacing[1] = spacingY;
        mOutputSpacing[2] = spacingZ;
    }

    void MeshDRR::ComputeCenteredEuler3DTransform()
    {
        const double degreeToRadians = (std::atan(1.0) * 4.0) / 180.0;
        mTransform = TransformUtil::GetTransformationMatrix(mPolyDataCenter, mTranslation, degreeToRadians * mRotation);
    }

    void MeshDRR::ComputeFocalPoint()
    {
        // ray source/focal point.
        mFocalPoint[0] = mPolyDataCenter[0];
        mFocalPoint[1] = mPolyDataCenter[1];
        mFocalPoint[2] = mPolyDataCenter[2] - mSourceToMeshDistance;

        mFocalPoint = TransformUtil::GetTransformedPoint(mFocalPoint, mTransform);
    }

    std::optional<std::vector<vtkVector3d>> GetRayMeshIntersectionPoints(const double meshBounds[6], vtkOBBTree* obbTree, const Ray& ray)
    {
        if(!CollisionDetectionUtil::RayIntersectsAABB(ray.StartPos, ray.EndPos, meshBounds))
            return std::nullopt;

        auto intersectPoints = vtkSmartPointer<vtkPoints>::New();
        const int result = obbTree->IntersectWithLine(ray.StartPos.GetData(), ray.EndPos.GetData(), intersectPoints, nullptr);

        if(result == 0) //no intersection
            return std::nullopt;

        const auto pntsCnt = intersectPoints->GetNumberOfPoints();
        std::vector<vtkVector3d> intersectedPntsVec(pntsCnt);
        for(int i = 0; i < pntsCnt; i++)
        {
            intersectedPntsVec[i] = vtkVector3d(intersectPoints->GetPoint(i));
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
            throw std::runtime_error("Mesh DRR GetIntegral(). Input points vector is empty!");

        auto meshBounds = info.Mesh->GetBounds();
        auto obbTree = MeshUtil::GetOBBTree(info.Mesh);

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

    vtkSmartPointer<vtkImageData> MeshDRR::GenerateOutputImageDataSeq() const
    {
        if constexpr(DEBUG)
        {
            std::cout << "Output image dimension: " << mOutputDimension[0] << ", " << mOutputDimension[1] << ", " << mOutputDimension[2]
                      << ", spacing: " << mOutputSpacing[0] << ", " << mOutputSpacing[1] << ", " << mOutputSpacing[2] << std::endl;
        }

        auto outputImage = vtkSmartPointer<vtkImageData>::New();
        outputImage->SetOrigin(mOutputOrigin.GetData());
        outputImage->SetDimensions(mOutputDimension.GetData());
        outputImage->SetSpacing(mOutputSpacing.GetData());
        outputImage->AllocateScalars(VTK_UNSIGNED_SHORT, 1);

        const auto vectorSize = mOutputDimension[0] * mOutputDimension[1] * mOutputDimension[2];
        std::vector<vtkVector3d> inputPointsVec;
        inputPointsVec.reserve(vectorSize);

        std::vector<short*> outPixelsVec;
        outPixelsVec.reserve(vectorSize);

        for(int z = 0; z < mOutputDimension[2]; z++)
        {
            for(int y = 0; y < mOutputDimension[1]; y++)
            {
                for(int x = 0; x < mOutputDimension[0]; x++)
                {
                    //current output point position
                    double outputPoint[3];
                    outputImage->TransformIndexToPhysicalPoint(x, y, z, outputPoint);

                    //corresponding input pixel position
                    auto inputPoint = TransformUtil::GetTransformedPoint(outputPoint, mTransform);

                    inputPointsVec.push_back(inputPoint);

                    //cache output pixel
                    auto outPixel = static_cast<short*>(outputImage->GetScalarPointer(x, y, z));
                    outPixelsVec.push_back(outPixel);
                }
            }
        }

        MeshDRRInfo info;
        info.Mesh = mPolyData;
        info.InputPoints = inputPointsVec;
        info.FocalPoint = mFocalPoint;
        info.AttenuationCoefficient = mAttenuationCoefficient;

        const auto resultPixelValueVec = GetIntegral(info);

        for(int i = 0; i < vectorSize; ++i)
        {
            auto ptr = outPixelsVec[i];
            *ptr = (short) resultPixelValueVec[i];
        }

        return outputImage;
    }

#ifdef _WIN32
    vtkSmartPointer<vtkImageData> MeshDRR::GenerateOutputImageDataPar() const
    {
        if constexpr(DEBUG)
        {
            std::cout << "Output image dimension: " << mOutputDimension[0] << ", " << mOutputDimension[1] << ", " << mOutputDimension[2]
                      << ", spacing: " << mOutputSpacing[0] << ", " << mOutputSpacing[1] << ", " << mOutputSpacing[2] << std::endl;
        }

        auto outputImage = vtkSmartPointer<vtkImageData>::New();
        outputImage->SetOrigin(mOutputOrigin.GetData());
        outputImage->SetDimensions(mOutputDimension.GetData());
        outputImage->SetSpacing(mOutputSpacing.GetData());
        outputImage->AllocateScalars(VTK_UNSIGNED_SHORT, 1);

        const auto vectorSize = mOutputDimension[0] * mOutputDimension[1] * mOutputDimension[2];
        std::vector<vtkVector3d> inputPointsVec;
        inputPointsVec.reserve(vectorSize);

        std::vector<short*> outPixelsVec;
        outPixelsVec.reserve(vectorSize);

        for(int z = 0; z < mOutputDimension[2]; z++)
        {
            for(int y = 0; y < mOutputDimension[1]; y++)
            {
                for(int x = 0; x < mOutputDimension[0]; x++)
                {
                    //current output point position
                    double outputPoint[3];
                    outputImage->TransformIndexToPhysicalPoint(x, y, z, outputPoint);

                    //corresponding input pixel position
                    auto inputPoint = TransformUtil::GetTransformedPoint(outputPoint, mTransform);
                    inputPointsVec.push_back(inputPoint);

                    //cache output pixel
                    auto outPixel = static_cast<short*>(outputImage->GetScalarPointer(x, y, z));
                    outPixelsVec.push_back(outPixel);
                }
            }
        }

        auto meshBounds = mPolyData->GetBounds();
        auto obbTree = MeshUtil::GetOBBTree(mPolyData);

        std::vector<short> resultPixelValueVec(vectorSize);
        std::transform(std::execution::par, inputPointsVec.begin(), inputPointsVec.end(), resultPixelValueVec.begin(),
                       [this, &obbTree, &meshBounds](const vtkVector3d& point)
                       {
                           return (short) GetIntegral(meshBounds, obbTree, {mFocalPoint, point}, mAttenuationCoefficient);
                       });

        for(int i = 0; i < vectorSize; ++i)
        {
            auto ptr = outPixelsVec[i];
            *ptr = resultPixelValueVec[i];
        }

        return outputImage;
    }
#endif

    void MeshDRR::Update()
    {
        ComputeCenteredEuler3DTransform();
        ComputeFocalPoint();

        mOutputOrigin[0] = mPolyDataCenter[0] - mOutputSpacing[0] * (mOutputDimension[0] - 1.0) * 0.5;
        mOutputOrigin[1] = mPolyDataCenter[1] - mOutputSpacing[1] * (mOutputDimension[1] - 1.0) * 0.5;
        mOutputOrigin[2] = mPolyDataCenter[2] + mMeshToDetectorDistance;

#ifdef _WIN32
        mOutputImageData = GenerateOutputImageDataPar();
#else
        mOutputImageData = GenerateOutputImageDataSeq();
#endif
    }

    vtkSmartPointer<vtkImageData> MeshDRR::GetOutput() const
    {
        return mOutputImageData;
    }
}
