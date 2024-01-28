#pragma  once

#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkVector.h>

#include <vector>

namespace Algorithm
{
    enum class TraversalDirection
    {
        UNDEFINED_DIRECTION = -1,
        TRANSVERSE_IN_X = 0,
        TRANSVERSE_IN_Y = 1,
        TRANSVERSE_IN_Z = 2,
    };

    class RayCastHelper
    {
    public:
        void SetImage(vtkImageData* imageData);
        bool SetRay(const vtkVector3d& rayOrigin, const vtkVector3d& rayDirection);
        bool IntegrateAboveThreshold(double& integral, const double threshold);

    private:
        vtkImageData* mImage = nullptr;
        vtkVector3i mImageDimension{0, 0, 0};
        vtkVector3d mImageSpacing{0.0, 0.0, 0.0};
        std::vector<vtkVector3d> mBoundingCorner;
        std::vector<vtkVector4d> mBoundingPlane;

        bool mIsValidRay = false;
        vtkVector3d mRayVoxelStartPosition{0.0, 0.0, 0.0};
        vtkVector3d mRayVoxelEndPosition{0.0, 0.0, 0.0};

        int mTotalRayVoxelPlanes = 0;
        TraversalDirection mTraversalDirection = TraversalDirection::UNDEFINED_DIRECTION;

        vtkVector3d mVoxelIncrement{0.0, 0.0, 0.0};
        vtkVector3d mPositionVoxel{0.0, 0.0, 0.0};

        const short* mRayIntersectionVoxels[4]{nullptr, nullptr, nullptr, nullptr};
        vtkVector3i mRayIntersectionVoxelIndex{0, 0, 0};


        void DefineCorners();
        void CalculateBoundingPlane();
        bool CalcRayIntercepts(const vtkVector3d& rayPosition, const vtkVector3d& rayDirection, vtkVector3d& rayStartCoord, vtkVector3d& rayEndCoord) const;
        void CalcDirectionVector();
        bool AdjustRayLength();
        void InitializeVoxelPointers();
        void Reset();
        double GetCurrentIntensity() const;
        void IncrementVoxelPointers();
        double GetRayPointSpacing() const;
    };
}
