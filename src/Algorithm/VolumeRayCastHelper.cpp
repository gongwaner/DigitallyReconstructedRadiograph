#include "VolumeRayCastHelper.h"

#include <vtkImageData.h>
#include <vtkVectorOperators.h>


namespace Algorithm
{
    enum class Plane
    {
        Left = 0,
        Right = 1,
        Back = 2,
        Front = 3,
        Top = 4,
        Bottom = 5,
    };

    vtkVector4i GetCornerIndices(const int sideID)
    {
        const auto planeSide = static_cast<Plane>(sideID);

        if(planeSide == Plane::Left)
            return {0, 1, 3, 2};

        if(planeSide == Plane::Right)
            return {4, 5, 7, 6};

        if(planeSide == Plane::Back)
            return {1, 5, 7, 3};

        if(planeSide == Plane::Front)
            return {0, 2, 6, 4};

        if(planeSide == Plane::Top)
            return {0, 1, 5, 4};

        if(planeSide == Plane::Bottom)
            return {2, 3, 7, 6};

        return {};
    }

    vtkVector3i GetPlanePntsIndices(const int sideID)
    {
        if(sideID == (int) Plane::Left)
            return {1, 2, 3};

        if(sideID == (int) Plane::Right)
            return {4, 5, 6};

        if(sideID == (int) Plane::Back)
            return {5, 3, 7};

        if(sideID == (int) Plane::Front)
            return {2, 4, 6};

        if(sideID == (int) Plane::Top)
            return {1, 5, 0};

        if(sideID == (int) Plane::Bottom)
            return {3, 7, 2};

        return {};
    }

    void VolumeRayCastHelper::SetImage(vtkImageData* input)
    {
        mImage = input;
        const auto spacing = mImage->GetSpacing();
        const auto dim = mImage->GetDimensions();

        for(int i = 0; i < 3; ++i)
        {
            mImageDimension[i] = dim[i];
            mImageSpacing[i] = spacing[i];
        }

        DefineCorners();
        CalculateBoundingPlane();
    }

    void VolumeRayCastHelper::DefineCorners()
    {
        const int numOfCorners = 8;
        mBoundingCorner.resize(numOfCorners);

        const double x = mImageSpacing[0] * mImageDimension[0];
        const double y = mImageSpacing[1] * mImageDimension[1];
        const double z = mImageSpacing[2] * mImageDimension[2];

        mBoundingCorner[0] = vtkVector3d(0, 0, z);//upper left front
        mBoundingCorner[1] = vtkVector3d(0, y, z);//upper left back
        mBoundingCorner[2] = vtkVector3d(0, 0, 0);//bottom left front
        mBoundingCorner[3] = vtkVector3d(0, y, 0);//bottom left back
        mBoundingCorner[4] = vtkVector3d(x, 0, z);//upper right front
        mBoundingCorner[5] = vtkVector3d(x, y, z);//upper right back
        mBoundingCorner[6] = vtkVector3d(x, 0, 0);//bottom right front
        mBoundingCorner[7] = vtkVector3d(x, y, 0);//bottom right back
    }

    void VolumeRayCastHelper::CalculateBoundingPlane()
    {
        const int numOfPlanes = 6;
        mBoundingPlane.resize(numOfPlanes);

        //find the equations of the planes
        for(int j = 0; j < numOfPlanes; ++j)
        {
            //lines from one corner to another in x,y,z direction
            const auto planePntsIndices = GetPlanePntsIndices(j);
            const auto line1 = mBoundingCorner[planePntsIndices[0]] - mBoundingCorner[planePntsIndices[1]];
            const auto line2 = mBoundingCorner[planePntsIndices[0]] - mBoundingCorner[planePntsIndices[2]];

            //take cross product
            vtkVector3d cross;
            cross[0] = line1[1] * line2[2] - line2[1] * line1[2];
            cross[1] = line2[0] * line1[2] - line1[0] * line2[2];
            cross[2] = line1[0] * line2[1] - line2[0] * line1[1];

            if(abs(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]) <= 1e-6)
            {
                throw std::runtime_error("ERROR! VolumeRayCastHelper::CalcPlanesAndCorners() Divide by zero plane!");
            }

            //find constant
            const auto D = -(cross[0] * mBoundingCorner[planePntsIndices[0]][0] + cross[1] * mBoundingCorner[planePntsIndices[0]][1] +
                             cross[2] * mBoundingCorner[planePntsIndices[0]][2]);

            //initialise plane value and normalise
            const auto ratio = 1.0 / std::sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
            for(int i = 0; i < 3; ++i)
                mBoundingPlane[j][i] = cross[i] * ratio;

            mBoundingPlane[j][3] = D * ratio;
        }
    }

    bool VolumeRayCastHelper::CalcRayIntercepts(const vtkVector3d& rayPosition, const vtkVector3d& rayDirection,
                                                vtkVector3d& rayStartCoord, vtkVector3d& rayEndCoord) const
    {
        //calculate ray-plane interception points
        const int numOfPlanes = 6;
        const int cornerIndicesCnt = 4;

        std::vector<bool> hasIntersection(numOfPlanes, false);
        std::vector<vtkVector3d> cornerVect(cornerIndicesCnt, vtkVector3d(0.0, 0.0, 0.0));

        std::vector<vtkVector3d> cubeIntercepts(numOfPlanes);
        int nSidesCrossed = 0;

        for(int sideID = 0; sideID < numOfPlanes; ++sideID)
        {
            const double denom = (mBoundingPlane[sideID][0] * rayDirection[0] + mBoundingPlane[sideID][1] * rayDirection[1] +
                                  mBoundingPlane[sideID][2] * rayDirection[2]);

            vtkVector3d interceptPnt;
            if(static_cast<long>(denom * 100) != 0)
            {
                hasIntersection[sideID] = true;

                const double d = -(mBoundingPlane[sideID][3] + mBoundingPlane[sideID][0] * rayPosition[0] +
                                   mBoundingPlane[sideID][1] * rayPosition[1] + mBoundingPlane[sideID][2] * rayPosition[2]) /
                                 denom;

                for(int i = 0; i < 3; ++i)
                    interceptPnt[i] = rayPosition[i] + d * rayDirection[i];

                const auto cornerIndices = GetCornerIndices(sideID);
                //calculate vectors from corner of ct volume to intercept.
                for(int i = 0; i < cornerIndicesCnt; ++i)
                {
                    for(int index = 0; index < 3; ++index)
                    {
                        cornerVect[i][index] = mBoundingCorner[cornerIndices[i]][index] - interceptPnt[index];
                    }
                }
            }

            //do cross product with these vectors
            int cross[cornerIndicesCnt][3];
            for(unsigned int i = 0; i < cornerIndicesCnt; ++i)
            {
                const auto a = cornerVect[i];
                const auto b = cornerVect[(i + 1) % cornerIndicesCnt];

                //the int and divide by 100 are to avoid rounding errors.
                //if these are not included then you get values fluctuating around 0
                //and so in the subsequent check, all the values are not above or below 0.
                //If you "INT" by too much here though you can get problems in the corners of your volume
                //when rays are allowed to go through more than one plane.
                cross[i][0] = static_cast<int>((a[1] * b[2] - a[2] * b[1]) / 100);
                cross[i][1] = static_cast<int>((a[2] * b[0] - a[0] * b[2]) / 100);
                cross[i][2] = static_cast<int>((a[0] * b[1] - a[1] * b[0]) / 100);
            }

            //see if a sign change occurred between all these cross products
            //if not, then the ray went through this plane
            int crossFlag = 0;
            for(int i = 0; i < 3; ++i)
            {
                if((cross[0][i] <= 0 && cross[1][i] <= 0 && cross[2][i] <= 0 && cross[3][i] <= 0) ||
                   (cross[0][i] >= 0 && cross[1][i] >= 0 && cross[2][i] >= 0 && cross[3][i] >= 0))
                {
                    ++crossFlag;
                }
            }

            if(crossFlag == 3 && hasIntersection[sideID])
            {
                for(int k = 0; k < 3; ++k)
                    cubeIntercepts[nSidesCrossed][k] = interceptPnt[k];

                ++nSidesCrossed;
            }
        }

        if(nSidesCrossed == 2)
        {
            rayStartCoord = cubeIntercepts[0];
            rayEndCoord = cubeIntercepts[1];

            return true;
        }

        //if 'nSidesCrossed' is larger than 2, this means that the ray goes through
        //a corner of the volume and due to rounding errors, the ray is
        //deemed to go through more than two planes.  To obtain the correct
        //start and end positions we choose the two intercept values which
        //are furthest from each other.
        if(nSidesCrossed >= 3)
        {
            if(nSidesCrossed >= 5)
                std::cout << "WARNING: 5 sides crossed" << std::endl;

            double maxInterDist = 0.0;
            for(int j = 0; j < nSidesCrossed - 1; ++j)
            {
                for(int k = j + 1; k < nSidesCrossed; ++k)
                {
                    double interDist = 0.0;
                    for(int i = 0; i < 3; ++i)
                    {
                        interDist += (cubeIntercepts[j][i] - cubeIntercepts[k][i]) *
                                     (cubeIntercepts[j][i] - cubeIntercepts[k][i]);
                    }

                    if(interDist > maxInterDist)
                    {
                        maxInterDist = interDist;

                        rayStartCoord = cubeIntercepts[j];
                        rayEndCoord = cubeIntercepts[k];
                    }
                }
            }
            return true;
        }

        return false;
    }

    void VolumeRayCastHelper::InitializeVoxelPointers()
    {
        for(int i = 0; i < 3; ++i)
        {
            mRayIntersectionVoxelIndex[i] = (int) mRayVoxelStartPosition[i];
        }
        const auto ix = mRayIntersectionVoxelIndex[0];
        const auto iy = mRayIntersectionVoxelIndex[1];
        const auto iz = mRayIntersectionVoxelIndex[2];

        for(auto& element: mRayIntersectionVoxels)
            element = nullptr;

        if(mTraversalDirection == TraversalDirection::TRANSVERSE_IN_X)
        {
            if((ix >= 0) && (ix < mImageDimension[0]) && (iy >= 0) && (iy + 1 < mImageDimension[1]) && (iz >= 0) &&
               (iz + 1 < mImageDimension[2]))
            {
                mRayIntersectionVoxels[0] = static_cast<short*>(mImage->GetScalarPointer(ix, iy, iz));
                mRayIntersectionVoxels[1] = static_cast<short*>(mImage->GetScalarPointer(ix, iy + 1, iz));
                mRayIntersectionVoxels[2] = static_cast<short*>(mImage->GetScalarPointer(ix, iy, iz + 1));
                mRayIntersectionVoxels[3] = static_cast<short*>(mImage->GetScalarPointer(ix, iy + 1, iz + 1));
            }
        }
        else if(mTraversalDirection == TraversalDirection::TRANSVERSE_IN_Y)
        {
            if((ix >= 0) && (ix + 1 < mImageDimension[0]) && (iy >= 0) && (iy < mImageDimension[1]) && (iz >= 0) &&
               (iz + 1 < mImageDimension[2]))
            {
                mRayIntersectionVoxels[0] = static_cast<short*>(mImage->GetScalarPointer(ix, iy, iz));
                mRayIntersectionVoxels[1] = static_cast<short*>(mImage->GetScalarPointer(ix + 1, iy, iz));
                mRayIntersectionVoxels[2] = static_cast<short*>(mImage->GetScalarPointer(ix, iy, iz + 1));
                mRayIntersectionVoxels[3] = static_cast<short*>(mImage->GetScalarPointer(ix + 1, iy, iz + 1));
            }
        }
        else if(mTraversalDirection == TraversalDirection::TRANSVERSE_IN_Z)
        {
            if((ix >= 0) && (ix + 1 < mImageDimension[0]) && (iy >= 0) && (iy + 1 < mImageDimension[1]) &&
               (iz >= 0) && (iz < mImageDimension[2]))
            {
                mRayIntersectionVoxels[0] = static_cast<short*>(mImage->GetScalarPointer(ix, iy, iz));
                mRayIntersectionVoxels[1] = static_cast<short*>(mImage->GetScalarPointer(ix + 1, iy, iz));
                mRayIntersectionVoxels[2] = static_cast<short*>(mImage->GetScalarPointer(ix, iy + 1, iz));
                mRayIntersectionVoxels[3] = static_cast<short*>(mImage->GetScalarPointer(ix + 1, iy + 1, iz));
            }
        }
        else
        {
            throw std::runtime_error("VolumeRayCastHelper::InitializeVoxelPointers(). Traversal direction undefined!");
        }
    }

    /**
     * Reset the iterator to the start of the ray.
     */
    void VolumeRayCastHelper::Reset()
    {
        if(mIsValidRay)
        {
            mPositionVoxel = mRayVoxelStartPosition;
            InitializeVoxelPointers();
        }
        else
        {
            mTraversalDirection = TraversalDirection::UNDEFINED_DIRECTION;
            mTotalRayVoxelPlanes = 0;

            for(int i = 0; i < 3; ++i)
            {
                mRayVoxelStartPosition[i] = 0.0;
                mRayVoxelEndPosition[i] = 0.0;
                mVoxelIncrement[i] = 0.0;
                mRayIntersectionVoxelIndex[i] = 0;
            }

            for(auto& element: mRayIntersectionVoxels)
            {
                element = nullptr;
            }
        }
    }

    /**
     * Calculate the ray direction vector in voxels and hence the voxel  increment required to
     * traverse the ray, and the number of interpolation points on the ray.
     * This routine also shifts the coordinate frame by half a voxel for two of the
     * directional components (i.e. those lying within the planes of voxels being traversed)
     */
    void VolumeRayCastHelper::CalcDirectionVector()
    {
        //calculate the number of voxels in each direction
        vtkVector3d diff;
        vtkVector3d absDiff;
        for(int i = 0; i < 3; ++i)
        {
            diff[i] = mRayVoxelStartPosition[i] - mRayVoxelEndPosition[i];
            absDiff[i] = abs(diff[i]);
        }

        //the direction iterated in is that with the greatest number of voxels
        const auto max = std::max(absDiff[0], std::max(absDiff[1], absDiff[2]));
        mTotalRayVoxelPlanes = (int) max;
        if(max == absDiff[0])
            mTraversalDirection = TraversalDirection::TRANSVERSE_IN_X;
        else if(max == absDiff[1])
            mTraversalDirection = TraversalDirection::TRANSVERSE_IN_Y;
        else
            mTraversalDirection = TraversalDirection::TRANSVERSE_IN_Z;

        const int cnt = 3;
        const int index = (int) mTraversalDirection;
        const int nextIndex = (index + 1) % cnt;
        const int prevIndex = (index + 2) % cnt;

        const double delta = diff[index];
        const bool reverse = delta < 0;
        mVoxelIncrement[index] = reverse ? 1 : -1;
        mVoxelIncrement[nextIndex] = reverse ? (diff[nextIndex] / delta) : -(diff[nextIndex] / delta);
        mVoxelIncrement[prevIndex] = reverse ? (diff[prevIndex] / delta) : -(diff[prevIndex] / delta);

        //alter the start position in order to place the center of the voxels in the correct positions,
        //rather than placing them at the corner of voxels which is what happens if this is not
        //carried out.  The reason why x has no -0.5 is because this is the direction we are going to
        //iterate in, and therefore we wish to go from center to center rather than finding the surrounding voxels.
        const auto startPos = mRayVoxelStartPosition[index];
        const auto incrementValue = mVoxelIncrement[index];
        const auto nextIncrementValue = mVoxelIncrement[nextIndex];
        const auto prevIncrementValue = mVoxelIncrement[prevIndex];

        mRayVoxelStartPosition[nextIndex] += ((int) startPos - startPos) * nextIncrementValue * incrementValue + 0.5 * nextIncrementValue - 0.5;
        mRayVoxelStartPosition[prevIndex] += ((int) startPos - startPos) * prevIncrementValue * incrementValue + 0.5 * prevIncrementValue - 0.5;
        mRayVoxelStartPosition[index] = (int) startPos + 0.5 * incrementValue;
    }

    bool IsWithinVolume(const vtkVector3i& start, const vtkVector3i& direction, const vtkVector3i& volumeDimension)
    {
        return (start[0] >= 0) && (start[1] >= 0) && (start[2] >= 0) &&
               (start[0] + direction[0] < volumeDimension[0]) &&
               (start[1] + direction[1] < volumeDimension[1]) &&
               (start[2] + direction[2] < volumeDimension[2]);
    }

    /**
    * Ensure that the ray lies within the volume by reducing the length of the ray
    * until both start and end coordinates lie inside the volume
    */
    bool VolumeRayCastHelper::AdjustRayLength()
    {
        vtkVector3i direction;
        if(mTraversalDirection == TraversalDirection::TRANSVERSE_IN_X)
            direction = {0, 1, 1};
        else if(mTraversalDirection == TraversalDirection::TRANSVERSE_IN_Y)
            direction = {1, 0, 1};
        else if(mTraversalDirection == TraversalDirection::TRANSVERSE_IN_Z)
            direction = {1, 1, 0};
        else
            return false;

        bool startOK, endOK;
        vtkVector3i start;
        do
        {
            for(int i = 0; i < 3; ++i)
                start[i] = static_cast<int>(std::floor(mRayVoxelStartPosition[i]));

            startOK = IsWithinVolume(start, direction, mImageDimension);
            if(!startOK)
            {
                for(int i = 0; i < 3; ++i)
                    mRayVoxelStartPosition[i] += mVoxelIncrement[i];

                --mTotalRayVoxelPlanes;
            }

            for(int i = 0; i < 3; ++i)
                start[i] = static_cast<int>(std::floor(mRayVoxelStartPosition[i] + mTotalRayVoxelPlanes * mVoxelIncrement[i]));
            endOK = IsWithinVolume(start, direction, mImageDimension);
            if(!endOK)
            {
                --mTotalRayVoxelPlanes;
            }
        } while((!(startOK && endOK)) && (mTotalRayVoxelPlanes > 1));

        return (startOK && endOK);
    }

    bool VolumeRayCastHelper::SetRay(const vtkVector3d& rayPosition, const vtkVector3d& rayDirection)
    {
        //compute the ray path for this coordinate in mm
        vtkVector3d rayStartCoordInMM, rayEndCoordInMM;
        mIsValidRay = CalcRayIntercepts(rayPosition, rayDirection, rayStartCoordInMM, rayEndCoordInMM);

        if(!mIsValidRay)
            return false;

        //convert the start and end coordinates of the ray to voxels
        mRayVoxelStartPosition = rayStartCoordInMM / mImageSpacing;
        mRayVoxelEndPosition = rayEndCoordInMM / mImageSpacing;

        CalcDirectionVector();
        mIsValidRay = AdjustRayLength();
        Reset();

        return mIsValidRay;
    }

    void VolumeRayCastHelper::SetAttenuationCoefficient(const double attenuationCoefficient)
    {
        mAttenuationCoefficient = attenuationCoefficient;
    }

    double VolumeRayCastHelper::GetCurrentIntensity() const
    {
        if(!mIsValidRay)
            return 0.0;

        const auto a = static_cast<double>(*mRayIntersectionVoxels[0]);
        const auto b = static_cast<double>(*mRayIntersectionVoxels[1] - a);
        const auto c = static_cast<double>(*mRayIntersectionVoxels[2] - a);
        const auto d = static_cast<double>(*mRayIntersectionVoxels[3] - a - b - c);

        double y, z;
        if(mTraversalDirection == TraversalDirection::TRANSVERSE_IN_X)
        {
            y = mPositionVoxel[1] - std::floor(mPositionVoxel[1]);
            z = mPositionVoxel[2] - std::floor(mPositionVoxel[2]);
        }
        else if(mTraversalDirection == TraversalDirection::TRANSVERSE_IN_Y)
        {
            y = mPositionVoxel[0] - std::floor(mPositionVoxel[0]);
            z = mPositionVoxel[2] - std::floor(mPositionVoxel[2]);
        }
        else if(mTraversalDirection == TraversalDirection::TRANSVERSE_IN_Z)
        {
            y = mPositionVoxel[0] - std::floor(mPositionVoxel[0]);
            z = mPositionVoxel[1] - std::floor(mPositionVoxel[1]);
        }
        else
        {
            //throw std::runtime_error("VolumeRayCastHelper::GetCurrentIntensity(). The ray traversal direction is unset");
        }

        return a + b * y + c * z + d * y * z;
    }

    void VolumeRayCastHelper::IncrementVoxelPointers()
    {
        vtkVector3d prev;
        for(int i = 0; i < 3; ++i)
        {
            prev[i] = mPositionVoxel[i];
            mPositionVoxel[i] += mVoxelIncrement[i];
        }

        vtkVector3i diff;
        for(int i = 0; i < 3; ++i)
        {
            diff[i] = static_cast<int>(mPositionVoxel[i]) - static_cast<int>(prev[i]);
            mRayIntersectionVoxelIndex[i] += diff[i];
        }

        int totalRayVoxelPlanes = diff[0] + diff[1] * mImageDimension[0] + diff[2] * mImageDimension[0] * mImageDimension[1];

        for(auto& element: mRayIntersectionVoxels)
            element += totalRayVoxelPlanes;
    }

    double VolumeRayCastHelper::GetRayPointSpacing() const
    {
        if(mIsValidRay)
        {
            return std::sqrt(mVoxelIncrement[0] * mImageSpacing[0] * mVoxelIncrement[0] * mImageSpacing[0] +
                             mVoxelIncrement[1] * mImageSpacing[1] * mVoxelIncrement[1] * mImageSpacing[1] +
                             mVoxelIncrement[2] * mImageSpacing[2] * mVoxelIncrement[2] * mImageSpacing[2]);
        }

        return 0.0;
    }

    bool VolumeRayCastHelper::IntegrateAboveThreshold(double& integral, const double threshold)
    {
        if(!mIsValidRay)
            return false;

        //step along the ray as quickly as possible integrating the interpolated intensities
        double sum = 0.0;
        for(int numOfPlanesTraversed = 0; numOfPlanesTraversed < mTotalRayVoxelPlanes; ++numOfPlanesTraversed)
        {
            const auto intensity = GetCurrentIntensity();
            if(intensity > threshold)
            {
                sum += intensity - threshold;
            }

            IncrementVoxelPointers();
        }

        //the ray passes through the volume one plane of voxels at a time
        //however, if its moving diagonally the ray points will be further apart
        //so account for this by scaling by the distance moved.
        integral = sum * GetRayPointSpacing() * mAttenuationCoefficient;

        return true;
    }
}
