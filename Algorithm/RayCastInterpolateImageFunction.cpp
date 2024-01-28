#include "RayCastInterpolateImageFunction.h"

#include <vtkVectorOperators.h>

#include "RayCastHelper.h"
#include "../Utility/Utility.h"

namespace Algorithm
{
    void RayCastInterpolateImageFunction::SetImageData(vtkSmartPointer<vtkImageData> imageData)
    {
        mImageData = imageData;
    }

    void RayCastInterpolateImageFunction::SetTransform(vtkSmartPointer<vtkMatrix4x4> transform)
    {
        mTransform = transform;
        mTransformedFocalPoint = TransformUtil::GetTransformedPoint(mFocalPoint, mTransform);
    }

    void RayCastInterpolateImageFunction::SetFocalPoint(const vtkVector3d& focalPoint)
    {
        mFocalPoint = focalPoint;
        mTransformedFocalPoint = TransformUtil::GetTransformedPoint(mFocalPoint, mTransform);
    }

    void RayCastInterpolateImageFunction::SetThreshold(const double threshold)
    {
        mThreshold = threshold;
    }

    double RayCastInterpolateImageFunction::Evaluate(const vtkVector3d& point)
    {
        auto origin = mImageData->GetOrigin();
        auto spacing = mImageData->GetSpacing();
        auto rayPosition = point;
        for(int i = 0; i < 3; ++i)
        {
            rayPosition[i] -= origin[i] - 0.5 * spacing[i];
        }

        auto direction = (mTransformedFocalPoint - point).Normalized();

        double integral = 0.0;
        RayCastHelper ray;
        ray.SetImage(mImageData);
        ray.SetRay(rayPosition, direction);
        ray.IntegrateAboveThreshold(integral, mThreshold);

        return integral;
    }

    double RayCastInterpolateImageFunction::EvaluateAtContinuousIndex(const double index[3])
    {
        double physicalPnt[3];
        mImageData->TransformContinuousIndexToPhysicalPoint(index, physicalPnt);

        return Evaluate(vtkVector3d(physicalPnt));
    }
}
