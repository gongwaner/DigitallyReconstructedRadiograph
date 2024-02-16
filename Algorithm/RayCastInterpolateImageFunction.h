#pragma once

#include <vtkVector.h>


class vtkImageData;
class vtkMatrix4x4;

namespace Algorithm
{
    class RayCastInterpolateImageFunction
    {
    public:
        void SetImageData(vtkImageData* imageData);
        void SetTransform(vtkMatrix4x4* transform);
        void SetFocalPoint(const vtkVector3d& focalPoint);
        void SetThreshold(double threshold);
        double EvaluateAtContinuousIndex(const double index[3]) const;

    private:
        vtkImageData* mImageData = nullptr;
        vtkMatrix4x4* mTransform = nullptr;
        vtkVector3d mFocalPoint{0, 0, 0};
        vtkVector3d mTransformedFocalPoint{0, 0, 0};
        double mThreshold = 0.0;

        double Evaluate(const vtkVector3d& point) const;
    };
}
