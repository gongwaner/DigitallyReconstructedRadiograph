#pragma once

#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkMatrix4x4.h>
#include <vtkVector.h>

#include "RayCastHelper.h"

namespace Algorithm
{
    class RayCastInterpolateImageFunction
    {
    public:
        void SetImageData(vtkSmartPointer<vtkImageData> imageData);
        void SetTransform(vtkSmartPointer<vtkMatrix4x4> transform);
        void SetFocalPoint(const vtkVector3d& focalPoint);
        void SetThreshold(double threshold);
        double EvaluateAtContinuousIndex(const double index[3]);

    private:
        vtkSmartPointer<vtkImageData> mImageData = nullptr;
        vtkSmartPointer<vtkMatrix4x4> mTransform = vtkSmartPointer<vtkMatrix4x4>::New();
        vtkVector3d mFocalPoint = vtkVector3d(0, 0, 0);
        vtkVector3d mTransformedFocalPoint = vtkVector3d(0, 0, 0);
        double mThreshold = 0.0;

        double Evaluate(const vtkVector3d& point);
    };
}
