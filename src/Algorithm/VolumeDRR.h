#pragma  once

#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include <vtkVector.h>


class vtkImageData;

namespace Algorithm
{
    //reference ITK DRR example: https://github.com/InsightSoftwareConsortium/ITK/blob/master/Examples/Filtering/DigitallyReconstructedRadiograph1.cxx
    class VolumeDRR
    {
    public:
        void SetInputImage(vtkImageData* imageData);
        void SetThreshold(double threshold);
        void SetSourceToImageDistance(float distance);
        void SetDefaultPixelValue(double value);
        void SetImageTranslation(double transX, double transY, double transZ);
        void SetImageRotation(double rotX, double rotY, double rotZ);
        void SetImageTransform(vtkMatrix4x4* transform);
        void SetOutputImageDimension(const vtkVector3i& dim);
        void SetOutputImageDimension(int dimX, int dimY, int dimZ);
        void SetOutputSpacing(const vtkVector3d& spacing);
        void SetOutputSpacing(double spacingX, double spacingY, double spacingZ);
        void SetProjectionToPerspective();
        void SetProjectionToOrthographic();
        void SetDebugModeOn();

        void Update();

        vtkSmartPointer<vtkImageData> GetOutput() const;

    private:
        void ComputeCenteredEuler3DTransform();
        void ComputeFocalPoint();

        vtkVector3d GetOrthographicRayDirection() const;
        double GetIntegral(const vtkVector3d& point) const;
        vtkSmartPointer<vtkImageData> GenerateOutputImageDataSeq() const;
        vtkSmartPointer<vtkImageData> GenerateOutputImageDataPar() const;

    private:
        vtkImageData* mImageData = nullptr;
        vtkSmartPointer<vtkMatrix4x4> mTransform = nullptr;

        vtkVector3d mImageCenter{0.0, 0.0, 0.0};
        vtkVector3d mTranslation{0.0, 0.0, 0.0};
        vtkVector3d mRotation{0.0, 0.0, 0.0};
        vtkVector3d mFocalPoint{0.0, 0.0, 0.0};

        //ray cast parameters
        double mSourceToImageDistance = 200.0;
        double mImageToDetectorDistance = 200.0;
        double mThreshold = 0.0;

        //output 2D DRR image
        vtkSmartPointer<vtkImageData> mOutputImageData = nullptr;
        vtkVector3d mOutputOrigin{0.0, 0.0, 0.0};
        vtkVector3i mOutputDimension{512, 512, 1};
        vtkVector3d mOutputSpacing{1.0, 1.0, 1.0};
        double mDefaultPixelValue = 0.0;

        //projection mode
        bool mPerspectiveProjection = true;//false means orthographic projection

        //debug flag
        bool mDebug = false;
    };
}
