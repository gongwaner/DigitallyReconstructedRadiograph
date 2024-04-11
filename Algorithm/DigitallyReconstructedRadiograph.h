#pragma  once

#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include <vtkVector.h>


class vtkImageData;

namespace Algorithm
{
    //follows ITK DRR example: https://github.com/InsightSoftwareConsortium/ITK/blob/master/Examples/Filtering/DigitallyReconstructedRadiograph1.cxx
    class DigitallyReconstructedRadiograph
    {
    public:
        void SetInputImage(vtkImageData* imageData);
        void SetThreshold(double threshold);
        void SetSourceToImageDistance(float distance);
        void SetDefaultPixelValue(double value);
        void SetImageRotation(float rotX, float rotY, float rotZ);
        void SetOutputImageDimension(int dimX, int dimY, int dimZ);
        void SetOutputImageDimension(const int dimensions[3]);
        void SetOutputSpacing(double spacingX, double spacingY, double spacingZ);
        void SetOutputSpacing(const double spacing[3]);

        void Update();

        vtkSmartPointer<vtkImageData> GetOutput() const;

        bool mDebug = false;

    private:
        vtkImageData* mImageData = nullptr;
        vtkSmartPointer<vtkMatrix4x4> mTransform = nullptr;

        vtkVector3d mImageCenter{0.0, 0.0, 0.0};
        vtkVector3d mTranslation{0.0, 0.0, 0.0};
        vtkVector3d mFocalPoint{0.0, 0.0, 0.0};

        //Rotation around x,y,z axis in degrees
        float mRotX = 0.0f;
        float mRotY = 0.0f;
        float mRotZ = 0.0f;

        //ray cast parameters
        double mSourceToImageDistance = 400.0;
        double mImageToOutputDistance = 400.0;
        double mThreshold = 0.0;

        //output 2D DRR image
        vtkSmartPointer<vtkImageData> mOutputImageData = nullptr;
        vtkVector3d mOutputOrigin = vtkVector3d(0, 0, 0);
        int mOutputDimension[3]{512, 512, 1};
        double mOutputSpacing[3]{1.0, 1.0, 1.0};
        double mDefaultPixelValue = 0.0;

        void ComputeCenteredEuler3DTransform();
        void ComputeFocalPoint();

        vtkSmartPointer<vtkImageData> GenerateOutputImageDataSeq() const;
        vtkSmartPointer<vtkImageData> GenerateOutputImageDataPar() const;
    };
}
