#pragma  once

#include <vtkSmartPointer.h>
#include <vtkVector.h>


class vtkImageData;
class vtkPolyData;
class vtkMatrix4x4;
class vtkOBBTree;

namespace Algorithm
{
    struct Ray
    {
        vtkVector3d StartPos;
        vtkVector3d EndPos;
    };

    struct MeshDRRInfo
    {
        vtkPolyData* Mesh;
        std::vector<vtkVector3d> InputPoints;
        vtkVector3d FocalPoint;
        double AttenuationCoefficient = 1.0;
    };

    class MeshDRR
    {
    public:
        void SetInputPolyData(vtkPolyData* polyData);
        void SetAttenuationCoefficient(double attenuationCoefficient);
        void SetTranslation(double transX, double transY, double transZ);
        void SetRotation(double rotX, double rotY, double rotZ);
        void SetTransform(vtkMatrix4x4* transform);
        void SetSourceToMeshDistance(float distance);
        void SetDefaultPixelValue(double value);
        void SetOutputImageDimension(const vtkVector3i& dim);
        void SetOutputImageDimension(int dimX, int dimY, int dimZ);
        void SetOutputSpacing(const vtkVector3d& spacing);
        void SetOutputSpacing(double spacingX, double spacingY, double spacingZ);

        void Update();

        vtkSmartPointer<vtkImageData> GetOutput() const;

    private:
        void ComputeCenteredEuler3DTransform();
        void ComputeFocalPoint();

        vtkSmartPointer<vtkImageData> GenerateOutputImageDataSeq() const;
        vtkSmartPointer<vtkImageData> GenerateOutputImageDataPar() const;

    private:
        bool mDebug = true;

        vtkPolyData* mPolyData = nullptr;
        double mAttenuationCoefficient = 1.0;
        vtkSmartPointer<vtkMatrix4x4> mTransform = nullptr;

        vtkVector3d mPolyDataCenter{0.0, 0.0, 0.0};
        vtkVector3d mTranslation{0.0, 0.0, 0.0};
        vtkVector3d mRotation{0.0, 0.0, 0.0};
        vtkVector3d mFocalPoint{0.0, 0.0, 0.0};

        //ray cast parameters
        double mSourceToMeshDistance = 200.0;
        double mMeshToDetectorDistance = 200.0;

        //output 2D DRR image
        vtkSmartPointer<vtkImageData> mOutputImageData = nullptr;
        vtkVector3d mOutputOrigin{0.0, 0.0, 0.0};
        vtkVector3i mOutputDimension{512, 512, 1};
        vtkVector3d mOutputSpacing{1.0, 1.0, 1.0};
        double mDefaultPixelValue = 0.0;
    };
}
