#include "MeshDRR.h"

#include "../Utility/Utility.h"
#include "../Utility/RayCastUtil.h"

#include <vtkPolyData.h>
#include <vtkImageData.h>
#include <vtkVectorOperators.h>
#include <vtkImageIterator.h>
#include <vtkImageCast.h>
#include <vtkOBBTree.h>

#include <execution>


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

    vtkSmartPointer<vtkImageData> MeshDRR::GenerateOutputImageDataSeq() const
    {
        if(mDebug)
            printf("Output image dimension: %d, %d, %d, spacing: %f, %f, %f\n", mOutputDimension[0], mOutputDimension[1], mOutputDimension[2], mOutputSpacing[0], mOutputSpacing[1],
                   mOutputSpacing[2]);

        auto outputImage = vtkSmartPointer<vtkImageData>::New();
        outputImage->SetOrigin(mOutputOrigin.GetData());
        outputImage->SetDimensions(mOutputDimension.GetData());
        outputImage->SetSpacing(mOutputSpacing.GetData());
        outputImage->AllocateScalars(VTK_UNSIGNED_SHORT, 1);

        auto obbTree = RayCastUtil::GetOBBTree(mPolyData);

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

                    //evaluate input at right position and copy to the output
                    auto outPixel = static_cast<short*>(outputImage->GetScalarPointer(x, y, z));
                    *outPixel = (short) RayCastUtil::IntegrateEnergy(obbTree, {mFocalPoint, inputPoint}, mAttenuationCoefficient);
                }
            }
        }

        return outputImage;
    }

#ifdef _WIN32
    vtkSmartPointer<vtkImageData> MeshDRR::GenerateOutputImageDataPar() const
    {
        if(mDebug)
            printf("Output image dimension: %d, %d, %d, spacing: %f, %f, %f\n", mOutputDimension[0], mOutputDimension[1], mOutputDimension[2], mOutputSpacing[0], mOutputSpacing[1],
                   mOutputSpacing[2]);

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

                    //evaluate input at right position and copy to the output
                    auto outPixel = static_cast<short*>(outputImage->GetScalarPointer(x, y, z));
                    outPixelsVec.push_back(outPixel);
                }
            }
        }

        auto obbTree = RayCastUtil::GetOBBTree(mPolyData);

        std::vector<short> resultPixelValueVec(vectorSize);
        std::transform(std::execution::par, inputPointsVec.begin(), inputPointsVec.end(), resultPixelValueVec.begin(), [this, &obbTree](const vtkVector3d& point)
        {
            return (short) RayCastUtil::IntegrateEnergy(obbTree, {mFocalPoint, point}, mAttenuationCoefficient);
        });

        for(int i = 0; i < resultPixelValueVec.size(); ++i)
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
        auto outputImage = GenerateOutputImageDataPar();
#else
        auto outputImage = GenerateOutputImageDataSeq();
#endif

        //convert to unsigned short
        auto castFilter = vtkSmartPointer<vtkImageCast>::New();
        castFilter->SetOutputScalarTypeToUnsignedShort();
        castFilter->SetInputData(outputImage);
        castFilter->Update();

        mOutputImageData = castFilter->GetOutput();
    }

    vtkSmartPointer<vtkImageData> MeshDRR::GetOutput() const
    {
        return mOutputImageData;
    }
}
