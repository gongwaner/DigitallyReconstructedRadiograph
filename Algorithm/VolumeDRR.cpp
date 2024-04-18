#include "VolumeDRR.h"

#include "../Utility/Utility.h"
#include "VolumeRayCastHelper.h"

#include <vtkImageData.h>
#include <vtkImageIterator.h>
#include <vtkImageCast.h>
#include <vtkVectorOperators.h>

#include <execution>


namespace Algorithm
{
    void VolumeDRR::SetInputImage(vtkImageData* imageData)
    {
        mImageData = imageData;

        const auto imageOrigin = mImageData->GetOrigin();
        const auto imageSpacing = mImageData->GetSpacing();
        const auto imageDimension = mImageData->GetDimensions();

        if(mDebug)
        {
            printf("Input image dimension: %d, %d, %d, spacing: %f, %f, %f\n",
                   imageDimension[0], imageDimension[1], imageDimension[2],
                   imageSpacing[0], imageSpacing[1], imageSpacing[2]);
        }

        for(int i = 0; i < 3; ++i)
            mImageCenter[i] = imageOrigin[i] + imageSpacing[i] * imageDimension[i] * 0.5;
    }

    void VolumeDRR::SetThreshold(const double threshold)
    {
        mThreshold = threshold;
    }

    void VolumeDRR::SetSourceToImageDistance(const float distance)
    {
        mSourceToImageDistance = distance;
    }

    void VolumeDRR::SetDefaultPixelValue(const double value)
    {
        mDefaultPixelValue = value;
    }

    void VolumeDRR::SetImageTranslation(const double transX, const double transY, const double transZ)
    {
        mTranslation[0] = transX;
        mTranslation[1] = transY;
        mTranslation[2] = transZ;
    }

    void VolumeDRR::SetImageRotation(const double rotX, const double rotY, const double rotZ)
    {
        mRotation[0] = rotX;
        mRotation[1] = rotY;
        mRotation[2] = rotZ;
    }

    void VolumeDRR::SetImageTransform(vtkMatrix4x4* transform)
    {
        mTransform = transform;
    }

    void VolumeDRR::SetOutputImageDimension(const int dimX, const int dimY, const int dimZ)
    {
        mOutputDimension[0] = dimX;
        mOutputDimension[1] = dimY;
        mOutputDimension[2] = dimZ;
    }

    void VolumeDRR::SetOutputImageDimension(const vtkVector3i& dim)
    {
        mOutputDimension = dim;
    }

    void VolumeDRR::SetOutputSpacing(const double spacingX, const double spacingY, const double spacingZ)
    {
        mOutputSpacing[0] = spacingX;
        mOutputSpacing[1] = spacingY;
        mOutputSpacing[2] = spacingZ;
    }

    void VolumeDRR::SetOutputSpacing(const vtkVector3d& spacing)
    {
        mOutputSpacing = spacing;
    }

    void VolumeDRR::ComputeCenteredEuler3DTransform()
    {
        const double degreeToRadians = (std::atan(1.0) * 4.0) / 180.0;
        mTransform = TransformUtil::GetTransformationMatrix(mImageCenter, mTranslation, degreeToRadians * mRotation);
    }

    void VolumeDRR::ComputeFocalPoint()
    {
        //ray source/focal point. used by interpolator
        mFocalPoint[0] = mImageCenter[0];
        mFocalPoint[1] = mImageCenter[1];
        mFocalPoint[2] = mImageCenter[2] - mSourceToImageDistance * 0.5;

        mFocalPoint = TransformUtil::GetTransformedPoint(mFocalPoint, mTransform);
    }

    double VolumeDRR::Evaluate(const vtkVector3d& point) const
    {
        const auto origin = mImageData->GetOrigin();
        const auto spacing = mImageData->GetSpacing();
        auto rayPosition = point;
        for(int i = 0; i < 3; ++i)
        {
            rayPosition[i] -= origin[i] - 0.5 * spacing[i];
        }

        const auto direction = (mFocalPoint - point).Normalized();

        double integral = 0.0;

        VolumeRayCastHelper ray;
        ray.SetImage(mImageData);
        ray.SetRay(rayPosition, direction);
        ray.IntegrateAboveThreshold(integral, mThreshold);

        return integral;
    }

    //ref: https://github.com/InsightSoftwareConsortium/ITK/blob/12328e1d2ff8becab001654dccbb393657521691/Modules/Filtering/ImageGrid/include/itkResampleImageFilter.hxx#L369
    vtkSmartPointer<vtkImageData> VolumeDRR::GenerateOutputImageDataSeq() const
    {
        if(mDebug)
            printf("Output image dimension: %d, %d, %d, spacing: %f, %f, %f\n", mOutputDimension[0], mOutputDimension[1], mOutputDimension[2],
                   mOutputSpacing[0], mOutputSpacing[1],
                   mOutputSpacing[2]);

        auto outputImage = vtkSmartPointer<vtkImageData>::New();
        outputImage->SetOrigin(mOutputOrigin.GetData());
        outputImage->SetDimensions(mOutputDimension.GetData());
        outputImage->SetSpacing(mOutputSpacing.GetData());
        outputImage->AllocateScalars(mImageData->GetScalarType(), 1);

        //iterate through every pixel in detector
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
                    double inputIndex[3];
                    mImageData->TransformPhysicalPointToContinuousIndex(inputPoint.GetData(), inputIndex);

                    //evaluate input at right position and copy to the output
                    auto outPixel = static_cast<short*>(outputImage->GetScalarPointer(x, y, z));

                    //if(interpolator->IsInsideBuffer(inputIndex)
                    *outPixel = (short) Evaluate(inputPoint);
                    //else
                    //outPixel[0] = mDefaultPixelValue;
                }
            }
        }

        return outputImage;
    }

#ifdef _WIN32
    /**
     * parallel version of DDR calculation using parallel STL
     * currently available only on WIN
     */
    vtkSmartPointer<vtkImageData> VolumeDRR::GenerateOutputImageDataPar() const
    {
        if(mDebug)
            printf("Output image dimension: %d, %d, %d, spacing: %f, %f, %f\n",
                   mOutputDimension[0], mOutputDimension[1], mOutputDimension[2],
                   mOutputSpacing[0], mOutputSpacing[1], mOutputSpacing[2]);

        auto outputImage = vtkSmartPointer<vtkImageData>::New();
        outputImage->SetOrigin(mOutputOrigin.GetData());
        outputImage->SetDimensions(mOutputDimension.GetData());
        outputImage->SetSpacing(mOutputSpacing.GetData());
        outputImage->AllocateScalars(mImageData->GetScalarType(), 1);

        auto start = std::chrono::high_resolution_clock::now();

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
                    const auto inputPoint = TransformUtil::GetTransformedPoint(vtkVector3d(outputPoint), mTransform);
                    inputPointsVec.emplace_back(inputPoint);

                    auto outPixel = static_cast<short*>(outputImage->GetScalarPointer(x, y, z));
                    outPixelsVec.push_back(outPixel);
                }
            }
        }

        std::vector<short> resultPixelValueVec(vectorSize);
        std::transform(std::execution::par, inputPointsVec.begin(), inputPointsVec.end(), resultPixelValueVec.begin(),
                       [this](const vtkVector3d& point)
                       {
                           return (short) Evaluate(point);
                       });

        for(int i = 0; i < resultPixelValueVec.size(); ++i)
        {
            auto ptr = outPixelsVec[i];
            *ptr = resultPixelValueVec[i];
        }

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "the parallel version GenerateOutputImageDataPar() takes " << duration.count() << " ms" << std::endl;

        return outputImage;
    }
#endif

    void VolumeDRR::Update()
    {
        ComputeCenteredEuler3DTransform();
        ComputeFocalPoint();

        // The default position of the input volume, prior to its transformation is
        // half-way between the ray source and screen and unless specified
        // otherwise the normal from the "screen" to the ray source passes
        // directly through the centre of the DRR.
        mOutputOrigin[0] = mImageCenter[0] - mOutputSpacing[0] * (mOutputDimension[0] - 1.0) * 0.5;
        mOutputOrigin[1] = mImageCenter[1] - mOutputSpacing[1] * (mOutputDimension[1] - 1.0) * 0.5;
        mOutputOrigin[2] = mImageCenter[2] + mImageToOutputDistance * 0.5;

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

    vtkSmartPointer<vtkImageData> VolumeDRR::GetOutput() const
    {
        return mOutputImageData;
    }
}
