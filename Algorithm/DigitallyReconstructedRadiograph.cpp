#include "DigitallyReconstructedRadiograph.h"

#include "RayCastInterpolateImageFunction.h"
#include "../Utility/Utility.h"

#include <vtkImageData.h>
#include <vtkImageIterator.h>
#include <vtkImageCast.h>

#include <execution>


namespace Algorithm
{
    void DigitallyReconstructedRadiograph::SetInputImage(vtkImageData* imageData)
    {
        mImageData = imageData;

        const auto imageOrigin = mImageData->GetOrigin();
        const auto imageSpacing = mImageData->GetSpacing();
        const auto imageDimension = mImageData->GetDimensions();

        if(mDebug)
        {
            printf("Input image dimension: %d, %d, %d, spacing: %f, %f, %f\n", imageDimension[0], imageDimension[1],
                   imageDimension[2], imageSpacing[0], imageSpacing[1], imageSpacing[2]);
        }

        for(int i = 0; i < 3; ++i)
            mImageCenter[i] = imageOrigin[i] + imageSpacing[i] * imageDimension[i] * 0.5;
    }

    void DigitallyReconstructedRadiograph::SetThreshold(const double threshold)
    {
        mThreshold = threshold;
    }

    void DigitallyReconstructedRadiograph::SetSourceToImageDistance(const float distance)
    {
        mSourceToImageDistance = distance;
    }

    void DigitallyReconstructedRadiograph::SetDefaultPixelValue(const double value)
    {
        mDefaultPixelValue = value;
    }

    void DigitallyReconstructedRadiograph::SetImageRotation(const float rotX, const float rotY, const float rotZ)
    {
        mRotX = rotX;
        mRotY = rotY;
        mRotZ = rotZ;
    }

    void DigitallyReconstructedRadiograph::SetOutputImageDimension(const int dimX, const int dimY, const int dimZ)
    {
        mOutputDimension[0] = dimX;
        mOutputDimension[1] = dimY;
        mOutputDimension[2] = dimZ;
    }

    void DigitallyReconstructedRadiograph::SetOutputImageDimension(const int dimensions[3])
    {
        for(int i = 0; i < 3; ++i)
            mOutputDimension[i] = dimensions[i];
    }

    void DigitallyReconstructedRadiograph::SetOutputSpacing(const double spacingX, const double spacingY,
                                                            const double spacingZ)
    {
        mOutputSpacing[0] = spacingX;
        mOutputSpacing[1] = spacingY;
        mOutputSpacing[2] = spacingZ;
    }

    void DigitallyReconstructedRadiograph::SetOutputSpacing(const double spacing[3])
    {
        for(int i = 0; i < 3; ++i)
            mOutputSpacing[i] = spacing[i];
    }

    void DigitallyReconstructedRadiograph::ComputeCenteredEuler3DTransform()
    {
        const double degreeToRadians = (std::atan(1.0) * 4.0) / 180.0;
        auto angleX = degreeToRadians * mRotX;
        auto angleY = degreeToRadians * mRotY;
        auto angleZ = degreeToRadians * mRotZ;

        mTransform = TransformUtil::GetTransformationMatrix(mImageCenter, mTranslation, angleX, angleY, angleZ);
    }

    void DigitallyReconstructedRadiograph::ComputeFocalPoint()
    {
        // The ray cast interpolator needs to know the initial position of the ray source or focal point.
        // In this example we place the input volume at the origin and halfway between the ray source
        // and the screen. The distance between the ray source and the screen is {mSourceToImageDistance}
        // and specified by the user.
        mFocalPoint[0] = mImageCenter[0];
        mFocalPoint[1] = mImageCenter[1];
        mFocalPoint[2] = mImageCenter[2] - mSourceToImageDistance * 0.5;
    }

    //ref: https://github.com/InsightSoftwareConsortium/ITK/blob/12328e1d2ff8becab001654dccbb393657521691/Modules/Filtering/ImageGrid/include/itkResampleImageFilter.hxx#L369
    vtkSmartPointer<vtkImageData> DigitallyReconstructedRadiograph::GenerateOutputImageDataSeq() const
    {
        if(mDebug)
            printf("Output image dimension: %d, %d, %d, spacing: %f, %f, %f\n", mOutputDimension[0],
                   mOutputDimension[1], mOutputDimension[2], mOutputSpacing[0], mOutputSpacing[1], mOutputSpacing[2]);

        auto outputImage = vtkSmartPointer<vtkImageData>::New();
        outputImage->SetOrigin(mOutputOrigin.GetData());
        outputImage->SetDimensions(mOutputDimension);
        outputImage->SetSpacing(mOutputSpacing);
        outputImage->AllocateScalars(mImageData->GetScalarType(), 1);

        RayCastInterpolateImageFunction interpolator;
        interpolator.SetImageData(mImageData);
        interpolator.SetTransform(mTransform);
        interpolator.SetFocalPoint(mFocalPoint);
        interpolator.SetThreshold(mThreshold);

        // Retrieve the entries from the image data and print them to the screen
        for(int z = 0; z < outputImage->GetDimensions()[2]; z++)
        {
            for(int y = 0; y < outputImage->GetDimensions()[1]; y++)
            {
                for(int x = 0; x < outputImage->GetDimensions()[0]; x++)
                {
                    //current output point position
                    double outputPoint[3];
                    outputImage->TransformIndexToPhysicalPoint(x, y, z, outputPoint);

                    //corresponding input pixel position
                    auto inputPoint = TransformUtil::GetTransformedPoint(vtkVector3d(outputPoint), mTransform);
                    double inputIndex[3];
                    mImageData->TransformPhysicalPointToContinuousIndex(inputPoint.GetData(), inputIndex);

                    //evaluate input at right position and copy to the output
                    auto outPixel = static_cast<short*>(outputImage->GetScalarPointer(x, y, z));

                    //if(interpolator->IsInsideBuffer(inputIndex)
                    *outPixel = (short) interpolator.EvaluateAtContinuousIndex(inputIndex);
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
    vtkSmartPointer<vtkImageData> DigitallyReconstructedRadiograph::GenerateOutputImageDataPar() const
    {
        if(mDebug)
            printf("Output image dimension: %d, %d, %d, spacing: %f, %f, %f\n", mOutputDimension[0],
                   mOutputDimension[1], mOutputDimension[2], mOutputSpacing[0], mOutputSpacing[1], mOutputSpacing[2]);

        auto outputImage = vtkSmartPointer<vtkImageData>::New();
        outputImage->SetOrigin(mOutputOrigin.GetData());
        outputImage->SetDimensions(mOutputDimension);
        outputImage->SetSpacing(mOutputSpacing);
        outputImage->AllocateScalars(mImageData->GetScalarType(), 1);

        RayCastInterpolateImageFunction interpolator;
        interpolator.SetImageData(mImageData);
        interpolator.SetTransform(mTransform);
        interpolator.SetFocalPoint(mFocalPoint);
        interpolator.SetThreshold(mThreshold);

        auto start = std::chrono::high_resolution_clock::now();

        const auto vectorSize = mOutputDimension[0] * mOutputDimension[1] * mOutputDimension[2];
        std::vector<vtkVector3d> inputIndicesVec;
        inputIndicesVec.reserve(vectorSize);

        std::vector<short*> outPixelsVec;
        outPixelsVec.reserve(vectorSize);

        for(int z = 0; z < outputImage->GetDimensions()[2]; z++)
        {
            for(int y = 0; y < outputImage->GetDimensions()[1]; y++)
            {
                for(int x = 0; x < outputImage->GetDimensions()[0]; x++)
                {
                    //current output point position
                    double outputPoint[3];
                    outputImage->TransformIndexToPhysicalPoint(x, y, z, outputPoint);

                    //corresponding input pixel position
                    const auto inputPoint = TransformUtil::GetTransformedPoint(vtkVector3d(outputPoint), mTransform);
                    double inputIndex[3];
                    mImageData->TransformPhysicalPointToContinuousIndex(inputPoint.GetData(), inputIndex);
                    inputIndicesVec.emplace_back(inputIndex);

                    auto outPixel = static_cast<short*>(outputImage->GetScalarPointer(x, y, z));
                    outPixelsVec.push_back(outPixel);
                }
            }
        }

        std::vector<short> resultPixelValueVec(vectorSize);
        std::transform(std::execution::par, inputIndicesVec.begin(), inputIndicesVec.end(), resultPixelValueVec.begin(),
                       [&interpolator](const vtkVector3d& inputIndex)
                       {
                           return (short) interpolator.EvaluateAtContinuousIndex(inputIndex.GetData());
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

    void DigitallyReconstructedRadiograph::Update()
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

    vtkSmartPointer<vtkImageData> DigitallyReconstructedRadiograph::GetOutput() const
    {
        return mOutputImageData;
    }
}
