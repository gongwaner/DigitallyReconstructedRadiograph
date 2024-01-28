#include "Algorithm/DigitallyReconstructedRadiograph.h"

#include "Utility/Utility.h"
#include "Utility/TestUtil.h"

#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkImageIterator.h>

#include <filesystem>

int main(int argc, char* argv[])
{
    auto cwd = std::filesystem::current_path();
    printf("current working dir: %s\n", cwd.string().c_str());

#ifdef _WIN32
    auto projectDir = cwd.parent_path();
    printf("projectDir dir: %s\n", projectDir.string().c_str());
#elif __APPLE__
    auto projectDir = cwd.parent_path().parent_path().parent_path().parent_path();
    printf("projectDir dir: %s\n", projectDir.string().c_str());
#else
    printf("Operating system not supported!\n");
#endif

    const auto dataDir = projectDir.append("Data");
    printf("dataDir dir: %s\n", dataDir.string().c_str());
    if(!std::filesystem::is_directory(dataDir))
    {
        printf("data directory does not exist!\n");
        return 1;
    }

    auto dicomFolder = dataDir;
    dicomFolder = dicomFolder.append("pelvis");
    auto imageData = IOUtil::ReadImageDataFromFolder(dicomFolder.string().c_str());

    const auto imageDimension = imageData->GetDimensions();
    const auto imageSpacing = imageData->GetSpacing();
    const auto scalarType = imageData->GetScalarTypeAsString();
    printf("Input image dimension: %d, %d, %d\nimage spacing: %f, %f, %f\nscalar type = %s\n", imageDimension[0], imageDimension[1],
           imageDimension[2], imageSpacing[0], imageSpacing[1], imageSpacing[2], scalarType);

    const bool visualizeCT = false;
    if(visualizeCT)
        TestUtil::VisualizeImageData(imageData);

    const int executionCnt = 1;
    long long totalExecutionTime = 0;
    vtkSmartPointer<vtkImageData> outputImage;
    for(int i = 0; i < executionCnt; ++i)
    {
        auto start = std::chrono::high_resolution_clock::now();

        Algorithm::DigitallyReconstructedRadiograph ddr;
        ddr.SetInputImage(imageData);
        ddr.SetImageRotation(90.0f, 0.0f, 0.0f);
        ddr.SetOutputImageDimension(imageDimension[0], imageDimension[1], 1);
        ddr.SetOutputSpacing(imageSpacing[0], imageSpacing[1], 1.0);
        ddr.SetThreshold(180);
        ddr.Update();

        outputImage = ddr.GetOutput();

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

        std::cout << "ddr takes " << duration.count() << " ms" << std::endl;

        totalExecutionTime += duration.count();
    }
    const auto avgExecutionTime = (double) totalExecutionTime / (double) executionCnt;
    printf("\n%i average execution time:%f ms\n", executionCnt, avgExecutionTime);
    
    auto outDir = dataDir;
    outDir = outDir.append("output");
    const auto outFile = outDir.append("drr.png");
    std::cout << "outFile: " << outFile << std::endl;
    TestUtil::WritePng(outFile.string().c_str(), outputImage);

    return EXIT_SUCCESS;
}
