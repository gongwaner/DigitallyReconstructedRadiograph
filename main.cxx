#include "Algorithm/VolumeDRR.h"
#include "Algorithm/MeshDRR.h"

#include <vtkImageData.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkImageIterator.h>

#include <filesystem>

#include "CommonUtility/IO/IOUtil.h"
#include "CommonUtility/Test/TestUtil.h"
#include "CommonUtility/Mesh/MeshUtil.h"
#include "CommonUtility/Visualization/VisualizationUtil.h"


void TestVolumeDRR()
{
    const auto dicomFolder = TestUtil::GetDataDir() / "pelvis";
    auto imageData = IOUtil::ReadImageData(dicomFolder.string().c_str());

    const auto imageDimension = imageData->GetDimensions();
    const auto imageSpacing = imageData->GetSpacing();
    const auto scalarType = imageData->GetScalarTypeAsString();
    printf("Input image dimension: %d, %d, %d\nimage spacing: %f, %f, %f\nscalar type = %s\n",
           imageDimension[0], imageDimension[1], imageDimension[2],
           imageSpacing[0], imageSpacing[1], imageSpacing[2], scalarType);

    const bool visualizeCT = false;
    if(visualizeCT)
        VisualizationUtil::VisualizeImageData(imageData);

    const int executionCnt = 1;
    long long totalExecutionTime = 0;
    vtkSmartPointer<vtkImageData> outputImage;
    for(int i = 0; i < executionCnt; ++i)
    {
        auto start = std::chrono::high_resolution_clock::now();

        Algorithm::VolumeDRR drr;
        drr.SetInputImage(imageData);
        drr.SetImageRotation(90.0f, 0.0f, 0.0f);
        drr.SetSourceToImageDistance(500);
        drr.SetOutputImageDimension(imageDimension[0], imageDimension[1], 1);
        drr.SetOutputSpacing(imageSpacing[0], imageSpacing[1], 1.0);
        drr.SetThreshold(-10);
        drr.Update();

        outputImage = drr.GetOutput();

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

        std::cout << "volume ddr takes " << duration.count() << " ms" << std::endl;

        totalExecutionTime += duration.count();
    }
    const auto avgExecutionTime = (double) totalExecutionTime / (double) executionCnt;
    printf("\n%i average execution time:%f ms\n", executionCnt, avgExecutionTime);

    const auto outFile = TestUtil::GetOutputDir().append("drr_volume.png");
    std::cout << "outFile: " << outFile << std::endl;
    IOUtil::WritePng(outFile.string().c_str(), outputImage);
}

void TestMeshDRR()
{
    const auto fileDir = TestUtil::GetDataDir().append("mesh").append("bunny.stl");
    auto polyData = IOUtil::ReadMesh(fileDir.string().c_str());

    auto dimension = MeshUtil::GetMeshDimension(polyData);
    std::cout << "dimension x = " << dimension[0] << ", dimensionY = " << dimension[1] << ", dimensionZ= " << dimension[2] << std::endl;

    auto start = std::chrono::high_resolution_clock::now();

    Algorithm::MeshDRR drr;
    drr.SetInputPolyData(polyData);
    drr.SetSourceToMeshDistance(150);
    drr.SetAttenuationCoefficient(1000);
    drr.SetOutputImageDimension(512, 512, 1);
    drr.Update();

    auto outputImage = drr.GetOutput();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    std::cout << "mesh ddr takes " << duration.count() << " ms" << std::endl;

    const auto outFile = TestUtil::GetOutputDir().append("drr_mesh.png");
    std::cout << "outFile: " << outFile << std::endl;
    IOUtil::WritePng(outFile.string().c_str(), outputImage);
}

int main(int argc, char* argv[])
{
    TestVolumeDRR();
    TestMeshDRR();

    return EXIT_SUCCESS;
}
