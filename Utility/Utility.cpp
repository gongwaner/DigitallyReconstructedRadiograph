#include "Utility.h"

#include <vtkSmartPointer.h>
#include <vtkDICOMImageReader.h>
#include <vtkPNGWriter.h>
#include <vtkPolyData.h>
#include <vtkImageData.h>
#include <vtkMatrix3x3.h>
#include <vtkMatrix4x4.h>

#include <filesystem>
#include <vtkSTLReader.h>
#include <vtkPointData.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkPolyDataReader.h>
#include <vtkImageCast.h>


namespace IOUtil
{
    bool PathExist(const char* fileDir)
    {
        const auto path = std::filesystem::path(fileDir);
        if(!std::filesystem::exists(path))
        {
            std::cerr << "ERROR: Directory " << path.string() << " does not exist!" << std::endl;
            return false;
        }

        return true;
    }

    bool IsValidDirectory(const char* fileDir)
    {
        const auto path = std::filesystem::path(fileDir);
        const auto parentDir = path.parent_path();
        if(!std::filesystem::is_directory(parentDir))
        {
            std::cerr << "ERROR: Directory " << parentDir.string() << " does not exist!" << std::endl;
            return false;
        }

        return true;
    }

    vtkSmartPointer<vtkPolyData> ReadMesh(const char* fileDir)
    {
        if(!PathExist(fileDir))
            throw std::runtime_error("ReadPolyData(). Path does not exist!");

        const auto path = std::filesystem::path(fileDir);
        const auto extension = std::filesystem::path(fileDir).extension();
        std::cout << "Reading " << path << std::endl;

        vtkSmartPointer<vtkPolyData> polyData;
        if(extension == ".ply")
        {
            auto reader = vtkSmartPointer<vtkPLYReader>::New();
            reader->SetFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if(extension == ".obj")
        {
            auto reader = vtkSmartPointer<vtkOBJReader>::New();
            reader->SetFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if(extension == ".stl")
        {
            auto reader = vtkSmartPointer<vtkSTLReader>::New();
            reader->SetFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if(extension == ".vtk")
        {
            auto reader = vtkSmartPointer<vtkPolyDataReader>::New();
            reader->SetFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else
        {
            throw std::runtime_error("ReadPolyData(). ERROR: unsupported file extension!");
        }

        printf("Poly data vertices cnt: %lld, cells cnt: %lld\n", polyData->GetNumberOfPoints(), polyData->GetNumberOfCells());

        return polyData;
    }

    vtkSmartPointer<vtkImageData> ReadImageDataFromFolder(const char* folder)
    {
        if(!PathExist(folder))
        {
            throw std::runtime_error("ReadImageDataFromFolder(). Folder does not exist!");
        }

        const auto path = std::filesystem::path(folder);
        std::cout << "reading " << path << std::endl;

        auto reader = vtkSmartPointer<vtkDICOMImageReader>::New();
        reader->SetDirectoryName(folder);
        reader->Update();

        return reader->GetOutput();
    }

    void WritePng(const char* fileDir, vtkImageData* imageData, const bool isUnsignedShort)
    {
        if(!IsValidDirectory(fileDir))
        {
            std::cout << "Creating Directory..." << std::endl;

            const auto path = std::filesystem::path(fileDir);
            const auto parentDir = path.parent_path();
            std::filesystem::create_directories(parentDir);
        }

        //png only supports unsigned char/short so cast it before export
        auto castFilter = vtkSmartPointer<vtkImageCast>::New();
        castFilter->SetInputData(imageData);
        isUnsignedShort ? castFilter->SetOutputScalarTypeToUnsignedShort() : castFilter->SetOutputScalarTypeToUnsignedChar();
        castFilter->Update();
        auto output = castFilter->GetOutput();

        auto writer = vtkSmartPointer<vtkPNGWriter>::New();
        writer->SetFileName(fileDir);
        writer->SetInputData(output);
        writer->Write();
    }
}

namespace CommonUtil
{
    void Print(const std::string& msg, const double vec[3])
    {
        std::cout << msg << vec[0] << ", " << vec[1] << ", " << vec[2] << std::endl;
    }

    void Print(const std::string& msg, const vtkVector3d& vec)
    {
        Print(msg, vec.GetData());
    }
}

namespace TransformUtil
{
    //ref: https://github.com/hinerm/ITK/blob/21f48c6d98e21ecece09be16a747221d7094d8a9/Modules/Core/Transform/include/itkEuler3DTransform.hxx#L202C1-L202C1
    vtkSmartPointer<vtkMatrix3x3> GetRotationMatrix(const double rotationAngleX, const double rotationAngleY, const double rotationAngleZ)
    {
        auto rotationX = vtkSmartPointer<vtkMatrix3x3>::New();
        {
            const double cosX = std::cos(rotationAngleX);
            const double sinX = std::sin(rotationAngleX);

            rotationX->SetElement(0, 0, 1);
            rotationX->SetElement(0, 1, 0);
            rotationX->SetElement(0, 2, 0);
            rotationX->SetElement(1, 0, 0);
            rotationX->SetElement(1, 1, cosX);
            rotationX->SetElement(1, 2, -sinX);
            rotationX->SetElement(2, 0, 0);
            rotationX->SetElement(2, 1, sinX);
            rotationX->SetElement(2, 2, cosX);
        }

        auto rotationY = vtkSmartPointer<vtkMatrix3x3>::New();
        {
            const double cosY = std::cos(rotationAngleY);
            const double sinY = std::sin(rotationAngleY);

            rotationY->SetElement(0, 0, cosY);
            rotationY->SetElement(0, 1, 0);
            rotationY->SetElement(0, 2, sinY);
            rotationY->SetElement(1, 0, 0);
            rotationY->SetElement(1, 1, 1);
            rotationY->SetElement(1, 2, 0);
            rotationY->SetElement(2, 0, -sinY);
            rotationY->SetElement(2, 1, 0);
            rotationY->SetElement(2, 2, cosY);
        }

        auto rotationZ = vtkSmartPointer<vtkMatrix3x3>::New();
        {
            const double cosZ = std::cos(rotationAngleZ);
            const double sinZ = std::sin(rotationAngleZ);

            rotationZ->SetElement(0, 0, cosZ);
            rotationZ->SetElement(0, 1, -sinZ);
            rotationZ->SetElement(0, 2, 0);
            rotationZ->SetElement(1, 0, sinZ);
            rotationZ->SetElement(1, 1, cosZ);
            rotationZ->SetElement(1, 2, 0);
            rotationZ->SetElement(2, 0, 0);
            rotationZ->SetElement(2, 1, 0);
            rotationZ->SetElement(2, 2, 1);
        }

        //compute Z*Y*X
        auto zyx = vtkSmartPointer<vtkMatrix3x3>::New();
        vtkMatrix3x3::Multiply3x3(rotationZ, rotationY, zyx);
        vtkMatrix3x3::Multiply3x3(zyx, rotationX, zyx);
        zyx->Invert();//by default VTK uses pre-multiply but we need post multiply

        return zyx;
    }

    //ref: https://github.com/InsightSoftwareConsortium/ITK/blob/bf54f34df211145c38100ed262ee7a9740b51ba5/Modules/Core/Common/include/itkImageBase.h#L353
    vtkVector3d GetOffset(const vtkVector3d& center, const vtkVector3d& translation, const vtkMatrix3x3* rotationMatrix)
    {
        vtkVector3d offset;
        const int dimension = 3;
        for(int i = 0; i < dimension; ++i)
        {
            offset[i] = translation[i] + center[i];

            for(int j = 0; j < dimension; ++j)
            {
                offset[i] -= rotationMatrix->GetElement(i, j) * center[j];
            }
        }

        return offset;
    }

    vtkSmartPointer<vtkMatrix4x4> GetTransformationMatrix(const vtkVector3d& center, const vtkVector3d& translation, const vtkVector3d& rotation)
    {
        auto rotationMatrix = GetRotationMatrix(rotation[0], rotation[1], rotation[2]);
        auto offset = GetOffset(center, translation, rotationMatrix);

        auto transformationMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
        const int dimension = 3;
        //set rotation
        for(int i = 0; i < dimension; ++i)
        {
            for(int j = 0; j < dimension; ++j)
            {
                transformationMatrix->SetElement(i, j, rotationMatrix->GetElement(i, j));
            }
        }

        //set offset
        for(int i = 0; i < dimension; ++i)
        {
            transformationMatrix->SetElement(i, 3, offset[i]);
        }

        return transformationMatrix;
    }

    vtkVector3d GetTransformedPoint(const double point[3], vtkMatrix4x4* transform)
    {
        vtkVector4d in(point[0], point[1], point[2], 1);
        double out[4];
        transform->MultiplyPoint(in.GetData(), out);

        return {out[0], out[1], out[2]};
    }

    vtkVector3d GetTransformedPoint(const vtkVector3d& point, vtkMatrix4x4* transform)
    {
        return GetTransformedPoint(point.GetData(), transform);
    }

    vtkVector3d GetTransformedVector(const vtkVector3d& vec, vtkMatrix4x4* transformMat)
    {
        double in[4]{vec[0], vec[1], vec[2], 0};
        double out[4];
        transformMat->MultiplyPoint(in, out);

        return {out[0], out[1], out[2]};
    }
}

namespace MeshUtil
{
    vtkVector3d GetMeshDimension(vtkPolyData* polyData)
    {
        auto polyBounds = polyData->GetBounds();
        const auto dimensionX = polyBounds[1] - polyBounds[0];
        const auto dimensionY = polyBounds[3] - polyBounds[2];
        const auto dimensionZ = polyBounds[5] - polyBounds[4];

        return {dimensionX, dimensionY, dimensionZ};
    }
}
