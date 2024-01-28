#include "Utility.h"

#include <vtkDICOMImageReader.h>

#include <filesystem>

namespace IOUtil
{
    bool PathExist(const char* fileDir)
    {
        auto path = std::filesystem::path(fileDir);
        if(!std::filesystem::exists(path))
        {
            std::cerr << "ERROR: Directory " << path.string() << " does not exist!" << std::endl;
            return false;
        }

        return true;
    }

    vtkSmartPointer<vtkImageData> ReadImageDataFromFolder(const char* folder)
    {
        if(!PathExist(folder))
        {
            return nullptr;
        }

        auto path = std::filesystem::path(folder);
        std::cout << "reading " << path << std::endl;

        auto reader = vtkSmartPointer<vtkDICOMImageReader>::New();
        reader->SetDirectoryName(folder);
        reader->Update();

        return reader->GetOutput();
    }
}

namespace TransformUtil
{
    //ref: https://github.com/hinerm/ITK/blob/21f48c6d98e21ecece09be16a747221d7094d8a9/Modules/Core/Transform/include/itkEuler3DTransform.hxx#L202C1-L202C1
    vtkSmartPointer<vtkMatrix3x3> GetRotationMatrix(const double rotationAngleX, const double rotationAngleY, const double rotationAngleZ)
    {
        const double one = 1.0;
        const double zero = 0.0;

        auto rotationX = vtkSmartPointer<vtkMatrix3x3>::New();
        {
            const double cosX = std::cos(rotationAngleX);
            const double sinX = std::sin(rotationAngleX);

            rotationX->SetElement(0, 0, one);
            rotationX->SetElement(0, 1, zero);
            rotationX->SetElement(0, 2, zero);
            rotationX->SetElement(1, 0, zero);
            rotationX->SetElement(1, 1, cosX);
            rotationX->SetElement(1, 2, -sinX);
            rotationX->SetElement(2, 0, zero);
            rotationX->SetElement(2, 1, sinX);
            rotationX->SetElement(2, 2, cosX);
        }

        auto rotationY = vtkSmartPointer<vtkMatrix3x3>::New();
        {
            const double cosY = std::cos(rotationAngleY);
            const double sinY = std::sin(rotationAngleY);

            rotationY->SetElement(0, 0, cosY);
            rotationY->SetElement(0, 1, zero);
            rotationY->SetElement(0, 2, sinY);
            rotationY->SetElement(1, 0, zero);
            rotationY->SetElement(1, 1, one);
            rotationY->SetElement(1, 2, zero);
            rotationY->SetElement(2, 0, -sinY);
            rotationY->SetElement(2, 1, zero);
            rotationY->SetElement(2, 2, cosY);
        }

        auto rotationZ = vtkSmartPointer<vtkMatrix3x3>::New();
        {
            const double cosZ = std::cos(rotationAngleZ);
            const double sinZ = std::sin(rotationAngleZ);

            rotationZ->SetElement(0, 0, cosZ);
            rotationZ->SetElement(0, 1, -sinZ);
            rotationZ->SetElement(0, 2, zero);
            rotationZ->SetElement(1, 0, sinZ);
            rotationZ->SetElement(1, 1, cosZ);
            rotationZ->SetElement(1, 2, zero);
            rotationZ->SetElement(2, 0, zero);
            rotationZ->SetElement(2, 1, zero);
            rotationZ->SetElement(2, 2, one);
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

    vtkSmartPointer<vtkMatrix4x4> GetTransformationMatrix(const vtkVector3d& center, const vtkVector3d& translation,
                                                          const double rotationAngleX, const double rotationAngleY, const double rotationAngleZ)
    {
        auto rotationMatrix = GetRotationMatrix(rotationAngleX, rotationAngleY, rotationAngleZ);
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

    vtkVector3d GetTransformedPoint(const vtkVector3d& point, vtkMatrix4x4* transform)
    {
        vtkVector4d in(point[0], point[1], point[2], 1);
        double out[4];
        transform->MultiplyPoint(in.GetData(), out);

        return {out[0], out[1], out[2]};
    }
}
