#pragma once

#include <vtkVector.h>

class vtkPolyData;
class vtkImageData;
class vtkMatrix3x3;
class vtkMatrix4x4;

namespace IOUtil
{
    vtkSmartPointer<vtkPolyData> ReadMesh(const char* fileDir);
    vtkSmartPointer<vtkImageData> ReadImageDataFromFolder(const char* folder);
    void WritePng(const char* fileDir, vtkImageData* imageData);
}

namespace CommonUtil
{
    void Print(const std::string& msg, const double vec[3]);
    void Print(const std::string& msg, const vtkVector3d& vec);
}

namespace TransformUtil
{
    vtkSmartPointer<vtkMatrix3x3> GetRotationMatrix(double rotationAngleX, double rotationAngleY, double rotationAngleZ);
    vtkVector3d GetOffset(const vtkVector3d& center, const vtkVector3d& translation, const vtkMatrix3x3* rotationMatrix);
    vtkSmartPointer<vtkMatrix4x4> GetTransformationMatrix(const vtkVector3d& center, const vtkVector3d& translation, const vtkVector3d& rotation);
    vtkVector3d GetTransformedPoint(const double point[3], vtkMatrix4x4* transform);
    vtkVector3d GetTransformedPoint(const vtkVector3d& point, vtkMatrix4x4* transform);
}

namespace MeshUtil
{
    vtkVector3d GetMeshDimension(vtkPolyData* polyData);
}
