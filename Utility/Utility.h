#pragma once

#include <vtkVector.h>

class vtkImageData;
class vtkMatrix3x3;
class vtkMatrix4x4;

namespace IOUtil
{
    vtkSmartPointer<vtkImageData> ReadImageDataFromFolder(const char* folder);
    void WritePng(const char* fileDir, vtkImageData* imageData);
}

namespace TransformUtil
{
    vtkSmartPointer<vtkMatrix3x3> GetRotationMatrix(double rotationAngleX, double rotationAngleY, double rotationAngleZ);
    vtkVector3d GetOffset(const vtkVector3d& center, const vtkVector3d& translation, const vtkMatrix3x3* rotationMatrix);
    vtkSmartPointer<vtkMatrix4x4> GetTransformationMatrix(const vtkVector3d& center, const vtkVector3d& translation,
                                                          double rotationAngleX, double rotationAngleY, double rotationAngleZ);
    vtkVector3d GetTransformedPoint(const vtkVector3d& point, vtkMatrix4x4* transform);
}
