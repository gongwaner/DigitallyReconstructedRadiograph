#pragma once

#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkMatrix3x3.h>
#include <vtkMatrix4x4.h>
#include <vtkVector.h>

namespace IOUtil
{
    vtkSmartPointer<vtkImageData> ReadImageDataFromFolder(const char* folder);
}

namespace TransformUtil
{
    vtkSmartPointer<vtkMatrix3x3> GetRotationMatrix(const double rotationAngleX, const double rotationAngleY, const double rotationAngleZ);

    vtkVector3d GetOffset(const vtkVector3d& center, const vtkVector3d& translation, const vtkMatrix3x3* rotationMatrix);

    vtkSmartPointer<vtkMatrix4x4> GetTransformationMatrix(const vtkVector3d& center, const vtkVector3d& translation,
                                                          const double rotationAngleX, const double rotationAngleY, const double rotationAngleZ);

    vtkVector3d GetTransformedPoint(const vtkVector3d& point, vtkMatrix4x4* transform);
}

namespace ColorUtil
{
    int ConvertDoubleToUnsignedChar(double inValue);
}
