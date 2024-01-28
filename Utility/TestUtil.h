#pragma  once

#include <vtkImageData.h>

namespace TestUtil
{
    void VisualizeImageData(vtkImageData* imageData);

    void WritePng(const char* fileDir, vtkImageData* imageData);
}

