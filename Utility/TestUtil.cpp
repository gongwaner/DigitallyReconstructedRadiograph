#include "TestUtil.h"

#include <vtkImageData.h>
#include <vtkNamedColors.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkActor2D.h>
#include <vtkImageViewer2.h>
#include <vtkInteractorStyleImage.h>
#include <vtkTextMapper.h>
#include <vtkTextProperty.h>

#include <sstream>


namespace TestUtil
{
    // Helper class to format slice status message.
    namespace StatusMessage
    {
        std::string Format(int slice, int maxSlice)
        {
            std::stringstream tmp;
            tmp << "Slice Number  " << slice + 1 << "/" << maxSlice + 1;
            return tmp.str();
        }
    }

    class ImageInteractorStyle : public vtkInteractorStyleImage
    {
    public:
        static ImageInteractorStyle* New();

    vtkTypeMacro(ImageInteractorStyle, vtkInteractorStyleImage);

    protected:
        vtkImageViewer2* mImageViewer;
        vtkTextMapper* mStatusMapper;
        int mCurrentSlice;
        int mMinSlice;
        int mMaxSlice;

    public:
        void SetImageViewer(vtkImageViewer2* imageViewer)
        {
            mImageViewer = imageViewer;
            mMinSlice = imageViewer->GetSliceMin();
            mMaxSlice = imageViewer->GetSliceMax();
            mCurrentSlice = mMinSlice;
            cout << "Slicer: Min = " << mMinSlice << ", Max = " << mMaxSlice << std::endl;
        }

        void SetStatusMapper(vtkTextMapper* statusMapper)
        {
            mStatusMapper = statusMapper;
        }

    protected:
        void MoveSliceForward()
        {
            if(mCurrentSlice < mMaxSlice)
            {
                mCurrentSlice += 1;
                std::cout << "MoveSliceForward::Slice = " << mCurrentSlice << std::endl;
                mImageViewer->SetSlice(mCurrentSlice);

                const auto msg = StatusMessage::Format(mCurrentSlice, mMaxSlice);
                mStatusMapper->SetInput(msg.c_str());
                mImageViewer->Render();
            }
        }

        void MoveSliceBackward()
        {
            if(mCurrentSlice > mMinSlice)
            {
                mCurrentSlice -= 1;
                cout << "MoveSliceBackward::Slice = " << mCurrentSlice << std::endl;
                mImageViewer->SetSlice(mCurrentSlice);

                const auto msg = StatusMessage::Format(mCurrentSlice, mMaxSlice);
                mStatusMapper->SetInput(msg.c_str());
                mImageViewer->Render();
            }
        }

        void OnKeyDown() override
        {
            std::string key = this->GetInteractor()->GetKeySym();
            if(key == "Up")
            {
                MoveSliceForward();
            }
            else if(key == "Down")
            {
                MoveSliceBackward();
            }

            vtkInteractorStyleImage::OnKeyDown();
        }

        void OnMouseWheelForward() override
        {
            MoveSliceForward();
            // don't forward events, otherwise the image will be zoomed
            // in case another interactorstyle is used (e.g. trackballstyle, ...)
            // vtkInteractorStyleImage::OnMouseWheelForward();
        }

        void OnMouseWheelBackward() override
        {
            if(mCurrentSlice > mMinSlice)
            {
                MoveSliceBackward();
            }
            // don't forward events, otherwise the image will be zoomed
            // in case another interactorstyle is used (e.g. trackballstyle, ...)
            // vtkInteractorStyleImage::OnMouseWheelBackward();
        }
    };

    vtkStandardNewMacro(ImageInteractorStyle);

    void VisualizeImageData(vtkImageData* imageData)
    {
        auto imageViewer = vtkSmartPointer<vtkImageViewer2>::New();
        imageViewer->SetInputData(imageData);

        // Slice status message.
        auto sliceTextProp = vtkSmartPointer<vtkTextProperty>::New();
        sliceTextProp->SetFontFamilyToCourier();
        sliceTextProp->SetFontSize(20);
        sliceTextProp->SetVerticalJustificationToBottom();
        sliceTextProp->SetJustificationToLeft();

        auto sliceTextMapper = vtkSmartPointer<vtkTextMapper>::New();
        const auto msg = StatusMessage::Format(imageViewer->GetSliceMin(), imageViewer->GetSliceMax());
        sliceTextMapper->SetInput(msg.c_str());
        sliceTextMapper->SetTextProperty(sliceTextProp);

        auto sliceTextActor = vtkSmartPointer<vtkActor2D>::New();
        sliceTextActor->SetMapper(sliceTextMapper);
        sliceTextActor->SetPosition(15, 10);

        // Usage hint message.
        auto usageTextProp = vtkSmartPointer<vtkTextProperty>::New();
        usageTextProp->SetFontFamilyToCourier();
        usageTextProp->SetFontSize(14);
        usageTextProp->SetVerticalJustificationToTop();
        usageTextProp->SetJustificationToLeft();

        auto usageTextMapper = vtkSmartPointer<vtkTextMapper>::New();
        usageTextMapper->SetInput(
                "- Slice with mouse wheel\n  or Up/Down-Key\n- Zoom with pressed right\n "
                " mouse button while dragging");
        usageTextMapper->SetTextProperty(usageTextProp);

        auto usageTextActor = vtkSmartPointer<vtkActor2D>::New();
        usageTextActor->SetMapper(usageTextMapper);
        usageTextActor->GetPositionCoordinate()
                ->SetCoordinateSystemToNormalizedDisplay();
        usageTextActor->GetPositionCoordinate()->SetValue(0.05, 0.95);

        // Create an interactor with our own style (inherit from
        // vtkInteractorStyleImage) in order to catch mousewheel and key events.
        auto renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

        auto myInteractorStyle = vtkSmartPointer<ImageInteractorStyle>::New();

        // Make imageviewer2 and sliceTextMapper visible to our interactorstyle
        // to enable slice status message updates when scrolling through the slices.
        myInteractorStyle->SetImageViewer(imageViewer);
        myInteractorStyle->SetStatusMapper(sliceTextMapper);

        imageViewer->SetupInteractor(renderWindowInteractor);
        // Make the interactor use our own interactorstyle
        // cause SetupInteractor() is defining it's own default interatorstyle
        // this must be called after SetupInteractor().
        renderWindowInteractor->SetInteractorStyle(myInteractorStyle);
        // Sdd slice status message and usage hint message to the renderer.
        imageViewer->GetRenderer()->AddActor2D(sliceTextActor);
        imageViewer->GetRenderer()->AddActor2D(usageTextActor);

        // Initialize rendering and interaction.
        auto colors = vtkSmartPointer<vtkNamedColors>::New();

        imageViewer->Render();
        imageViewer->GetRenderer()->ResetCamera();
        imageViewer->GetRenderer()->SetBackground(colors->GetColor3d("SlateGray").GetData());
        imageViewer->GetRenderWindow()->SetSize(800, 800);
        imageViewer->GetRenderWindow()->SetWindowName("ReadDICOMSeries");
        imageViewer->Render();
        renderWindowInteractor->Start();
    }
}
