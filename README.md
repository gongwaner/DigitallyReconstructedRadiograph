# DigitallyReconstructedRadiograph


Port ITK Digitally Reconstructed Radiograph(DRR) to VTK with optimization and refactor.<br/>
 * On windows platform, use std::execution::par to accelerate DRR generation
 * Add orthographic projection DRR
 * Add Mesh DRR

<br/>Dicom data acquired from [The NLM Visible Human Project](https://central.xnat.org/app/template/XDATScreen_report_xnat_projectData.vm/search_element/xnat:projectData/search_field/xnat:projectData.ID/search_value/HumanCT)

<br/>**Results**
<br/>left:3D Volume Data(Rendered by VolView); right: 2D x-ray image
![image](https://github.com/gongwaner/DigitallyReconstructedRadiograph/assets/29704759/c1b2f6c8-9783-4ad7-96d1-290029faa325)
![image](https://github.com/gongwaner/DigitallyReconstructedRadiograph/assets/29704759/700c4ceb-11fb-4068-b4df-5e14ba544d2a)
