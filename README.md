# DigitallyReconstructedRadiograph


Port ITK Digitally Reconstructed Radiograph(DRR) to VTK with optimization and refactor.<br/>
 * On windows platform, use std::execution::par to accelerate DRR generation
 * Add orthographic projection DRR
 * Add Mesh DRR

<br/>Dicom data acquired from [The NLM Visible Human Project](https://central.xnat.org/app/template/XDATScreen_report_xnat_projectData.vm/search_element/xnat:projectData/search_field/xnat:projectData.ID/search_value/HumanCT)

<br/>**Results**
<br/>left:3D Volume Data(Rendered by VolView); right: 2D x-ray image
<img width="1006" height="209" alt="image" src="https://github.com/user-attachments/assets/1e267ccd-6bf7-4e3a-945b-a72caf1b6357" />
