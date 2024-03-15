import dicom2nifti

dir_dicom = "/Users/mnlmrc/Downloads/DICOM/20240313/2024_03_13_SMP1_100/1.CF65B1BD/anatomical"
dir_output = f"{dir_dicom}/anatomical.nii"

# dicom2nifti.convert_directory(dir_dicom, dir_output, compression=True, reorient=True)
dicom2nifti.convert_dicom.dicom_series_to_nifti(dir_dicom, dir_output, reorient_nifti=True)
