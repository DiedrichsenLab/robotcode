import dicom2nifti

dir_dicom = "/Users/mnlmrc/Downloads/DICOM/20240313/2024_03_13_SMP1_100/1.CF65B1BD/7"
dir_output = dir_dicom

dicom2nifti.convert_directory(dir_dicom, dir_output, compression=False, reorient=True)

