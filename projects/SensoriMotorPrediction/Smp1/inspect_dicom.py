import pydicom
import os

dicom_directory = "/Users/mnlmrc/Downloads/DICOM/20240313/2024_03_13_SMP1_100/1.CF65B1BD/anatomical"  # Change to your DICOM directory path
unique_series_ids = set()

# Iterate over each file in the directory
for filename in os.listdir(dicom_directory):
    if filename.endswith(".dcm"):
        filepath = os.path.join(dicom_directory, filename)
        try:
            # Read the DICOM file
            ds = pydicom.dcmread(filepath)
            # Add the Series Instance UID to the set
            unique_series_ids.add(ds.SeriesInstanceUID)
        except Exception as e:
            print(f"Error reading {filename}: {e}")

# Check how many unique Series Instance UIDs were found
if len(unique_series_ids) == 1:
    print("All files belong to the same series.")
else:
    print(f"Found {len(unique_series_ids)} different series. Consider organizing them into separate directories.")
