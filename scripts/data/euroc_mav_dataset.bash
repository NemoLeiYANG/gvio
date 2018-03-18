#!/bin/bash
set -e  # Exit on first error
DOWNLOAD_PATH="/data/euroc_mav/raw"
BASE_URL="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
# DATASETS=("/machine_hall/MH_01_easy"
#           "/machine_hall/MH_02_easy"
#           "/machine_hall/MH_03_medium"
#           "/machine_hall/MH_04_difficult"
#           "/machine_hall/MH_05_difficult"
DATASETS=("vicon_room1/V1_01_easy"
          "vicon_room1/V1_02_medium"
          "vicon_room1/V1_03_difficult"
          "vicon_room2/V2_01_easy"
          "vicon_room2/V2_02_medium"
          "vicon_room2/V2_03_difficult")

cd ${DOWNLOAD_PATH}

for DATASET in "${DATASETS[@]}"; do
  # Download
  DATASET_NAME=$(basename "${DATASET}")
  if [ ! -f "$DATASET_NAME.zip" ]; then
    echo "Downloading [${DATASET}]"
    wget "${BASE_URL}/${DATASET}/${DATASET_NAME}.zip"
  fi

  # Extract
  unzip "$DATASET_NAME.zip"
  mv mav0 "$DATASET_NAME"
done
