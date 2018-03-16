#!/bin/bash
set -e  # Exit on first error
DOWNLOAD_PATH="/data/euroc_mav/raw"
BASE_URL="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall"
DATASETS=("MH_01_easy"
          "MH_02_easy"
          "MH_03_medium"
          "MH_04_difficult"
          "MH_05_difficult"
          "V1_01_easy"
          "V1_02_medium"
          "V1_03_difficult"
          "V2_01_easy"
          "V2_02_medium"
          "V2_03_difficult")

cd ${DOWNLOAD_PATH}
for DATASET in "${DATASETS[@]}"; do
  echo "Downloading [${DATASET}]"
  wget "${BASE_URL}/${DATASET}/${DATASET}.zip"
  unzip ${DATASET}.zip
  mv mav0 ${DATASET}
done
