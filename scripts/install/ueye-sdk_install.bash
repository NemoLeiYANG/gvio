#!/usr/bin/bash
set -e

BASE_URL="https://en.ids-imaging.com"

MACHINE_TYPE=`uname -m`
if [ ${MACHINE_TYPE} == 'x86_64' ]; then
  DOWNLOAD_PATH=/usr/local/src

  # Go to output directory
  cd $DOWNLOAD_PATH
  mkdir -p ueye_sdk && cd ueye_sdk

  # Download SDK
  DOWNLOAD_URL="download-ueye-lin64.html?file=tl_files/downloads/uEye_SDK/driver"
  DOWNLOAD_FILE="uEye-Linux-4.90-64.tgz"
  if [ ! -f uEye-Linux-4.90-64.tgz ]; then
    echo "Downloading ${DOWNLOAD_FILE}"
    wget "$BASE_URL/$DOWNLOAD_URL/$DOWNLOAD_FILE" -O $DOWNLOAD_FILE
  fi

  # Expand SDK
  if [ ! -f ueyesdk-setup-4.90-usb-amd64.gz.run ]; then
    echo "Expanding ${DOWNLOAD_FILE}"
    tar -xvf $DOWNLOAD_FILE
  fi

  # Uninstall SDK
  # /usr/local/share/ueye/bin/ueyed_install-usb uninstall
  # /usr/local/share/ueye/bin/ueyed_install-eth uninstall

  # Install SDK
  yes "\n" | ./ueyesdk-setup-4.90-usb-amd64.gz.run --nox11
  yes "\n" | ./ueyesdk-setup-4.90-eth-amd64.gz.run --nox11

  # Start uEye daemon
  /etc/init.d/ueyeusbdrc start
  echo "Finished installing uEye USB SDK"

else
  DOWNLOAD_PATH=/mnt/sdcard

  # Go to output directory
  cd $DOWNLOAD_PATH
  mkdir -p ueye_sdk && cd ueye_sdk

  # Download SDK
  DOWNLOAD_URL="download-ueye-emb-hardfloat.html?file=tl_files/downloads/LinuxEmbedded"
  DOWNLOAD_FILE="uEyeSDK-4.90.00-ARM_LINUX_IDS_AARCH64_GNU.tgz"
  if [ ! -f uEyeSDK-4.90.00-ARM_LINUX_IDS_AARCH64_GNU.tgz ]; then
    echo "Downloading ${DOWNLOAD_FILE}"
    wget "$BASE_URL/$DOWNLOAD_URL/$DOWNLOAD_FILE" -O $DOWNLOAD_FILE
  fi

  # Expand SDK
  if [ ! -f /usr/local/share/ueye/bin/ueyesdk-setup.sh ]; then
    echo "Expanding ${DOWNLOAD_FILE}"
    sudo tar -xvf $DOWNLOAD_FILE -C /
  fi

  # Run install script
  /usr/local/share/ueye/bin/ueyesdk-setup.sh

  # Start uEye daemon
  /etc/init.d/ueyeusbdrc start
  echo "Finished installing uEye USB SDK"
fi
