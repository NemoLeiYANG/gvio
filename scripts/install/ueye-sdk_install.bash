#!/usr/bin/bash
# set -e

BASE_URL="https://en.ids-imaging.com"
DOWNLOAD_URL="download-ueye-lin64.html?file=tl_files/downloads/uEye_SDK/driver"
DOWNLOAD_FILE="uEye-Linux-4.90-64.tgz"

# Go to output directory
cd /usr/local/src
mkdir -p ueye_sdk && cd ueye_sdk

# Download SDK
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
