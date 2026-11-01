#!/bin/sh
#
# Fetch the FacetimeHD firmware (5.60.0, from Apple's macOS 10.12.6 combo
# update) and the sensor calibration files, using the extraction script from
# https://github.com/patjak/facetimehd-firmware.  Only byte ranges of the
# update are downloaded (~25MB).  Run as a normal user; install with
#   sudo install -d /boot/firmware/facetimehd
#   sudo install -m 444 fw/*.bin fw/*.dat /boot/firmware/facetimehd/
set -eu

URL=https://updates.cdn-apple.com/2019/cert/041-90765-20191011-837e856d-b522-4865-b64c-641048ed77c4/macOSUpdCombo10.12.6.dmg

mkdir -p fw
cd fw

[ -d facetimehd-firmware ] ||
	git clone --depth 1 https://github.com/patjak/facetimehd-firmware.git

if [ ! -s AppleCameraInterface ]; then
echo "Downloading AppleCameraInterface..."
curl -s -L -r 699186570-703316225 "$URL" | xzcat -q > drv.chunk1
curl -s -L -r 703316242-707986973 "$URL" | xzcat -q > drv.chunk2
tail -c 249719 drv.chunk1 > AppleCameraInterface
head -c 507353 drv.chunk2 >> AppleCameraInterface
rm -f drv.chunk1 drv.chunk2
fi

if [ ! -s AppleCameraAssistant ]; then
echo "Downloading AppleCameraAssistant..."
curl -s -L -r 430282546-439105037 "$URL" | xzcat -q > asst.chunk
head -c $((7477241 + 515632)) asst.chunk | tail -c 515632 > AppleCameraAssistant
rm -f asst.chunk
fi

# The script checks SHA-256 hashes of everything it extracts.  It cd's to
# its own directory first, so run a copy that lives here.
cp facetimehd-firmware/extract-firmware.sh .
bash ./extract-firmware.sh \
	-x AppleCameraInterface -s AppleCameraAssistant

rm -f AppleCameraInterface AppleCameraAssistant
ls -l firmware.bin *.dat
