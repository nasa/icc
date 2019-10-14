#!/bin/bash
if test -z $NAIF_PATH
then
    echo "Setting NAIF path to" $PWD
    echo "To select a download location, export NAIF_PATH=/path/to/folder/containing/naif"
    export NAIF_PATH=$PWD
else
    echo "Saving data to NAIF path" $NAIF_PATH
fi

echo "Downloading 433 EROS kernel"
wget -q --show-progress ftp://ssd.jpl.nasa.gov/pub/ssd/SCRIPTS/smb_spk
chmod 755 smb_spk
./smb_spk -b "Eros;" 2000-Jan-1 2025-JAN-1 $USER@jpl.nasa.gov a433.bsp
mkdir -p "${NAIF_PATH}/naif/generic_kernels/spk/asteroids/"
mv a433.bsp "${NAIF_PATH}/naif/generic_kernels/spk/asteroids/"

echo "Downloading leap seconds kernel"
mkdir -p "${NAIF_PATH}/naif/generic_kernels/lsk/"
wget -q --show-progress ftp://naif.jpl.nasa.gov/pub/naif/generic_kernels/lsk/naif0012.tls -P "${NAIF_PATH}/naif/generic_kernels/lsk/"

echo "Downloading generic constants kernel"
mkdir -p "${NAIF_PATH}/naif/generic_kernels/pck/"
wget -q --show-progress ftp://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/pck00010.tpc -P "${NAIF_PATH}/naif/generic_kernels/pck/"
wget -q --show-progress ftp://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/gm_de431.tpc -P "${NAIF_PATH}/naif/generic_kernels/pck/"

echo "Downloading planets kernel"
mkdir -p "${NAIF_PATH}/naif/generic_kernels/spk/planets/"
wget -q --show-progress ftp://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de435.bsp -P "${NAIF_PATH}/naif/generic_kernels/spk/planets/"
