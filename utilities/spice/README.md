# SPICE utilities

- `download_SPICE_kernel.sh`: 
Downloads the SPICE kernels necessary for ICC in the folder `$NAIF_PATH`. To set up the download path, run `export NAIF_PATH=/path/to/folder/containing/naif`.

*The script requires the command `ftp` to be available. On MacOS, you may want to install ftp by typing `brew install inetutils`.*

- `load_spice_eros.m`: the function loads the SPICE kernels required by ICC. It takes as input the path to the NAIF folder.

- `Spice_Eros_Location.m`: a demo script that showcases the use of SPICE to compute the relative location of the Sun with respect to Eros.

## Requirements
Using SPICE requires the installation of the SPICE MATLAB toolkit (MICE), available at [this link](https://naif.jpl.nasa.gov/naif/toolkit_MATLAB.html).