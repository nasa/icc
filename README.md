# icc-dev
Contains MATLAB simulation and optimization code for spacecraft orbiting small Solar System Bodies.

___
## Quickstart
The examples folder contains example _main_ scripts, which demonstrate usage of the provided modules. The scripts should be executed from within this folder.

___
## Requirements

- [CVX](http://cvxr.com/cvx/). The academic version with the Mosek solver is highly recommended.
> **WARNING**: The communication optimization problem is known to falsely return infeasible with the SDPT3 solver. please use Mosek (if available) or SeDuMi.
- JPL's [Small Body Dynamics Toolkit (SBDT)](https://engineering.purdue.edu/people/kathleen.howell.1/Publications/Conferences/2015_AAS_SBDT.pdf). SBDT is not open-source. A license can be requested from [download.jpl.nasa.gov](download.jpl.nasa.gov) mentioning NTR-49005. 
- NASA's [SPICE](https://naif.jpl.nasa.gov/naif/aboutspice.html) MATLAB toolkit (MICE), available at [this link](https://naif.jpl.nasa.gov/naif/toolkit_MATLAB.html).
- In order to automatically download SPICE data, the [Expect/TCL](https://en.wikipedia.org/wiki/Expect) scripting language should be available. On Debian-based systems, you can `sudo apt-get install expect`. On MacOS, `brew install expect` with [Homebrew](https://brew.sh/). On Windows, either use the Windows Subsystem for Linux, or manually download SPICE data as discussed below..

### Setup
Set the following environment variables:

- `SBDT_PATH` should point to the folder where SBDT is installed.
- `NAIF_PATH` should point to the folder where NAIF data will be stored. This is typically the subfolder `utilities/spice` inside this repository.

Run the script `utilities/spice/download_SPICE_kernel.sh` to download required data from NASA's SPICE orbit database. If Expect/TCL is unavailable, manually download the orbit data for 433 EROS from [Horizons](https://ssd.jpl.nasa.gov/horizons.cgi) and save it as `${NAIF_PATH}/naif/generic_kernels/spk/asteroids/a433.bsp`.

To set environment variables:

- On Linux-based systems, add the line `export SBDT_PATH="path/to/your/copy/of/sbdt"` and `export NAIF_PATH="path/to/your/copy/of/naif"` to `~/.bashrc`. If you use a different shell (e.g. zsh), add the lines to the appropriate `.*rc` file.
- On MacOS, follow [these instructions](https://support.shotgunsoftware.com/hc/en-us/articles/219042108-Setting-global-environment-variables-on-OS-X). 
> **WARNING** On MacOS, setting environment variables in `~/.bashrc` or `~/.zshrc` will _not_ affect MATLAB unless MATLAB is launched from the terminal.
- On Windows, follow [these instructions](https://www.architectryan.com/2018/08/31/how-to-change-environment-variables-on-windows-10/). 

___
## Modules 
### small_body_dynamics
Integrators to simulate the orbital dynamics of a spacecraft in orbit around a small body.

### network_flow_communication_optimizer
Optimize the communication flows between spacecraft, given a set of orbits, data production, and a communication model.

### relay_orbit_optimizer
Optimize the orbit of relay spacecraft, given the orbits of science spacecraft, their data production, and the orbit of the carrier.

### observed_points_optimizer
Optimize the observation of spacecraft, given the orbits of science spacecraft.

### monte_carlo_coverage_optimizer
Runs a monte carlo to select spacecraft orbits. Calls the observed points optimizer to calculate the coverage reward of each trial orbit set. 

### utilities
Common utilities and supporting functions for icc simulation and analysis.

### media
Various media.

### Structure of Optimization Modules: 
The optimizer modules are composed of a _main_ function (is called outside the module) and several supporting functions which should only be called from within the module. Functions supporting multiple modules are placed in _utilities_.

___ 
## Interfaces
The system is defined through two special classes, which serve as the primary interfaces (inputs and outputs) between the modules. 
1. _SpacecraftSwarm_: Defines the swarm of spacecraft. 
2. _SphericalHarmonicsGravityIntegrator_SBDT_: Defines the small body that the spacecraft orbit around and integrates trajectories using JPL's Small Body Dynamics Integrator (SBDT). 

See the _examples_ directory for usage examples. 

___
### Troubleshooting SBDT
SBDT is typically distributed as a `.zip` file. The archive contains tests in the `Tests` folder and examples in the `Demos` folder.

Core SBDT functions are implemented in C and must be compiled to MATLAB _mex_ files to enable MATLAB to call them. The SBDT distribution contains pre-compiled MEX files for Windows, MacOS, and Linux inside the folder `CompiledMEX`.
If you encounter issues trying to run SBDT, it may be advisable to recompile the MEX files from scratch. To do this,
- Navigate to the SBDT root folder.
- In MATLAB, run `compileMexSBDT()`. A number of files will be generated in the folder. The file extension is `mexw64` on Windows, `mexa64` on Linux, and `mexmaci64` on MacOS. 
- Copy the newly generated mex files to the CompiledMEX folder, overwriting the previous ones.

### Detailed instructions on how to set environment variables

#### MacOS

- Create a file named `environment_variables.plist` in `~/Library/LaunchAgents/` (the file name is immaterial, but the extension should be `.plist`).

- Edit the file to contain the following text:
```xml
<?xml version="1.0" encoding="UTF-8"?>
 <!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
   <plist version="1.0">
   <dict>
   <key>Label</key>
   <string>Set environment variables that will be picked up by Spotlight</string>
   <key>ProgramArguments</key>
   <array>
     <string>/bin/launchctl</string>
     <string>setenv</string>
     <string>SBDT_PATH</string>
     <string>/Users/frossi/Documents/JPL/ICC/SBDT</string>
     <string>NAIF_PATH</string>
     <string>/Users/frossi/Documents/JPL/ICC/icc-dev/utilities/spice/</string>
   </array>
   <key>RunAtLoad</key>
   <true/>
 </dict>
 </plist>
```
Ensure that the SBDT and NAIF paths are set to the proper path on your machine.

- Run 
```
launchctl load ~/Library/LaunchAgents/environment_variables.plist
launchctl start ~/Library/LaunchAgents/environment_variables.plist
```