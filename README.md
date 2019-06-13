# icc-sim-matlab
Contains MATLAB simulation and optimization code for spacecraft orbiting Small Solar System Bodies.

## small_body_dynamics
Integrators to simulate the orbital dynamics of a spacecraft in orbit around a small body.

## network_flow_communication_optimizer
Optimize the communication flows between spacecraft, given a set of orbits, data production, and a communication model.

## relay_orbit_optimizer
Optimize the orbit of relay spacecraft, given the orbits of science spacecraft, their data production, and the orbit of the carrier.

## utilities
Common utilities for orbital dynamics.

## media
Various media.

## Dependencies
- [CVX](http://cvxr.com/cvx/). The academic version with the Mosek solver is highly recommended.
- JPL's [Small Body Dynamics Toolkit (SBDT)](https://engineering.purdue.edu/people/kathleen.howell.1/Publications/Conferences/2015_AAS_SBDT.pdf). SBDT is not open-source. A license can be requested from [download.jpl.nasa.gov](download.jpl.nasa.gov) mentioning NTR-49005. 

### Troubleshooting SBDT
SBDT is typically distributed as a `.zip` file. The archive contains tests in the `Tests` folder and examples in the `Demos` folder.

Core SBDT functions are implemented in C and must be compiled to MATLAB _mex_ files to enable MATLAB to call them. The SBDT distribution contains pre-compiled MEX files for Windows, MacOS, and Linux inside the folder `CompiledMEX`.
If you encounter issues trying to run SBDT, it may be advisable to recompile the MEX files from scratch. To do this,
- Navigate to the SBDT root folder.
- In MATLAB, run `compileMexSBDT()`. A number of files will be generated in the folder. The file extension is `mexw64` on Windows, `mexa64` on Linux, and `mexmaci64` on MacOS. 
- Copy the newly generated mex files to the CompiledMEX folder, overwriting the previous ones.
