%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%            Helper function to create rotation matrices.                 %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2019 by California Institute of Technology.  ALL RIGHTS RESERVED. %
% United  States  Government  sponsorship  acknowledged.   Any commercial use %
% must   be  negotiated  with  the  Office  of  Technology  Transfer  at  the %
% California Institute of Technology.                                         %
%                                                                             %
% This software may be subject to  U.S. export control laws  and regulations. %
% By accepting this document,  the user agrees to comply  with all applicable %
% U.S. export laws and regulations.  User  has the responsibility  to  obtain %
% export  licenses,  or  other  export  authority  as may be required  before %
% exporting  such  information  to  foreign  countries or providing access to %
% foreign persons.                                                            %
%                                                                             %
% This  software  is a copy  and  may not be current.  The latest  version is %
% maintained by and may be obtained from the Mobility  and  Robotics  Sytstem %
% Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches %
% are welcome and should be sent to the software's maintainer.                %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [R]=rotmat_axis_angle(axis_vector,angle_rad)

%Syntax: [R]=rotmat_axis_angle(axis_vector,angle_rad)
%Computes rotation matrix for a single rotation of angle_rad around axis_vector in a 3d space.
% See Taylor, Camillo J.; Kriegman, David J. (1994).
% "Minimization on the Lie Group SO(3) and Related Manifolds" (PDF).
% Technical Report No. 9405. Yale University.
% https://www.cis.upenn.edu/~cjtaylor/PUBLICATIONS/pdfs/TaylorTR94b.pdf
% Via https://en.wikipedia.org/w/index.php?title=Rotation_matrix&section=11#Rotation_matrix_from_axis_and_angle


if nargin<2
    error("You should specify both an axis vector (a 3d column vector) and an angle (in radians)")
end

assert(length(axis_vector)==3, "ERROR: axis_vector should be a vector of length 3");
assert(norm(axis_vector)>0, "ERROR: axis_vector has zero norm");

axis_vector = axis_vector/norm(axis_vector);

ux = axis_vector(1);
uy = axis_vector(2);
uz = axis_vector(3);

ct = cos(angle_rad);
st = sin(angle_rad);

R = [ct + ux^2*(1-ct), ux*uy*(1-ct) - uz*st, ux*uz*(1-ct)+uy*st; ...
    uy*ux*(1-ct) + uz*st, ct + uy^2*(1-ct), uy*uz*(1-ct) - ux*st; ...
    uz*ux*(1-ct) - uy*st, uz*uy*(1-ct) + ux*st, ct+uz^2*(1-ct)];

return;