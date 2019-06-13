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

function [R]=rotmat(angle,axis)

%Syntax: [R]=rotmat(angle,axis)
%Computes rotation matrix for a single rotation around one of the three axes x, y, z in a 3d space. In order to stack rotations, call the function several times. Remember, however, that after the first rotation you'll turn around the NEW axes, which will be different from the old ones.
%The axis parameter can be x (or i, or 1), y (or j, or 2), z (or k, or 3). Angle is given in radians.

if axis~='x' && axis~='y' && axis~='z'  && axis~='i'  && axis~='j' && axis~='k' &&axis~=1 && axis~=2 && axis~=3 || nargin<2
	disp('Invalid axis specified, returning identity matrix')
	R=eye(3);
	return
end

if axis=='x' || axis=='i' || axis==1
	R=[ 1 0 0; 0 cos(angle) sin(angle); 0 -sin(angle) cos(angle)];
elseif axis=='y' || axis=='j' || axis==2
	R=[cos(angle) 0 -sin(angle); 0 1 0; sin(angle) 0 cos(angle)];
elseif axis=='z' || axis=='k' || axis==3
	R=[cos(angle) sin(angle) 0; -sin(angle) cos(angle) 0; 0 0 1];
end
return;