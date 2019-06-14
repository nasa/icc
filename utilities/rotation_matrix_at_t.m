% Copyright 2019, by the California Institute of Technology. ALL RIGHTS
% RESERVED. United States Government sponsorship acknowledged. Any
% commercial use must be negotiated with the Office of Technology Transfer
% at the California Institute of Technology. This software may be subject
% to U.S. export control laws and regulations. By accepting this document,
% the user agrees to comply with all applicable U.S. export laws and
% regulations. User has the responsibility to obtain export licenses, or
% other export authority as may be required, before exporting such
% information to foreign countries or providing access to foreign persons.
% 
% Author: Saptarshi Bandyopadhyay (saptarshi.bandyopadhyay@jpl.nasa.gov)

function Rot_matrix_at_t = rotation_matrix_at_t(t,rotation_rate)

theta = mod(rotation_rate*t,2*pi);

Rot_matrix_at_t = [cos(theta) -sin(theta) 0;
    sin(theta)  cos(theta) 0;
    0           0 1];
end