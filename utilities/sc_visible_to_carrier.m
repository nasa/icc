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

function this_visible = sc_visible_to_carrier(this_SC_pos, carrier_SC_pos, Radius)

% http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html

x0 = [0 0 0];

x1 = this_SC_pos;

x2 = carrier_SC_pos;

t = -dot(x1-x0,x2-x0)/(norm(x2-x1))^2;

xp = x1 + (x2-x1)*t;

if (t>=0) && (t<=1)
    
    if norm(xp-x0) > Radius
        
        this_visible = 1;
        
    else
        
        this_visible = 0;
        
    end
    
else
    
    if (norm(x1-x0) > Radius) &&  (norm(x2-x0) > Radius) 
        
        this_visible = 1;
        
    else
        
        this_visible = 0;
        
    end
    
end
