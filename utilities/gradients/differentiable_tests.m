%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2020 by California Institute of Technology.  ALL RIGHTS RESERVED. %
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

% Sanity checks on the differentials
clear all; close all; clc;

%% differentiable_angle_between_vectors
[a,da_dv1, da_dv2] = differentiable_angle_between_vectors();

for t=1:100
    v1 = rand(3,1);
    v2 = rand(3,1);

    angle = a(v1,v2);
    assert(abs(angle- acos(v1'*v2/norm(v1)/norm(v2)))<10*eps, "Angle computed incorrectly")

    da1_analytical = da_dv1(v1,v2);
    da2_analytical = da_dv2(v1,v2);
    for i=1:3
        pert = zeros(3,1);
        pert(i) = 1e-6;
        da1_numerical = (a(v1+pert,v2)-a(v1-pert,v2))/(2*norm(pert));
        assert(abs((da1_analytical(i)-da1_numerical)/norm(da1_analytical(i)))<0.0001, ...
            "ERROR: angle numerical derivative wrt v1 is off by more than 0.01pc")
        da2_numerical = (a(v1,v2+pert)-a(v1,v2-pert))/(2*norm(pert));
        assert(abs((da2_analytical(i)-da2_numerical)/norm(da2_analytical(i)))<0.0001, ...
            "ERROR: angle numerical derivative wrt v2 is off by more than 0.01pc")
    end
end
%% differentiable_norm_of_difference

[normdiff,dnormdiff_dv1, dnormdiff_dv2] = differentiable_norm_of_difference();

for t=1:100
    v1 = rand(3,1);
    v2 = rand(3,1);

    assert(normdiff(v1,v2) == norm(v1-v2), "ERROR: norm of difference is off")

    analytical_dnorm_v1 = dnormdiff_dv1(v1,v2);
    analytical_dnorm_v2 = dnormdiff_dv2(v1,v2);

    for i=1:3
        pert=zeros(3,1);
        pert(i) = 1e-6;
        num_norm_v1 = (normdiff(v1+pert, v2)-normdiff(v1-pert, v2))/(2*norm(pert));
        assert(abs((analytical_dnorm_v1(i)-num_norm_v1)/analytical_dnorm_v1(i))<0.0001, ...
            "ERROR: norm numerical derivative wrt v1 is off by more than 0.01pc")
        num_norm_v2 = (normdiff(v1, v2+pert)-normdiff(v1, v2-pert))/(2*norm(pert));
        assert(abs((analytical_dnorm_v2(i)-num_norm_v2)/analytical_dnorm_v2(i))<0.0001, ...
            "ERROR: norm numerical derivative wrt v2 is off by more than 0.01pc")
    end
end

%% Differentiable_logistic_function

edge = 1.;
width = 1;

[ssf, dssf] = differentiable_logistic_function(edge, width);


x = linspace(-4,6);
figure()
title("Derivative of logistic function")
hold all
plot(x,ssf(x),x,dssf(x))


for t=1:100
    pos = edge+4*width*(2*rand()-1);
    pert = 1e-5;
    numerical_derivative = (ssf(pos+pert)-ssf(pos-pert))/(2*norm(pert));
    analytical_derivative = dssf(pos);
    assert(abs((numerical_derivative-analytical_derivative)/analytical_derivative) < 0.0001,...
        "ERROR: differentiable logistic derivative is off by more than 0.01pc");
    plot(pos, ssf(pos), 'o')
    xx = linspace(pos-1, pos+1);
    plot(xx, analytical_derivative.*(xx-pos)+ssf(pos), 'b');
    plot(xx, numerical_derivative.*(xx-pos)+ssf(pos), 'r')
    
end
legend("Logistic", "derivative", "sample points", "analytical", "numerical", "location", "best")
%% Differentiable_window

lower_edge = -5;

upper_edge = 5;

width = .5;

[wf,dwf] = differentiable_window(lower_edge,upper_edge, width);

x = linspace(lower_edge-4*width,upper_edge+4*width);
figure()
title("Derivative of differentiable window")
hold all;
plot(x,wf(x),x,dwf(x))

for t=1:100
    pos = lower_edge + rand*(upper_edge-lower_edge+4*width) - 2*width;
    pert = 1e-5*width;
    numerical_derivative = (wf(pos+pert)-wf(pos-pert))/(2*norm(pert));
    analytical_derivative = dwf(pos); 
    assert(abs((numerical_derivative-analytical_derivative)/analytical_derivative) < 0.001,...
        "ERROR: differentiable logistic derivative is off by more than 0.1pc");
    plot(pos, wf(pos), 'o')
    xx = linspace(pos-.5, pos+.5);
    plot(xx, analytical_derivative.*(xx-pos)+wf(pos), 'b');
    plot(xx, numerical_derivative.*(xx-pos)+wf(pos), 'r')

end

legend("Window", "derivative", "sample points", "analytical", "numerical", "location", "best")

%% Differentiable window of norm difference
lower_edge = -5;

upper_edge = 5;

width = .5;

[wnd,dwnd_dv1, dwnd_dv2] = differentiable_window_of_norm_difference(lower_edge,upper_edge, width);

[wf,dwf] = differentiable_window(lower_edge,upper_edge, width);

for t=1:100
    v1 = rand(3,1);
    v2 = rand(3,1);
    
    assert(abs(wnd(v1,v2)- wf(norm(v1-v2)))<10*eps, "Window-of-norm-diff computed incorrectly")

    dwnd1_analytical = dwnd_dv1(v1,v2);
    dwnd2_analytical = dwnd_dv2(v1,v2);
    for i=1:3
        pert = zeros(3,1);
        pert(i) = 1e-6;
        dwnd1_numerical = (wnd(v1+pert,v2)-wnd(v1-pert,v2))/(2*norm(pert));
        assert(abs((dwnd1_analytical(i)-dwnd1_numerical)/norm(dwnd1_analytical(i)))<0.001, ...
            "ERROR: window-of-norm-diff numerical derivative wrt v1 is off by more than 0.1pc")
        dwnd2_numerical = (wnd(v1,v2+pert)-wnd(v1,v2-pert))/(2*norm(pert));
        assert(abs((dwnd2_analytical(i)-dwnd2_numerical)/norm(dwnd2_analytical(i)))<0.001, ...
            "ERROR: window-of-norm-diff numerical derivative wrt v2 is off by more than 0.1pc")
    end
end

%% Differentiable window of angle

lower_edge = -5;

upper_edge = 5;

width = .5;

[wa,dwa_dv1, dwa_dv2] = differentiable_window_of_angle(lower_edge,upper_edge, width);

[wf,dwf] = differentiable_window(lower_edge,upper_edge, width);

for t=1:100
    v1 = rand(3,1);
    v2 = rand(3,1);
    
    assert(abs(wa(v1,v2)- wf(acos(dot(v1/norm(v1),v2/norm(v2)))))<10*eps, ...
        "Window-of-angle computed incorrectly")

    dwa1_analytical = dwa_dv1(v1,v2);
    dwa2_analytical = dwa_dv2(v1,v2);
    for i=1:3
        pert = zeros(3,1);
        pert(i) = 1e-6;
        dwa1_numerical = (wa(v1+pert,v2)-wa(v1-pert,v2))/(2*norm(pert));
        dwa1_analytical(i);
        assert(abs((dwa1_analytical(i)-dwa1_numerical)/norm(dwa1_analytical(i)))<0.001, ...
            "ERROR: window-of-angle numerical derivative wrt v1 is off by more than 0.1pc")
        dwa2_numerical = (wa(v1,v2+pert)-wa(v1,v2-pert))/(2*norm(pert));
        assert(abs((dwa2_analytical(i)-dwa2_numerical)/norm(dwa2_analytical(i)))<0.001, ...
            "ERROR: window-of-angle numerical derivative wrt v2 is off by more than 0.1pc")
    end
end


%% fast_differentiable_angle_between_vectors

for t=1:100
    v1 = rand(3,1);
    v2 = rand(3,1);

    [angle,da_dv1, da_dv2] = fast_differentiable_angle_between_vectors(v1, v2);
    
    assert(abs(angle- acos(v1'*v2/norm(v1)/norm(v2)))<100*eps, "Angle computed incorrectly")

    da1_analytical = da_dv1;
    da2_analytical = da_dv2;
    for i=1:3
        pert = zeros(3,1);
        pert(i) = 1e-6;
        da1_numerical = (fast_differentiable_angle_between_vectors(v1+pert,v2)-fast_differentiable_angle_between_vectors(v1-pert,v2))/(2*norm(pert));
        assert(abs((da1_analytical(i)-da1_numerical)/norm(da1_analytical(i)))<0.0001, ...
            "ERROR: angle numerical derivative wrt v1 is off by more than 0.01pc")
        da2_numerical = (a(v1,v2+pert)-a(v1,v2-pert))/(2*norm(pert));
        assert(abs((da2_analytical(i)-da2_numerical)/norm(da2_analytical(i)))<0.0001, ...
            "ERROR: angle numerical derivative wrt v2 is off by more than 0.01pc")
    end
end
%% fast_differentiable_norm_of_difference


for t=1:100
    v1 = rand(3,1);
    v2 = rand(3,1);
    
    [normdiff,dnormdiff_dv1, dnormdiff_dv2] = fast_differentiable_norm_of_difference(v1, v2);
    assert(abs(normdiff - norm(v1-v2))<100*eps, "ERROR: norm of difference is off")

    analytical_dnorm_v1 = dnormdiff_dv1;
    analytical_dnorm_v2 = dnormdiff_dv2;

    for i=1:3
        pert=zeros(3,1);
        pert(i) = 1e-6;
        num_norm_v1 = (fast_differentiable_norm_of_difference(v1+pert, v2)-fast_differentiable_norm_of_difference(v1-pert, v2))/(2*norm(pert));
        assert(abs((analytical_dnorm_v1(i)-num_norm_v1)/analytical_dnorm_v1(i))<0.0001, ...
            "ERROR: norm numerical derivative wrt v1 is off by more than 0.01pc")
        num_norm_v2 = (fast_differentiable_norm_of_difference(v1, v2+pert)-fast_differentiable_norm_of_difference(v1, v2-pert))/(2*norm(pert));
        assert(abs((analytical_dnorm_v2(i)-num_norm_v2)/analytical_dnorm_v2(i))<0.0001, ...
            "ERROR: norm numerical derivative wrt v2 is off by more than 0.01pc")
    end
end

%% fast_Differentiable_logistic_function

edge = 1.;
width = 1;


x = linspace(-4,6);
figure()
title("Derivative of logistic function")
hold all
ssf = zeros(size(x));
dssf = ssf;
for i=1:length(x)
    [ssf(i), dssf(i)] = fast_differentiable_logistic_function(x(i), edge, width);
end
plot(x,ssf,x,dssf)


for t=1:100
    pos = edge+4*width*(2*rand()-1);
    pert = 1e-5;
    numerical_derivative = (fast_differentiable_logistic_function(pos+pert, edge, width)-fast_differentiable_logistic_function(pos-pert, edge, width))/(2*norm(pert));
    [ssfpos, analytical_derivative] = fast_differentiable_logistic_function(pos, edge, width);
    assert(abs((numerical_derivative-analytical_derivative)/analytical_derivative) < 0.0001,...
        "ERROR: differentiable logistic derivative is off by more than 0.01pc");
    plot(pos, ssfpos, 'o')
    xx = linspace(pos-1, pos+1);
    plot(xx, analytical_derivative.*(xx-pos)+ssfpos, 'b');
    plot(xx, numerical_derivative.*(xx-pos)+ssfpos, 'r')
    
end
legend("Logistic", "derivative", "sample points", "analytical", "numerical", "location", "best")
%% fast_Differentiable_window

lower_edge = -5;

upper_edge = 5;

width = .5;

x = linspace(lower_edge-4*width,upper_edge+4*width);
wf = zeros(size(x));
dwf = zeros(size(x));
for i=1:length(x)
   [wf(i), dwf(i)] = fast_differentiable_window(x(i), lower_edge,upper_edge, width); 
end

figure()
title("Derivative of differentiable window")
hold all;
plot(x,wf,x,dwf)

for t=1:100
    pos = lower_edge + rand*(upper_edge-lower_edge+4*width) - 2*width;
    pert = 1e-5*width;
    numerical_derivative = (fast_differentiable_window(pos+pert, lower_edge,upper_edge, width)-fast_differentiable_window(pos-pert, lower_edge,upper_edge, width))/(2*norm(pert));
    [wfpos, analytical_derivative] = fast_differentiable_window(pos, lower_edge,upper_edge, width); 
    assert(abs((numerical_derivative-analytical_derivative)/analytical_derivative) < 0.001,...
        "ERROR: differentiable logistic derivative is off by more than 0.1pc");
    plot(pos, wfpos, 'o')
    xx = linspace(pos-.5, pos+.5);
    plot(xx, analytical_derivative.*(xx-pos)+wfpos, 'b');
    plot(xx, numerical_derivative.*(xx-pos)+wfpos, 'r')

end

legend("Window", "derivative", "sample points", "analytical", "numerical", "location", "best")

%% fast_Differentiable window of norm difference
lower_edge = -5;

upper_edge = 5;

width = .5;

for t=1:100
    v1 = rand(3,1);
    v2 = rand(3,1);
    
    [wnd,dwnd_dv1, dwnd_dv2] = fast_differentiable_window_of_norm_difference(v1, v2, lower_edge,upper_edge, width);

    [wf,dwf] = fast_differentiable_window(norm(v1-v2), lower_edge,upper_edge, width);

    
    assert(abs(wnd- wf)<10*eps, "Window-of-norm-diff computed incorrectly")

    dwnd1_analytical = dwnd_dv1;
    dwnd2_analytical = dwnd_dv2;
    for i=1:3
        pert = zeros(3,1);
        pert(i) = 1e-6;
        dwnd1_numerical = (fast_differentiable_window_of_norm_difference(v1+pert,v2, lower_edge,upper_edge, width)-fast_differentiable_window_of_norm_difference(v1-pert,v2, lower_edge,upper_edge, width))/(2*norm(pert));
        assert(abs((dwnd1_analytical(i)-dwnd1_numerical)/norm(dwnd1_analytical(i)))<0.001, ...
            "ERROR: window-of-norm-diff numerical derivative wrt v1 is off by more than 0.1pc")
        dwnd2_numerical = (fast_differentiable_window_of_norm_difference(v1,v2+pert, lower_edge,upper_edge, width)-fast_differentiable_window_of_norm_difference(v1,v2-pert, lower_edge,upper_edge, width))/(2*norm(pert));
        assert(abs((dwnd2_analytical(i)-dwnd2_numerical)/norm(dwnd2_analytical(i)))<0.001, ...
            "ERROR: window-of-norm-diff numerical derivative wrt v2 is off by more than 0.1pc")
    end
end

%% Differentiable window of angle

lower_edge = -5;

upper_edge = 5;

width = .5;



for t=1:100
    v1 = rand(3,1);
    v2 = rand(3,1);
    
    [wa,dwa_dv1, dwa_dv2] = fast_differentiable_window_of_angle(v1, v2, lower_edge,upper_edge, width);

    [wf,dwf] = fast_differentiable_window(acos(dot(v1/norm(v1),v2/norm(v2))), lower_edge,upper_edge, width);
    
    assert(abs(wa- wf)<10*eps, "Window-of-angle computed incorrectly")

    dwa1_analytical = dwa_dv1;
    dwa2_analytical = dwa_dv2;
    for i=1:3
        pert = zeros(3,1);
        pert(i) = 1e-6;
        dwa1_numerical = (fast_differentiable_window_of_angle(v1+pert,v2, lower_edge,upper_edge, width)-fast_differentiable_window_of_angle(v1-pert,v2, lower_edge,upper_edge, width))/(2*norm(pert));
        dwa1_analytical(i);
        assert(abs((dwa1_analytical(i)-dwa1_numerical)/norm(dwa1_analytical(i)))<0.001, ...
            "ERROR: window-of-angle numerical derivative wrt v1 is off by more than 0.1pc")
        dwa2_numerical = (fast_differentiable_window_of_angle(v1,v2+pert, lower_edge,upper_edge, width)-fast_differentiable_window_of_angle(v1,v2-pert, lower_edge,upper_edge, width))/(2*norm(pert));
        assert(abs((dwa2_analytical(i)-dwa2_numerical)/norm(dwa2_analytical(i)))<0.001, ...
            "ERROR: window-of-angle numerical derivative wrt v2 is off by more than 0.1pc")
    end
end