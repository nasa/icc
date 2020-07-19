%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%  Tests for gradient utilities. Focus on ensuring syntax and dimensions  %
%  are correct. Functional tests are in gradient_test.m and               %
%  integrated_gradient_test.m                                             %
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

clear all; close all; clc;

%% Scalar tests

x1s=rand(1,3);
x2s=rand(1,3);

edge = 1.;
width = 1.;
lower_edge = -1;
upper_edge = 1;
lower_width = .5;
upper_width = .3;

s = rand();

[sf, dsf] = fast_differentiable_logistic_function(s, edge, width);

[wf,dwf] = fast_differentiable_window(s, lower_edge,upper_edge, lower_width, upper_width);

[normdiff,dnormdiff_dv1, dnormdiff_dv2] = fast_differentiable_norm_of_difference(x1s, x2s);

[wnd,dwnd_dv1, dwnd_dv2] = fast_differentiable_window_of_norm_difference(x1s, x2s, lower_edge, upper_edge, lower_width, upper_width);

[a,da_dv1, da_dv2] = fast_differentiable_angle_between_vectors(x1s, x2s);

[wa,dwa_dv1, dwa_dv2] = fast_differentiable_window_of_angle(x1s, x2s, lower_edge, upper_edge, lower_width, upper_width);

%% Vector tests

x1v = rand(5,3);
x2v = rand(5,3);

v = rand(5,1);

[sfv, dsfv] = fast_differentiable_logistic_function(v, edge, width);
for i=1:length(sfv)
    [sfvi, dsfvi] = fast_differentiable_logistic_function(v(i), edge, width);

    assert(sfvi == sfv(i), "ERROR: logistic function, arg mismatch at %d", i);
    assert(dsfvi == dsfv(i), "ERROR: logistic function, derivative mismatch at %d", i);
end

[wfv,dwfv] = fast_differentiable_window(v, lower_edge,upper_edge, lower_width, upper_width);

for i=1:length(wfv)
    [wfvi, dwfvi] = fast_differentiable_window(v(i), lower_edge,upper_edge, lower_width, upper_width);

    assert(wfvi == wfv(i), "ERROR: window function, arg mismatch at %d", i);
    assert(dwfvi == dwfv(i), "ERROR: window function, derivative mismatch at %d", i);
end

[normdiffv,dnormdiff_dv1v, dnormdiff_dv2v] = fast_differentiable_norm_of_difference(x1v, x2v);

for i=1:size(x2v,1)
    [normdiffvi,dnormdiff_dv1vi, dnormdiff_dv2vi] = fast_differentiable_norm_of_difference(x1v(i,:), x2v(i,:));
    
    assert(abs(normdiffv(i) - norm(x1v(i,:)-x2v(i,:),2))<50*eps, "ERROR: norm is wrong at %d", i);
    assert(abs(normdiffv(i) -normdiffvi) < 50*eps, "ERROR: norm of difference function, arg mismatch at %d", i);
    assert(all((dnormdiff_dv1v(i,:) -dnormdiff_dv1vi)<50*eps), "ERROR: norm of difference function, d1 mismatch at %d", i);
    assert(all((dnormdiff_dv2v(i,:) -dnormdiff_dv2vi)<50*eps), "ERROR: norm of difference function, d2 mismatch at %d", i);
end

[wndv,dwnd_dv1v, dwnd_dv2v] = fast_differentiable_window_of_norm_difference(x1v, x2v, lower_edge, upper_edge, lower_width, upper_width);
for i=1:size(x2v,1)
    [wndvi,dwnd_dv1vi, dwnd_dv2vi] = fast_differentiable_window_of_norm_difference(x1v(i,:), x2v(i,:), lower_edge, upper_edge, lower_width, upper_width);
    
    assert(abs(wndv(i) -wndvi) < 10*eps, "ERROR: windowed norm of difference function, arg mismatch at %d", i);
    assert(all((dwnd_dv1v(i,:) -dwnd_dv1vi)<50*eps), "ERROR: windowed norm of difference function, d1 mismatch at %d", i);
    assert(all((dwnd_dv2v(i,:) -dwnd_dv2vi)<50*eps), "ERROR: windowed norm of difference function, d2 mismatch at %d", i);
end

[av,da_dv1v, da_dv2v] = fast_differentiable_angle_between_vectors(x1v, x2v);

for i=1:size(x2v,1)
    [avi,da_dv1vi, da_dv2vi] = fast_differentiable_angle_between_vectors(x1v(i,:), x2v(i,:));

    assert(abs(av(i) -avi) < 50*eps, "ERROR: angle between vectors function, arg mismatch at %d", i);
    assert(all((da_dv1v(i,:) -da_dv1vi)<50*eps), "ERROR: angle between vectors function, d1 mismatch at %d", i);
    assert(all((da_dv2v(i,:) -da_dv2vi)<50*eps), "ERROR: angle between vectors function, d2 mismatch at %d", i);
end

[wav,dwa_dv1v, dwa_dv2v] = fast_differentiable_window_of_angle(x1v, x2v, lower_edge, upper_edge, lower_width, upper_width);

for i=1:size(x2v,1)
    [wavi,dwa_dv1vi, dwa_dv2vi] = fast_differentiable_window_of_angle(x1v(i,:), x2v(i,:), lower_edge, upper_edge, lower_width, upper_width);

    assert(abs(wav(i) -wavi) < 50*eps, "ERROR: windowed angle between vectors function, arg mismatch at %d", i);
    assert(all((dwa_dv1v(i,:) -dwa_dv1vi)<50*eps), "ERROR: windowed angle between vectors function, d1 mismatch at %d", i);
    assert(all((dwa_dv2v(i,:) -dwa_dv2vi)<50*eps), "ERROR: windowed angle between vectors function, d2 mismatch at %d", i);
end