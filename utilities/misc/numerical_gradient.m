function [gradient] = numerical_gradient(fun_handle, linearization_point, increment)

% Computes the numerical gradient of function fun_handle around
% linearization_point. Increments the function by increment
% If increment is a scalar, it is applied to all entries. If it is a vector
% of the same length as linearization point, each entry of the increment is
% applied to the corresponding coordinate.

assert(length(increment)==1 || all(size(linearization_point) == size(increment)));

if length(increment)==1
    increment = increment*ones(size(linearization_point));
end

gradient = zeros(length(linearization_point),1);

parfor entry = 1:length(linearization_point)
    fprintf("%d/%d: ", entry, length(linearization_point));
    % We perturb along one direction only.
    perturbation = zeros(length(linearization_point),1);
    perturbation(entry) = increment(entry);
    try
       gradient(entry) = (fun_handle(linearization_point+perturbation) - fun_handle(linearization_point-perturbation))/(2*increment(entry));
    catch
       gradient(entry)= NaN;
       warning("No gradient could be computed at point "+ sprintf(" %d", linearization_point+perturbation))
    end
end