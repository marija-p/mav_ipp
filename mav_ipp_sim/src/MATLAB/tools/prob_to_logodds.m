function [logodds] = prob_to_logodds(prob)
% Converts probability to log-odds

logodds = log(prob./(1-prob));

end