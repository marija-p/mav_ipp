function [prob] = logodds_to_prob(logodds)
% Converts log-odds to probability

prob = 1 - (1./(1+exp(logodds)));

end