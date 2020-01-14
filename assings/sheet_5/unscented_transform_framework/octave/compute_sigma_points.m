function [sigma_points, w_m, w_c] = compute_sigma_points(mu, sigma, lambda, alpha, beta)
% This function samples 2n+1 sigma points from the distribution given by mu and sigma
% according to the unscented transform, where n is the dimensionality of mu.
% Each column of sigma_points should represent one sigma point
% i.e. sigma_points has a dimensionality of nx2n+1.
% The corresponding weights w_m and w_c of the points are computed using lambda, alpha, and beta:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n] (i.e. each of size 1x2n+1)
% They are later used to recover the mean and covariance respectively.

n = length(mu);
sigma_points = zeros(n,2*n+1);
w_m = zeros(1, 2*n+1);
w_c = zeros(1, 2*n+1);

% TODO: compute all sigma points
cholesky_decop = chol((n + lambda) * sigma);

sigma_points(:, 1) = transpose(mu);
sigma_points(:, 2:2+n-1) = mu + cholesky_decop;
sigma_points(:, 2+n:end) = mu - cholesky_decop;
% TODO compute weight vectors w_m and w_c
w_m(1) = lambda / (n + lambda);
w_c(1) = w_m(1) + (1 - alpha * alpha + beta * beta);

w_others = ones(1, 2 * n) * 1 / (2 * (n + alpha));
w_m(2: end)  = w_others;
w_c(2: end)  = w_others;

end
