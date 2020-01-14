function [mu, sigma, sigma_points] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model.
% mu: state vector containing robot pose and poses of landmarks obeserved so far
% Current robot pose = mu(1:3)
% Note that the landmark poses in mu are stacked in the order by which they were observed
% sigma: the covariance matrix of the system.
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% For computing lambda.
global scale;

% Compute sigma points
sigma_points = compute_sigma_points(mu, sigma);
sigma_points(3, :) = normalize_angle(sigma_points(3, :));
% Dimensionality
n = length(mu);
% lambda
lambda = scale - n;

% TODO: Transform all sigma points according to the odometry command
% Remember to vectorize your operations and normalize angles
% Tip: the function normalize_angle also works on a vector (row) of angles
tran_points = motion_command(sigma_points, u);

% Computing the weights for recovering the mean
wm = [lambda/scale, repmat(1/(2*scale),1,2*n)];
wc = wm;

% TODO: recover mu.
% Be careful when computing the robot's orientation (sum up the sines and
% cosines and recover the 'average' angle via atan2)
mu = zeros(n, 1);
sigma = zeros(n);
x_bar = 0;
y_bar = 0;
for i = 1:(2*n + 1)
  sigma_points(1:3, i) = motion_command(sigma_points(1:3, i), u);
  mu = mu + wm(i) * sigma_points(:, i);
  x_bar = x_bar + wm(i) * cos(sigma_points(3, i));
  y_bar = y_bar + wm(i) * sin(sigma_points(3, i));
endfor

mu(3) = normalize_angle(atan2(y_bar, x_bar));
% TODO: Recover sigma. Again, normalize the angular difference
for i = 1:2*n + 1;
    diff = sigma_points(:, i) -  mu;
    diff(3) = normalize_angle(diff(3));
    sigma = sigma + diff * wm(i) * diff';
endfor

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Add motion noise to sigma
sigma = sigma + R;

end

% Copied from Sheet 1 program
% By Gunasekaran
function [x] = motion_command(x, u)
  % Updates the robot pose according to the motion model
  % x: 3x1 vector representing the robot pose [x; y; theta]
  % u: struct containing odometry reading (r1, t, r2).
  % Use u.r1, u.t, and u.r2 to access the rotation and translation values
  %TODO: update x according to the motion represented by u
  x(1) = x(1) + (u.t * cos(x(3) + u.r1));
  x(2) = x(2) + (u.t * sin(x(3) + u.r1));
  x(3) = x(3) + u.r1 + u.r2;
  %TODO: remember to normalize theta by calling normalize_angle for x(3)
  x(3) = normalize_angle(x(3));
end