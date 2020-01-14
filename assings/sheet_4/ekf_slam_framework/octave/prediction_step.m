function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
mu_predicted = motion_command(mu(1:3), u);
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)

% TODO: Compute the 3x3 Jacobian Gx of the motion model
Gx = [1, 0, -1 * u.t * sin(mu(3) + u.r1);
      0, 1, u.t * cos(mu(3) + u.r1); 
      0, 0, 1];

mu(1:3) = mu_predicted;
% TODO: Construct the full Jacobian G
G = zeros(size(sigma));
In = size(mu)(1) - 3;
I = eye([In, In]);

G(1:3, 1:3) = Gx;
G(4:end, 4:end) = I;
% Motion noise
motionNoise = 0.1;
R3 = [motionNoise,           0, 0; 
                0, motionNoise, 0; 
                0,           0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Compute the predicted sigma after incorporating the motion
sigma_new = G * sigma * transpose(G) + R;

[sigma] = sigma_new;

end

% Copied from Sheet 1 program
% By Gunasekaran
function [x] = motion_command(x, u)
  % Updates the robot pose according to the motion model
  % x: 3x1 vector representing the robot pose [x; y; theta]
  % u: struct containing odometry reading (r1, t, r2).
  % Use u.r1, u.t, and u.r2 to access the rotation and translation values
  %TODO: update x according to the motion represented by u
  x_1 = x(1) + (u.t * cos(x(3) + u.r1));
  x_2 = x(2) + (u.t * sin(x(3) + u.r1));
  x_3 = x(3) + u.r1 + u.r2;
  %TODO: remember to normalize theta by calling normalize_angle for x(3)
  x_3 = normalize_angle(x_3);
  
  [x] = [x_1; x_2; x_3];
end
