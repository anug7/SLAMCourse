function [mu, sigma, observedLandmarks] = correction_step(mu, sigma, z, observedLandmarks)
% Updates the belief, i. e., mu and sigma after observing landmarks, according to the sensor model
% The employed sensor model measures the range and bearing of a landmark
% mu: 2N+3 x 1 vector representing the state mean.
% The first 3 components of mu correspond to the current estimate of the robot pose [x; y; theta]
% The current pose estimate of the landmark with id = j is: [mu(2*j+2); mu(2*j+3)]
% sigma: 2N+3 x 2N+3 is the covariance matrix
% z: struct array containing the landmark observations.
% Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.
% observedLandmarks(j) is false if the landmark with id = j has never been observed before.

% Number of measurements in this time step
m = size(z, 2);

% Z: vectorized form of all measurements made in this time step: [range_1; bearing_1; range_2; bearing_2; ...; range_m; bearing_m]
% ExpectedZ: vectorized form of all expected measurements in the same form.
% They are initialized here and should be filled out in the for loop below
Z = zeros(m*2, 1);
expectedZ = zeros(m*2, 1);
N = size(observedLandmarks, 2);

% Iterate over the measurements and compute the H matrix
% (stacked Jacobian blocks of the measurement function)
% H will be 2m x 2N+3
H = [];
% keyboard ()

for i = 1:m	
	% Get the id of the landmark corresponding to the i-th observation
	landmarkId = z(i).id;
  Fxj = zeros(5, 3 + (2 * N));
  Fxj(1:3, 1:3) = eye(3);
  Fxj(4:5, (2 * landmarkId) + 2: (2 * landmarkId) + 3) = eye(2);
  
  pose = mu(1:3);
	% If the landmark is obeserved for the first time:
	if(observedLandmarks(landmarkId)==false)
		% TODO: Initialize its pose in mu based on the measurement and the current robot pose:
		temp = [pose(1) + (z(i).range * cos(z(i).bearing + pose(3)));
            pose(2) + (z(i).range * sin(z(i).bearing + pose(3)))];
		% Indicate in the observedLandmarks vector that this landmark has been observed
		observedLandmarks(landmarkId) = true;
    % 3 for pose of robot. 
    mu((landmarkId * 2) + 2: (landmarkId * 2) + 3) = temp;
	endif

  mui = mu((landmarkId * 2) + 2: (landmarkId * 2) + 3);
	% TODO: Add the landmark measurement to the Z vector
	Z(i * 2 - 1) = z(i).range;
  Z(i * 2) = z(i).bearing;
	% TODO: Use the current estimate of the landmark pose
	% to compute the corresponding expected measurement in expectedZ:
  delta = [mui(1) - pose(1); mui(2) - pose(2)];
  q = transpose(delta) * delta;

  sqrt_q = sqrt(q);  
  expectedZ(i * 2 - 1) = sqrt_q;
  expectedZ(i * 2) = normalize_angle(atan2(delta(2), delta(1)) - pose(3));
	% TODO: Compute the Jacobian Hi of the measurement function h for this observation

	Hi = (1 / q) * [-1 * sqrt_q * delta(1), -1 * sqrt_q * delta(2),     0,     sqrt_q * delta(1), sqrt_q * delta(2);
                                delta(2), -1 * delta(1),         -1 * q,         -1 * delta(2),          delta(1)] * Fxj;
	% Augment H with the new Hi
	H = [H;Hi];
endfor

% TODO: Construct the sensor noise matrix Q
  Q = 0.01 * eye(2 * m);
  
% TODO: Compute the Kalman gain
  Kt = sigma * transpose(H) * inverse((H * sigma * transpose(H) + Q));
% TODO: Compute the difference between the expected and recorded measurements.
% Remember to normalize the bearings after subtracting!
% (hint: use the normalize_all_bearings function available in tools)
  norm_z = normalize_all_bearings((Z - expectedZ));
% TODO: Finish the correction step by computing the new mu and sigma.
  mu = mu + Kt * norm_z;
  eyes = eye(size(sigma, 1));
  sigma = (eyes - Kt * H) * sigma;
% Normalize theta in the robot pose.
  mu(3) = normalize_angle(mu(3));
end
