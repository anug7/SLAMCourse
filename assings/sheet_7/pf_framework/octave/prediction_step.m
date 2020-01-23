function particles = prediction_step(particles, u, noise)
% Updates the particles by drawing from the motion model
% Use u.r1, u.t, and u.r2 to access the rotation and translation values
% which have to be pertubated with Gaussian noise.
% The position of the i-th particle is given by the 3D vector
% particles(i).pose which represents (x, y, theta).

% noise parameters
% Assume Gaussian noise in each of the three parameters of the motion model.
% These three parameters may be used as standard deviations for sampling.
r1Noise = noise(1);
transNoise = noise(2);
r2Noise = noise(3);

numParticles = length(particles);
for i = 1:numParticles

  % append the old position to the history of the particle
  particles(i).history{end+1} = particles(i).pose;

  % TODO: sample a new pose for the particle
  particles(i).pose = motion_command(particles(i).pose, u, noise);

end

end

function [x] = motion_command(x, u, noise)
  % Updates the robot pose according to the motion model
  % x: 3x1 vector representing the robot pose [x; y; theta]
  % u: struct containing odometry reading (r1, t, r2).
  % Use u.r1, u.t, and u.r2 to access the rotation and translation values
  %TODO: update x according to the motion represented by u
  x(1) = x(1) + (u.t * cos(x(3) + u.r1)) + normrnd(0, noise(1) + noise(2) + noise(3));
  x(2) = x(2) + (u.t * sin(x(3) + u.r1)) + normrnd(0, noise(1) + noise(2) + noise(3));
  x(3) = x(3) + u.r1 + u.r2 + normrnd(0, noise(1) + noise(3));
  %TODO: remember to normalize theta by calling normalize_angle for x(3)
  x(3) = normalize_angle(x(3));
end

