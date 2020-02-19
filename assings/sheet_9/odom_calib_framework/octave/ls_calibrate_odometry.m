% this function solves the odometry calibration problem
% given a measurement matrix Z.
% We assume that the information matrix is the identity
% for each of the measurements
% Every row of the matrix contains
% z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
% Z:	The measurement matrix
% X:	the calibration matrix
% returns the correction matrix X
function X = ls_calibrate_odometry(Z)
  % initial solution (the identity transformation)
  X = ones(3); 
  omega = eye(3);
  keyboard();

  % TODO: initialize H and b of the linear system
  H = zeros(9, 9);
  b = zeros(9, 1);
  % TODO: loop through the measurements and update H and b
  % You may call the functions error_function and jacobian, see below
  % We assume that the information matrix is the identity.
  for i = 1: size(Z, 1)
    e = error_function(i, X, Z);
    ji = jacobian(i, Z);
    H = H + (ji' * ji);
    b = b + (ji' * e);
  endfor
  % TODO: solve and update the solution
  delX = -H \ b;
  X = X + [delX(1:3)'; delX(4:6)'; delX(7:9)'];
end

% this function computes the error of the i^th measurement in Z
% given the calibration parameters
% i:	the number of the measurement
% X:	the actual calibration parameters
% Z:	the measurement matrix, each row contains first the scan-match result
%       and then the motion reported by odometry
% e:	the error of the ith measurement
function e = error_function(i, X, Z)
  % TODO compute the error of each measurement
  scan_op = Z(i, 1:3)';
  odom_op = Z(i, 4:6)';
  e = scan_op - (X * odom_op);
end

% derivative of the error function for the ith measurement in Z
% i:	the measurement number
% Z:	the measurement matrix
% J:	the jacobian of the ith measurement
function J = jacobian(i, Z)
  % TODO compute the Jacobian
  odom_op = Z(i, 4:6);
  J = zeros(3, 9);
  J(1, 1:3) = -odom_op;
  J(2, 4:6) = -odom_op;
  J(3, 7:9) = -odom_op;
end
