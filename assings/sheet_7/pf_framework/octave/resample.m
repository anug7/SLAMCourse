% resample the set of particles.
% A particle has a probability proportional to its weight to get
% selected. A good option for such a resampling method is the so-called low
% variance sampling, Probabilistic Robotics pg. 109
function newParticles = resample(particles)

numParticles = length(particles);

w = [particles.weight];

% normalize the weight
w = w / sum(w);

% consider number of effective particles, to decide whether to resample or not
useNeff = false;
%useNeff = true;
if useNeff
  neff = 1. / sum(w.^2);
  neff
  if neff > 0.5*numParticles
    newParticles = particles;
    for i = 1:numParticles
      newParticles(i).weight = w(i);
    end
    return;
  end
end

newParticles = struct;

% TODO: implement the low variance re-sampling

% the cumulative sum

% initialize the step and the current position on the roulette wheel
% AI for robotics: https://classroom.udacity.com/courses/cs373/lessons/48704330/concepts/487480820923
wmax = 2 * max(w);
beta = 0;
% walk along the wheel to select the particles
for i = 1:numParticles
  beta = beta + unifrnd(0.0, wmax);
  index = max(1, round(unifrnd(1, numParticles)));
  while w(index) < beta
    beta = beta - w(index);
    index = index + 1
    index = max(1, mod (index, numParticles));
  endwhile
  newParticles(i) = particles(index);
  
end

end
