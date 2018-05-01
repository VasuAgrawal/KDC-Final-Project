filename = 'normal_run_timed3';

load(strcat(filename, '.mat'));

% Differentiate things to get other things
d_angle = zeros(size(angles));
d_vel = zeros(size(velocities));

diff_angle = angles(3:end, :) - angles(1:end-2, :);
diff_vel = velocities(3:end, :) - velocities(1:end-2, :);
diff_times = times(3:end, :) - times(1:end-2, :);

d_angle(2:end-1, :) = diff_angle ./ diff_times;
d_vel(2:end-1, :) = diff_vel ./ diff_times;

t = times(2:end-1);

subplot(2, 2, 1);
plot(t, angles(2:end-1, :));
title(strcat('Sensor Angle Measurement: ', filename), 'Interpreter', 'none');

subplot(2, 2, 2);
plot(t, velocities(2:end-1, :));
title(strcat('Sensor Velocity Measurement: ', filename), 'Interpreter', 'none');

subplot(2, 2, 3);
plot(t, d_angle(2:end-1, :));
title(strcat('Numerical Angle Derivative: ', filename), 'Interpreter', 'none');

subplot(2, 2, 4);
plot(t, d_vel(2:end-1, :));
title(strcat('Numerical Velocity Derivative: ', filename), 'Interpreter', 'none');


% save('output.mat', 'd_angle', 'd_vel');