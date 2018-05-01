load("normal_run_timed1.mat");

close all;

times = cumsum(time_deltas);

% plot(times, angles(:, 1));
% plot(times, angles(:, 2));


% plot(times, angles);
% legend({"Joint 1"; "Joint 2"; "Joint 3"; "Joint 4"; "Joint 5"});

avg_time = mean(time_deltas(:));

% vel2 = (angles(2:end, :) - angles(1:end-1, :)) ./ time_deltas(1:end-1, :);
vel2 = (angles(2:end, :) - angles(1:end-1, :)) ./ avg_time;
plot(times(1:end-1), vel2);
legend({"Joint 1"; "Joint 2"; "Joint 3"; "Joint 4"; "Joint 5"});

figure();
plot(times, velocities);
legend({"Joint 1"; "Joint 2"; "Joint 3"; "Joint 4"; "Joint 5"});