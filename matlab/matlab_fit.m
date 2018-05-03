clear;
close all;

filename = 'normal_run_timed1';

load(strcat(filename, '.mat'));

angle_diff = zeros(size(angles));
angle_ddiff = zeros(size(velocities));

for i = 1:5

    figure(i);
    x = times;
    y = angles(:, i);


    g = fittype('L / (1 + exp(-k * (x - r))) + b', 'independent', {'x'}, 'dependent', {'y'}, 'coefficients', {'L', 'k', 'r', 'b'});

    start_r = mean(x);
    start_L = y(end);
    start_b = y(1);
    start_k = 1;

    f = fit(x, y, g, 'StartPoint', [start_L, start_k, start_r, start_b]);
    [fx, fxx] = differentiate(f, x);

    subplot(3, 1, 1);
    plot(f, x, y);

    subplot(3, 1, 2);
    hold on;
    plot(x, fx);
    plot(x, velocities(:, i));

    subplot(3, 1, 3);
    plot(x, fxx);

    angle_diff(:, i) = fx;
    angle_ddiff(:, i) = fxx;
end

save(strcat(filename, '_diffs.mat'), 'angle_diff', 'angle_ddiff');