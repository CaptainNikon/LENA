% Code by RM
clear all
clc

reference_temp = [6, 10, 15, 20, 25, 30, 35, 40, 45, 50];
measured_temp = [3, 11, 15.5, 21, 26, 31, 36, 41, 46, 50];

p = polyfit(measured_temp, reference_temp, 1);

a = p(1);b = p(2);

fitted = polyval(p, measured_temp);

% Adapted by JS
SS_res = sum((reference_temp - fitted).^2);
SS_tot = sum((reference_temp - mean(reference_temp)).^2);
R_squared = 1 - SS_res / SS_tot;

% Create dummies for plot
fit_eqn = sprintf('y = %.3f·x + %.3f', a, b);
r2_text = sprintf('R^2 = %.4f', R_squared);

%r = corr(reference_humidity(:), fitted(:));
%disp(['r = ', num2str(r)]);

plot(measured_temp, reference_temp, 'bo'); 
hold on;
plot(measured_temp, fitted, 'r-');
xlabel('Measured Temperature');
ylabel('Reference Temperature');

% Add dummy label to plot
x_range = max(measured_temp) - min(measured_temp);
y_range = max(reference_temp) - min(reference_temp);

xpos = min(measured_temp) + 0.05 * x_range;
ypos = max(reference_temp) - 0.1 * y_range;
grayColor = [.9 .9 .9];

text(xpos, ypos, {fit_eqn; r2_text}, 'FontSize', 10, 'BackgroundColor', grayColor);

legend('Calibration Data','Linear Fit', 'Location', 'best');
grid on;
hold off;