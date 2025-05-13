clc
clear all

correct_init_value_humidity = 0.543;
correct_init_value_temp = 21.5;

reference_humidity = [0.5540, 0.5740, 0.5940, 0.6140, 0.6340, 0.6540, 0.6740, 0.6940, 0.7140, 0.7340, 0.7540, 0.7740, 0.787];
measured_humidity = [0.55, 0.61, 0.62, 0.64, 0.66, 0.68, 0.7, 0.72, 0.74, 0.76, 0.78, 0.8, 0.81];

p = polyfit(measured_humidity, reference_humidity, 1);

a = p(1);
b = p(2);

fitted = polyval(p, measured_humidity);

SS_res = sum((reference_humidity - fitted).^2);
SS_tot = sum((reference_humidity - mean(reference_humidity)).^2);
R_squared = 1 - SS_res / SS_tot;

% Create dummies for plot
fit_eqn = sprintf('y = %.3f·x + %.3f', a, b);
r2_text = sprintf('R^2 = %.4f', R_squared);

%r = corr(reference_humidity(:), fitted(:));
%disp(['r = ', num2str(r)]);

plot(measured_humidity, reference_humidity, 'bo'); 
hold on;
plot(measured_humidity, fitted, 'r-');
xlabel('Measured Humidity');
ylabel('Reference Humidity');

% Add dummy label to plot
x_range = max(measured_humidity) - min(measured_humidity);
y_range = max(reference_humidity) - min(reference_humidity);

xpos = min(measured_humidity) + 0.05 * x_range;
ypos = max(reference_humidity) - 0.1 * y_range;
grayColor = [.9 .9 .9];

text(xpos, ypos, {fit_eqn; r2_text}, 'FontSize', 10, 'BackgroundColor', grayColor);

legend('Calibration Data','Linear Fit', 'Location', 'best');
grid on;
hold off;