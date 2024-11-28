filename = 'filtered_bmp_altitude.csv';
t = readtable(filename);
raw_altitude = t{:,1};
kalman_filtered_altitude = t{:,2}; 

plot(raw_altitude,'--','linewidth',3);
hold on;
plot(kalman_filtered_altitude, 'linewidth',3);
title("Raw and filtered altitude, resting position");
ylabel("Altitude (m)")
grid on;
legend("Raw","Kalman filtered ");
colorbar;
