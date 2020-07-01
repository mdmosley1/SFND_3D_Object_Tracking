# Plot the ttc values vs the frame number
clear all; close all;

fname = "/tmp/sensor-fusion-camera.csv";
d = csvread(fname);

d(1,:) = []; # delete header values
lidarTTC = d(:,1);
cameraTTC = d(:,2);

figure();
plot(lidarTTC); hold all;
plot(cameraTTC);
legend("lidar TTC", "cameraTTC");
grid on
