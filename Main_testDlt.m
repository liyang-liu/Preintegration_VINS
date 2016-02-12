
clear;
close all;
clc;

dv = 0; dp = 0; dt = 1e-3;
for k=1:999
    dp = dp + dv*dt;
    dv = dv + 9.8*dt;
end

dp
dv