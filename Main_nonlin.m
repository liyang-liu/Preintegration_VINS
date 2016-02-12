clear;
close all;
clc;

load('initX.mat');
% x = x + randn(size(x))/6;
% load('Xgt.mat');
fprintf('Initial Value:\n\t X0=[');
fprintf('%f ', x);
fprintf(']\n'); 
% options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','InitDamping', 0)
xf = lsqnonlin(@fnCnUPredErr_lsqnonlin_wt, x);%, [], [], options);
fprintf('Final Value:\n\t Xf=[');
fprintf('%f ', xf);
fprintf(']\n'); 

load('Xgt.mat');
ef = xf - x;
maxe = max(abs(ef))

%%%%%%%%%%%%%%%
x_Nonlin = xf;
load('x_Jac.mat');
x_diff = x_Nonlin - xf;
dx = max(abs(x_diff));
en = fnCnUPredErr_lsqnonlin_wt(x_Nonlin);
ej = fnCnUPredErr_lsqnonlin_wt(xf);
de = en'*en - ej' * ej;
fprintf('\n The comparison of Jac and Nonlin:\n dx=%f\t de = %f\n', dx, de);

