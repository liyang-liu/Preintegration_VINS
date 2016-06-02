function [ e ] = fnCnUPredErr_lsqnonlin_wt( x )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

load('CovMatrixInv.mat');
e = fnCnUPredErr_lsqnonlin(x);
e = sqrtm(CovMatrixInv)*e;

end

