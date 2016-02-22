function [dp,dv,dphi] = fn_PredictIntlDelta(pu1,pu2,Ru1,Ru2,v1,v2,g,dbf,dbw,dt,J)


ddpdbf = J(1:3, 10:12); ddpdbw = J(1:3, 13:15);
ddvdbf = J(4:6, 10:12); ddvdbw = J(4:6, 13:15);
ddphidbw = J(7:9, 13:15);

dp = Ru1 * (pu2 - pu1 - v1 * dt - 0.5 * g * dt * dt) ...
    - ddpdbf * dbf - ddpdbw * dbw;

dv = Ru1 * (v2-v1-g*dt) - ddvdbf * dbf - ddvdbw * dbw;

[a,b,g] = fn_ABGFromR( Ru2 * (Ru1)' );

dphi = [a;b;g] - ddphidbw * dbw;

     