clc;
close all;
format short
A = [-1.064 1.000; 290.26 0.00]
B = [-0.25; -331.40];
C = [-123.34 0.00; 0.00 1.00];
D = [-13.51; 0];
states = {'AoA','q'};
inputs = {'\delta_c'};
outputs = {'Az','q'};
Sys=ss(A,B,C,D,'statename',states,....
'inputname',inputs,...
'outputname',outputs);
%TF 
TFS = tf(sys);
TF = TFS(2,1);
disp(pole(TF));

%LQR weight Matrices
Q = [0.1 0;0 0.1];
R=0.5;

%lQR Gain 
[K,S,e] =lqr(A,B,Q,R);
fprintf('eigenvalues of A-BK\n');
disp(eig(A-B*K));
fprintf('Feedback gain K');
disp(K)

%closed loop
Acl =A-B*K;
Bcl=B;
Syscl=ss(Acl,Bcl,C,D,'statename',states,....
'inputname',inputs,...
'outputname',outputs);

%TF Closed Loop
TFS = tf(sys);
TF = TFS(2,1);

% lQG Of KAlman Filter
G = eye(2);
H =1*eye(2);

%Kalman Q,R Noise Matrices
Qbar = diag(0.00015*ones(1,2));
Rbar = diag(0.55*ones(1,2));

%define noisy system
Sys_n=ss(A,[B G],C,[D H ]);
[kest,L,P]=kalman(Sys_n,Qbar,Rbar,0);
Aob = A-L*C;

fprintf('observer eigenvalues\n');
disp(eig(Aob));
dT1=0.75;
dT2=0.25;

%Missile Parameters 

R =6371e3;
vel=1021.08;
m2f=3.2811;

%Target Location
LAT_TARGET=34.6588;
LON_TARGET=-118.79745;
ELEV_TARGET=795;

%Target Location
LAT_INIT=34.2329;
LON_INIT=-119.4573;
ELEV_INIT=10000;

%OBSTCLE lOCATION
LAT_OBS =34.61916;
LON_OBS =-118.8429;

d2r=pi/180;

l1=LAT_INIT*d2r;
u1=LON_INIT*d2r;
l2=LAT_TARGET*d2r;
u2=LON_TARGET*d2r;

dl=l2-l1;
du=u2-u1;

a=sin(dl/2)^2+cos(l1)*cos(l2)*sin(du/2)^2;
c=2*atan2(sqrt(a),sqrt(1-a));
d=R*C;
r=sqrt(d^2+(ELEV_TARGET-ELEV_INIT)^2);
yaw_init=azimuth(LAT_INIT,LON_INIT,LAT_TARGET,LON_TARGET);
yaw=yaw_init*d2r;

dh=abs(ELEV_TARGET-ELEV_INIT);
FPA_INIT =atan(dh ./d); 







































