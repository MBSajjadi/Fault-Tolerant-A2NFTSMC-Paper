clc;clear;close all;

m = 1;
g = 9.81;

%% Initialization

Ts = 0.01;   %% Must be 0.0001, at least, to be Fully Robust.
tMax = 70;
t0 = 0;
t = t0:Ts:tMax;
N = numel(t);
n = 6;  % Number of DOF (X)

x0 = [0 0 0 0 0 0 2 0 2 0 0 0];       %% Ics
x = zeros(2*n,N);                            %% Compelete State Vector
x(:,1) = x0;

s = @(x) sin(x);
c = @(x) cos(x);

%% Reference Signals

CASE = 1;                   %% Linear Trajectory

% CASE = 2;               %% Circular Trajectory

% CASE = 3;               %% BiLinear Trajectory

% CASE = 4;               %% Hellical Trajectory (Second Derivative is
                                   % Highly Required).

[XD, XDoubleDotD] = setDesiredTrajectory(t,CASE,n);
sayd = XD(5,:);

Tr = [0 -1 0 1
        -1 0 1 0
        1 -1 1 -1
        1 1 1 1];

TransformInverse  =Tr^-1;

%% Control Signal Initialization

nU = 6;             %% iN cOnjunction with the Virtual Signals
u = 0.1*ones(nU,N);
W = 100*ones(4,N);

%% NFTSMC Parameters

bc = 0.01*ones(1,n);       
bPrime = [1 1 1 0.8 0.8 0.8];          
Landa = 2*ones(1,n);
LandaPrime = 1.5*ones(1,n);      % <2 AND >1
% K = [10 10 10 5 5 5];             % Static Gains
EtaController = [1 1 1 0.5 0.5 0.5];

%% Faults

FaultTime = 0;

AlphaAngle = 70*pi/180;
GammaAngle = 30*pi/180;
Eta = 0.5;      %% 30% LOE

%% Fault Observer

dHat = zeros(n,N);
Z = zeros(n,N);
LambDA = 10*[5 2 8 5 5 5]';

n = 6;

%% Main Fault Vector

uF = zeros(n,N);

%% Adaptive Laws

S = zeros(n,N);
KHAT = 3*ones(n,N);
KHAT_PARAMETER = [50 50 50 1 1 12]';     %% Adjust if needed
eDot = zeros(n,N);

%% Main Loop

tic;

for i=2:N  

    [K1RK,wStar,W(:,i),fPhi,fTheta,fPsi,uF(:,i)] =...
     Rotor2_Dynamic(t(i-1),x(:,i-1),u(:,i-1),FaultTime,...
                                AlphaAngle,GammaAngle,Eta);

    x(:,i) = stateCalculation(K1RK,x(:,i-1),...
                                        u(:,i-1),Ts,t(i-1),FaultTime,...
                                        AlphaAngle,GammaAngle,Eta);

    %% Virtual Control Design
    
    ux = u(1,i-1);
    uy = u(2,i-1);
    
    phid = asin(ux*s(sayd(i))-uy*c(sayd(i)));
    thetad = asin((ux-s(sayd(i))*s(phid))/(c(sayd(i))*c(phid)));
    
    XD(1,i) = phid;
    XD(3,i) = thetad;

    %% Disturbance Observer

    if(t(i)>=FaultTime)

      [dHat(:,i),Z(:,i)] = NonlinearObserver(t(i-1),...
                                                        LambDA,Z(:,i-1),x(:,i),...
                                                        Ts,u(:,i-1),fPhi,fTheta,fPsi);

    end
    
    %% Adaptive Tunning

    ro = bPrime(1)*LandaPrime(1)*(abs(eDot(:,i-1))).^(LandaPrime(1)-1);
    KHAT(:,i) = KHAT(:,i-1) + Ts*KHAT_PARAMETER.*(ro.*abs(S(:,i-1)));

        %% NFTSMC 

    [u(:,i),S(:,i),eDot(:,i)] = ...
    NonsingularFastTSMC(wStar,x(:,i),XD(:,i),XDoubleDotD(:,i),bc,...
                                     bPrime, Landa,LandaPrime,...
                                     EtaController,KHAT(:,i),dHat(:,i),TransformInverse);

end

%% PlotResults
 
X = [x(7:12,:)
        x(1:6,:)];

xDesired = [XD(7:12,:)
                  XD(1:6,:)];

plotResults(t,X,xDesired,u,uF,dHat,t(end))

%% Norm Calculation

ERROR = X-xDesired;
ex = ERROR(1,:);
ey = ERROR(3,:);
ez = ERROR(5,:);
ephi = ERROR(7,:);
etheta = ERROR(9,:);
epsi = ERROR(11,:);

TABLE = [norm(ex(end-500:end))
               norm(ey(end-500:end))
               norm(ez(end-500:end))
               norm(ephi(end-500:end))
               norm(etheta(end-500:end))
               norm(epsi(end-500:end))];
