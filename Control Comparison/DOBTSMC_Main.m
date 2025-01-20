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

x0 = [0 0 0 0 0 0 2 0 2 0 2 0];       %% Ics
x = zeros(2*n,N);                            %% Compelete State Vector
x(:,1) = x0;

s = @(x) sin(x);
c = @(x) cos(x);

%% Reference Signals

% CASE = 1;                   %% Linear Trajectory

% CASE = 2;               %% Circular Trajectory

CASE = 3;               %% BiLinear Trajectory

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
bPrime = 4*[1 1 0.5 1 1 0.8];          
Landa = 2*ones(1,n);
LandaPrime = 1.5*ones(1,n);      % <2 AND >1
LandaPrime(3) = 1.8;
K = [20 20 10 8 10 8];             % Static Gains
EtaController = 0*[1 1 1 0.5 0.5 0.5];

%% Faults

FaultTime = 0;

AlphaAngle = 50*pi/180;
GammaAngle = 20*pi/180;
Eta = 0.3;      %% 30% LOE

%% Fault Observer

dHat = zeros(n,N);
Z = zeros(n,N);
LambDA = 8*[5 2 8 5 5 5]';

%% Observer

n = 6;

%% Main Fault Vector

uF = zeros(n,N);

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
    
    phid = 0;
    thetad = 0;
    
    %% Desired Roll/Pittch Angles
    
    XD(1,i) = phid;
    XD(3,i) = thetad;

    %% Disturbance Observer

    if(t(i)>=FaultTime)

      [dHat(:,i),Z(:,i)] = NonlinearObserver(t(i-1),...
                                                        LambDA,Z(:,i-1),x(:,i),...
                                                        Ts,u(:,i-1),fPhi,fTheta,fPsi);

    end
    

        %% NFTSMC 

    [u(:,i),~,~] = ...
    NonsingularFastTSMC(wStar,x(:,i),XD(:,i),XDoubleDotD(:,i),bc,...
                                     bPrime, Landa,LandaPrime,...
                                     EtaController, K,dHat(:,i),TransformInverse);

end

%% PlotResults
 
X = [x(7:12,:)
        x(1:6,:)];

xDesired = [XD(7:12,:)
                  XD(1:6,:)];

plotResults(t,X,xDesired,u,uF,dHat,t(end))
