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
% K = [20 20 20 7 7 7];             % Static Gains
EtaController = [1 1 1 0.5 0.5 0.5];

%% Faults

FaultTime = 25;

AlphaAngle = 50*pi/180;
GammaAngle = 20*pi/180;
Eta = 0.3;      %% 30% LOE

%% Fault Observer

dHat = zeros(n,N);
Z = zeros(n,N);
LambDA = 8*[5 2 8 5 5 5]';

%% Observer

xHat = x;
% xHat(:,1) = 0.1*ones(2*n,1);        % In order for residuals not to deviate at the beginning
n = 6;
C = eye(2*n);
% C = [1 0 0 0 0 0 0 0 0 0 0 0
%         0 0 1 0 0 0 0 0 0 0 0 0
%         0 0 0 0 1 0 0 0 0 0 0 0
%         0 0 0 0 0 0 1 0 0 0 0 0
%         0 0 0 0 0 0 0 0 1 0 0 0
%         0 0 0 0 0 0 0 0 0 0 1 0];

L = 0.47/2;      % One-Half Length. Full Length equals 2*L = 47cm

Ix = 0.0081;        % X Axis Moment of Intertia
Iy = Ix;               % Y Axis Moment of Intertia
Iz = 0.0142;        % Z Axis MOMENT of Intertia

Kf = 1e-6;
Kt = 1.2e-6;

A = [0     1           zeros(1,10)
        0 -Kt*L/Ix     zeros(1,10)
        0    0     0   1 zeros(1,8)
        0    0     0  -Kt*L/Iy zeros(1,8)
        0    0     0 0 0 1 zeros(1,6)
        0    0     0 0 0 -Kt*L/Iz zeros(1,6)
        zeros(1,7) 1 0 0 0 0
        zeros(1,7) -Kf/m 0 0 0 0
        zeros(1,9) 1 0 0
        zeros(1,9) -Kf/m 0 0
        zeros(1,11) 1
        zeros(1,11) -Kf/m]; 

DeltaLyapunov = 2;
ObserverGain = LyapunovEquation(A,C,DeltaLyapunov);

%% Main Fault Vector

uF = zeros(n,N);

%% Residuals Parameters

xBarResidue = zeros(n,N);
UpperBoundOfResidualsVector = xBarResidue;
LowerBoundOfResidualsVector = xBarResidue;

j = 0;          % Iteration Num.

%{
     alpha = 50, gamma = 20, Eta = 0.3
%}
Ku1i = [0.1 0.1 1 0.01 0.01 1];                            %% [phi theta psi x y z]
Ku2i = [0.05 0.05 0.05 0.05 0.01 0.05];
Ku3i = [0.5 0.5 0.5 0.1 0.1 0.5];

%{
     alpha = 10, gamma = 5, Eta = 0.1
%}
% Ku1i = [1 1 1 0.01 0.1 0.1];                            %% [phi theta psi x y z]
% Ku2i = [0.05 0.05 0.05 0.05 0.01 0.05];
% Ku3i = [0.5 0.5 0.5 0.05 0.05 0.05];

KL1i = Ku1i;
KL2i = Ku2i;
KL3i = -Ku3i;

h = 10;
TimeWindow = h*Ts;

%% Adaptive Laws

S = zeros(n,N);
KHAT = 3*ones(n,N);
KHAT_PARAMETER = [50 50 50 5 5 5]';     %% Adjust if needed
eDot = zeros(n,N);

%% Noise

VarianceW = 0.00;
VarianceR = 0.00;

Q = VarianceW*diag([ones(1,12)]);
R = VarianceR*diag([ones(1,12)]);

for i=1:12
    Q(2*i,2*i) = 0;     % fOR System Dynamics
end

w = sqrt(VarianceW)*randn(2*n,N);
v = sqrt(VarianceR)*randn(2*n,N);

%% Main Loop

tic;

for i=2:N  

    [K1RK,wStar,W(:,i),fPhi,fTheta,fPsi,uF(:,i)] =...
     Rotor2_Dynamic(t(i-1),x(:,i-1),u(:,i-1),FaultTime,...
                                AlphaAngle,GammaAngle,Eta,w(:,i-1));

    x(:,i) = stateCalculation(K1RK,x(:,i-1),...
                                        u(:,i-1),Ts,t(i-1),FaultTime,...
                                        AlphaAngle,GammaAngle,Eta,w(:,i-1),v(:,i));
    
    %% Healthy Observer

    K1RK4 = HealthyDynamics(xHat(:,i-1),x(:,i-1),u(:,i-1),ObserverGain,C);
    xHat(:,i) = ObserverStateCalculation(K1RK4,xHat(:,i-1),x(:,i-1),u(:,i-1),Ts,ObserverGain,C);

    %% Residual Evaluation

         xBarResidue(:,i) = [x(7,i)-xHat(7,i)       % [x,y,z,phi,theta,psi]
                                      x(9,i)-xHat(9,i)
                                      x(11,i)-xHat(11,i)
                                      x(1,i)-xHat(1,i)
                                      x(3,i)-xHat(3,i)
                                      x(5,i)-xHat(5,i)];       % [x,y,z,phi,theta,psi]'
         
         if (t(i)-TimeWindow>=0)
             
           j=j+1;  % (t-T)
           
           UpperBoundOfResidualsVector(:,i) = ...
               UpperBoundsOfResidues(xBarResidue(:,j:i),Ts,TimeWindow,Ku1i,Ku2i,Ku3i);
           
            LowerBoundOfResidualsVector(:,i) = ...
               LowerBoundsOfResidues(xBarResidue(:,j:i),Ts,TimeWindow,KL1i,KL2i,KL3i);
             
         end

    %% Virtual Control Design
    
    ux = u(1,i-1);
    uy = u(2,i-1);
    
    phid = asin(ux*s(sayd(i))-uy*c(sayd(i)));
    thetad = asin((ux-s(sayd(i))*s(phid))/(c(sayd(i))*c(phid)));
    
    %% Desired Roll/Pittch Angles
    
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
                                     EtaController, KHAT(:,i),dHat(:,i),TransformInverse);

end

%% PlotResults
 
X = [x(7:12,:)
        x(1:6,:)];

xDesired = [XD(7:12,:)
                  XD(1:6,:)];

plotResults(t,X,xDesired,u,uF,dHat,xBarResidue,...
    UpperBoundOfResidualsVector,LowerBoundOfResidualsVector,KHAT,t(end))
