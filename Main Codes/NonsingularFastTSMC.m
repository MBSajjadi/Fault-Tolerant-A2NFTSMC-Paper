function [u,S,eDot] = ...
    NonsingularFastTSMC(wStar,x,XD,XDoubleDotD,bc,...
                                     bPrime, Landa,LandaPrime, EtaController,K,dHat,T)
    
    xDot = x(8);
    yDot = x(10);
    zDot =x(12);
    phiDot = x(2);
    thetaDot = x(4);
    psiDot = x(6);
    g = 9.81;
    
    L = 0.47/2;          % One-Half Length. Full Length equals 2*L = 47cm
    m = 1;                % Mass of the Quadrotor

    Ix = 0.0081;        % X Axis Moment of Intertia
    Iy = Ix;               % Y Axis Moment of Intertia
    Iz = 0.0142;        % Z Axis MOMENT of Intertia
    JTP = 10.4e-5; 

    b = 5.42e-5;    % Drag Force Coefficient
    d = 1.1e-6;      % Drag Torque Coefficient

    Kf = 1e-6;
    Kt = 1.2e-6;
        
    EtaX = EtaController(4);
    EtaY = EtaController(5);
    EtaZ =  EtaController(6);
    
    EtaPhi = EtaController(1);
    EtaTheta = EtaController(2);
    EtaPsi = EtaController(3);
    
    Kx = K(4);
    Ky = K(5);
    Kz = K(6); 
    Kphi = K(1);
    Ktheta = K(2);
    Kpsi = K(3);
    
    bx = bc(4);
    by = bc(5);
    bz = bc(6);
    bPhi = bc(1);
    bTheta = bc(2);
    bPsi = bc(3);
    
    bPrimeX = bPrime(4);
    bPrimeY = bPrime(5);
    bPrimeZ = bPrime(6);
    bPrimePhi = bPrime(1);
    bPrimeTheta = bPrime(2);
    bPrimePsi = bPrime(3);
    
    LandaX = Landa(4);
    LandaY = Landa(5);
    LandaZ = Landa(6);
    LandaPhi = Landa(1);
    LandaTheta = Landa(2);
    LandaPsi = Landa(3);
    
    LandaPrimeX = LandaPrime(4);
    LandaPrimeY = LandaPrime(5);
    LandaPrimeZ = LandaPrime(6);
    LandaPrimePhi = LandaPrime(1);
    LandaPrimeTheta = LandaPrime(2);
    LandaPrimePsi = LandaPrime(3);
    
    e = x-XD;
    
    ex = e(7);
    ey = e(9);
    ez = e(11);
    ePhi = e(1);
    eTheta = e(3);
    ePsi = e(5);
    
    eDotx = e(8);
    eDoty = e(10);
    eDotz = e(12);
    eDotPhi = e(2);
    eDotTheta = e(4);
    eDotPsi = e(6);

    eDot = [eDotPhi eDotTheta eDotPsi eDotx eDoty eDotz]';
    
    %% Nonsingular Sliding Surfaces
    
    Sx = ex +...
        bx*((abs(ex))^LandaX)*sign(ex) +...
        bPrimeX*(abs(eDotx)^LandaPrimeX)*sign(eDotx);
    
    Sy = ey +...
        by*((abs(ey))^LandaY)*sign(ey) +...
        bPrimeY*(abs(eDoty)^LandaPrimeY)*sign(eDoty);
    
    Sz = ez +...
        bz*((abs(ez))^LandaZ)*sign(ez) +...
        bPrimeZ*(abs(eDotz)^LandaPrimeZ)*sign(eDotz);
    
    Sphi = ePhi +...
        bPhi*((abs(ePhi))^LandaPhi)*sign(ePhi) +...
        bPrimePhi*(abs(eDotPhi)^LandaPrimePhi)*sign(eDotPhi);
    
    Stheta = eTheta +...
        bTheta*((abs(eTheta))^LandaTheta)*sign(eTheta) +...
        bPrimeTheta*(abs(eDotTheta)^LandaPrimeTheta)*sign(eDotTheta);
    
    Spsi = ePsi +...
              bPsi*((abs(ePsi))^LandaPsi)*sign(ePsi) +...
              bPrimePsi*(abs(eDotPsi)^LandaPrimePsi)*sign(eDotPsi);
   
    S = [Sphi Stheta Spsi Sx Sy Sz]';

    %% Z-Direction
    
    RegulationPartZ = -bPrimeZ^-1*LandaPrimeZ^-1*(abs(eDotz)^(2-LandaPrimeZ))*sign(eDotz)*...
    (1+bz*LandaZ*abs(ez)^(LandaZ-1));
    TrackingPartZ = g+Kf*zDot/m+XDoubleDotD(3)-EtaZ*Sz-Kz*tanh(Sz)-dHat(6);
    uz = (RegulationPartZ+TrackingPartZ)/(cos(x(1))*cos(x(3)));

    %% X-Direction
    
    RegulationPartX = -bPrimeX^-1*LandaPrimeX^-1*(abs(eDotx)^(2-LandaPrimeX))*sign(eDotx)*...
        (1+bx*LandaX*abs(ex)^(LandaX-1));
    TrackingPartX = Kf*xDot/m+XDoubleDotD(1)-EtaX*Sx-Kx*tanh(Sx)-dHat(4);
    ux = (RegulationPartX+TrackingPartX)/uz;
    
    %% Y-Direction
    
    RegulationPartY = -bPrimeY^-1*LandaPrimeY^-1*(abs(eDoty)^(2-LandaPrimeY))*sign(eDoty)*...
    (1+by*LandaY*abs(ey)^(LandaY-1));
    TrackingPartY = Kf*yDot/m+XDoubleDotD(2)-EtaY*Sy-Ky*tanh(Sy)-dHat(5);
    uy = (RegulationPartY+TrackingPartY)/uz;
    
    %% PHI-Direction
    
    RegulationPartPhi = bPrimePhi^-1*LandaPrimePhi^-1*(abs(eDotPhi)^(2-LandaPrimePhi))*sign(eDotPhi)*...
    (1+bPhi*LandaPhi*abs(ePhi)^(LandaPhi-1));
    TrackingPartPhi = -XDoubleDotD(4)+((Iy-Iz)/Ix)*thetaDot*psiDot+...
        JTP*thetaDot*wStar/Ix-Kt*L*phiDot/Ix+EtaPhi*Sphi+Kphi*tanh(Sphi);
    uPhi = (-Ix/L)*(dHat(1)+RegulationPartPhi+TrackingPartPhi);
    
    %% Theta-Direction
    
    RegulationPartTheta = bPrimeTheta^-1*LandaPrimeTheta^-1*(abs(eDotTheta)^(2-LandaPrimeTheta))*sign(eDotTheta)*...
    (1+bTheta*LandaTheta*abs(eTheta)^(LandaTheta-1));
    TrackingPartTheta = -XDoubleDotD(5)+((Iz-Ix)/Iy)*phiDot*psiDot...
        -JTP*phiDot*wStar/Iy-Kt*L*thetaDot/Iy+EtaTheta*Stheta+Ktheta*tanh(Stheta);
    uTheta = (-Iy/L)*(dHat(2)+RegulationPartTheta+TrackingPartTheta);
    
    %% PSI-Direction
    
    RegulationPartPsi = bPrimePsi^-1*LandaPrimePsi^-1*(abs(eDotPsi)^(2-LandaPrimePsi))*sign(eDotPsi)*...
    (1+bPsi*LandaPsi*abs(ePsi)^(LandaPsi-1));
    TrackingPartPsi = -XDoubleDotD(6)+((Ix-Iy)/Iz)*phiDot*thetaDot-Kt*L*psiDot/Iz...
                              +EtaPsi*Spsi+Kpsi*tanh(Spsi);
    uPsi = (-(Iz*b)/d)*(dHat(3)+RegulationPartPsi+TrackingPartPsi);

    %% NFTSMC Cont.

    F =  T*[uPhi uTheta uPsi uz]';
    u = ([ux uy F(1) F(2) F(3) F(4)]');
    
end
