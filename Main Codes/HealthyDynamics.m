function XDOT = HealthyDynamics(xHat,x,u,ObserverGain,C)
    %% A. System Parameters
    
        L = 0.47/2;      % One-Half Length. Full Length equals 2*L = 47cm
        m = 1;             % Mass of the Quadrotor
        g = 9.81;

        Ix = 0.0081;        % X Axis Moment of Intertia
        Iy = Ix;               % Y Axis Moment of Intertia
        Iz = 0.0142;        % Z Axis MOMENT of Intertia
        JTP = 10.4e-5;    
        
        b = 5.42e-5;    % Drag Force Coefficient
        d = 1.1e-6;      % Drag Torque Coefficient
        
        Kf = 1e-6;
        Kt = 1.2e-6;

        %% Control Part
                                               
        Sol_Vector = [u(3) u(4) u(5) u(6)]';        % [F1, F2, F3, F4]'

        w1 = real(sqrt(Sol_Vector(1)/b));
        w2 = real(sqrt(Sol_Vector(2)/b));
        w3 = real(sqrt(Sol_Vector(3)/b));
        w4 = real(sqrt(Sol_Vector(4)/b));
                
    %% B. State Vector
    
        phi = xHat(1);
        theta = xHat(3);
    
        xDot = xHat(8);
        yDot = xHat(10);
        zDot = xHat(12);
        
        phiDot = xHat(2);
        thetaDot = xHat(4);
        sayDot = xHat(6);
    
        wStar = (w1 + w3 - w2 - w4);        % Disturbance    
    
        uThrust = u(3)+u(4)+u(5)+u(6);
    
        %% C. Fault Injection 
    
        c = @(x) cos(x);
    
       xDoubleDot = (1/m)*uThrust*u(1)-Kf*xDot/m;
       yDoubleDot = (1/m)*uThrust*u(2)-Kf*yDot/m;
       zDoubleDot = -g+(1/m)*uThrust*(c(theta)*c(phi))-Kf*zDot/m;
    
       fPhi = ((Iy-Iz)/Ix)*thetaDot*sayDot+JTP*thetaDot*wStar/Ix-Kt*L*phiDot/Ix;
       fTheta = ((Iz-Ix)/Iy)*phiDot*sayDot-JTP*phiDot*wStar/Iy-(Kt*L/Iy)*thetaDot;
       fPsi = ((Ix-Iy)/Iz)*phiDot*thetaDot-(Kt*L/Iz)*sayDot;
    
       phiDoubleDot = fPhi+L*(u(end)-u(4))/Ix;
       thetaDoubleDot = fTheta+L*(u(5)-u(3))/Iy;
       psiDoubleDot = fPsi+d*(-u(4)-u(end)+u(3)+u(5))/(b*Iz);

      %% State Space 
       
         XDOT =  [phiDot
                       phiDoubleDot
                       thetaDot
                       thetaDoubleDot
                       sayDot
                       psiDoubleDot
                       xDot
                       xDoubleDot
                       yDot
                       yDoubleDot
                       zDot
                       zDoubleDot] + ObserverGain*C*(x-xHat);
 
end
