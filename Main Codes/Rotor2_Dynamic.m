function [XDOT,wStar,W,fPhi,fTheta,fPsi,uFMain] =...
              Rotor2_Dynamic(t,x,u,FaultTime,alpha,gamma,Eta,w)
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
        F2 = u(4);

        w1 = real(sqrt(Sol_Vector(1)/b));
        w2 = real(sqrt(Sol_Vector(2)/b));
        w3 = real(sqrt(Sol_Vector(3)/b));
        w4 = real(sqrt(Sol_Vector(4)/b));

        W = [w1 w2 w3 w4]';
                
    %% B. State Vector
    
    phi = x(1);
    theta = x(3);
    say = x(5);

    xDot = x(8);
    yDot = x(10);
    zDot = x(12);
    
    phiDot = x(2);
    thetaDot = x(4);
    sayDot = x(6);

    wStar = (w1 + w3 - w2 - w4);        % Disturbance    

    uThrust = u(3)+u(4)+u(5)+u(6);

    %% C. Fault Injection 

    s = @(x) sin(x);
    c = @(x) cos(x);

%     if(t> 35)
%         dphi = 0.5*sin(0.7*t);
%         dtheta = 0.5*cos(0.7*t);
%         dpsi = 0.5*cos(-0.7*t);
%     else
        dphi = 0;dtheta=0;dpsi = 0;
%     end
    
    if(t>=FaultTime)

        ufPhi = (F2*(1-Eta)/Ix)*(-(d/b)*s(gamma)*c(alpha)-L*c(gamma)+L/(1-Eta))+...
            (JTP/Ix)*w2*(thetaDot*(1-c(gamma))+s(gamma)*s(alpha)*sayDot);

        ufTheta = -d*F2*(1-Eta)*s(gamma)*s(alpha)/(b*Iy)-...
            (JTP/Iy)*w2*(phiDot*(1-c(gamma))+s(gamma)*c(alpha)*sayDot);

        ufSay = (F2/Iz)*(1-Eta)*(L*s(gamma)*c(alpha)-(d/b)*c(gamma)+d/(b*(1-Eta)))+...
                        (JTP/Iz)*w2*s(gamma)*(-phiDot*s(alpha)+thetaDot*c(alpha));
        
        ufx = (u(1)/m)*((1-Eta)*c(gamma)-1)*F2+...
                    c(say)*c(theta)*(F2/m)*(1-Eta)*(s(gamma)*c(alpha))+...
                    (F2/m)*(s(gamma)*s(alpha))*(1-Eta)*(-s(say)*c(phi)+c(say)*s(theta)*s(phi));

        ufy = (u(2)/m)*((1-Eta)*c(gamma)-1)*F2+...
                    s(say)*c(theta)*(F2/m)*(1-Eta)*(s(gamma)*c(alpha))+...
                    (F2/m)*(s(gamma)*s(alpha))*(1-Eta)*(c(say)*c(phi)+s(say)*s(theta)*s(phi));

        ufz = (c(phi)*c(theta)/m)*((1-Eta)*c(gamma)-1)*F2-...
                    s(theta)*(F2/m)*(1-Eta)*(s(gamma)*c(alpha))+...
                    (F2/m)*(s(gamma)*s(alpha))*(1-Eta)*c(theta)*s(phi);

       xDoubleDot = (1/m)*uThrust*u(1)-Kf*xDot/m+ufx;
       yDoubleDot = (1/m)*uThrust*u(2)-Kf*yDot/m+ufy;
       zDoubleDot = -g+(1/m)*uThrust*(c(theta)*c(phi))-Kf*zDot/m+ufz;

       fPhi = ((Iy-Iz)/Ix)*thetaDot*sayDot+JTP*thetaDot*wStar/Ix-Kt*L*phiDot/Ix;
       fTheta = ((Iz-Ix)/Iy)*phiDot*sayDot-JTP*phiDot*wStar/Iy-(Kt*L/Iy)*thetaDot;
       fPsi = ((Ix-Iy)/Iz)*phiDot*thetaDot-(Kt*L/Iz)*sayDot;

       phiDoubleDot = fPhi+L*(u(end)-u(4))/Ix+ufPhi+dphi;
       thetaDoubleDot = fTheta+L*(u(5)-u(3))/Iy+ufTheta+dtheta;
       psiDoubleDot = fPsi+d*(-u(4)-u(end)+u(3)+u(5))/(b*Iz)+ufSay+dpsi;

    else

        ufx = 0;
        ufy = 0;
        ufz = 0;
        ufPhi = 0;
        ufTheta = 0;
        ufSay = 0;
       xDoubleDot = (1/m)*uThrust*u(1)-Kf*xDot/m+ufx;
       yDoubleDot = (1/m)*uThrust*u(2)-Kf*yDot/m+ufy;
       zDoubleDot = -g+(1/m)*uThrust*(c(theta)*c(phi))-Kf*zDot/m+ufz;

       fPhi = ((Iy-Iz)/Ix)*thetaDot*sayDot+JTP*thetaDot*wStar/Ix-Kt*L*phiDot/Ix+ufPhi;
       fTheta = ((Iz-Ix)/Iy)*phiDot*sayDot-JTP*phiDot*wStar/Iy-(Kt*L/Iy)*thetaDot+ufTheta;
       fPsi = ((Ix-Iy)/Iz)*phiDot*thetaDot-(Kt*L/Iz)*sayDot+ufSay;

       phiDoubleDot = fPhi+L*(u(end)-u(4))/Ix;
       thetaDoubleDot = fTheta+L*(u(5)-u(3))/Iy;
       psiDoubleDot = fPsi+d*(-u(4)-u(end)+u(3)+u(5))/(b*Iz);

    end

    uFMain = [ufPhi ufTheta ufSay ufx ufy ufz ]';
    
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
                       zDoubleDot] + w;
 
end
