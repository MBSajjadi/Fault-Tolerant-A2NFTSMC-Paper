function [u,e,uz] = SlidingModeControl(x,XD,xDoubleDotd,K,...
                                                      a,fPhi,fTheta,fPsi,TransformInverse,dHat)
   
    m = 1;
    Kf = 1e-6;
    e = x - XD;
    g = 9.81;
    L = 0.47/2;

    b = 5.42e-5;    % Drag Force Coefficient
    d = 1.1e-6;      % Drag Torque Coefficient

    Ix = 0.0081;        % X Axis Moment of Intertia
    Iy = Ix;               % Y Axis Moment of Intertia
    Iz = 0.0142;        % Z Axis MOMENT of Intertia

    S = [a(1)*e(1)+e(2)    % phi
           a(2)*e(3)+e(4)     % theta
           a(3)*e(5)+e(6)      % psi
           a(4)*e(7)+e(8)       % x
           a(5)*e(9)+e(10)      % y
           a(6)*e(11)+e(12)];    % z

    %% Altitude Control

    uz = m*(-dHat(6)+g-a(6)*e(12)+xDoubleDotd(3)+Kf*x(12)/m-K(6)*tanh(S(6)))/(cos(x(1))*cos(x(3)));
    ux = m*(-dHat(4)-a(4)*e(8)+xDoubleDotd(1)+Kf*x(8)/m-K(4)*tanh(S(4)))/uz;
    uy =  m*(-dHat(5)-a(5)*e(10)+xDoubleDotd(2)+Kf*x(10)/m-K(5)*tanh(S(5)))/uz;

    %% Atitude Control

    uPhi = (-Ix/L)*(dHat(1)+fPhi+a(1)*e(2)-xDoubleDotd(4)+K(1)*tanh(S(1)));
    uTheta =  (-Iy/L)*(dHat(2)+fTheta+a(2)*e(4)-xDoubleDotd(5)+K(2)*tanh(S(2)));
    uPsi = (-b*Iz/d)*(dHat(3)+a(3)*e(6)-xDoubleDotd(6)+fPsi+K(3)*tanh(S(3)));

    F = TransformInverse*[uPhi uTheta uPsi uz]';
    u = [ux,uy,F(1),F(2),F(3),F(4)]';

end