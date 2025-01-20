function [dHat,zNew] = NonlinearObserver(tOld,lambda,zOld,x,Ts,u,fphi,ftheta,fpsi)
    
    Kf = 1e-6;
    m = 1;
    L = 0.47/2;
    b = 5.42e-5;    % Drag Force Coefficient
    d = 1.1e-6;      % Drag Torque Coefficient

    Ix = 0.0081;        % X Axis Moment of Intertia
    Iy = Ix;               % Y Axis Moment of Intertia
    Iz = 0.0142;        % Z Axis MOMENT of Intertia
    g = 9.81;

    F4 = u(end);
    F3 = u(end-1);
    F1 = u(3);
    F2 = u(4);
    uThrust = F1+F3+F4+F2;
    c = @(x) cos(x);
    phi = x(1);
    theta = x(3);

    zDot = @(t,z) [-lambda(1)*z(1)-lambda(1)*(lambda(1)*x(2)+fphi+(L/Ix)*(F4-F2))
                          -lambda(2)*z(2)-lambda(2)*(lambda(2)*x(4)+ftheta+(L/Iy)*(F3-F1))
                          -lambda(3)*z(3)-lambda(3)*(lambda(3)*x(6)+fpsi+(d/(b*Iz))*(F1+F3-F4-F2))
                          -lambda(4)*z(4)-lambda(4)*(lambda(4)*x(8)-(Kf/m)*x(8)+u(1)*uThrust/m)
                          -lambda(5)*z(5)-lambda(5)*(lambda(5)*x(10)-(Kf/m)*x(10)+u(2)*uThrust/m)
                          -lambda(6)*z(6)-lambda(6)*(lambda(6)*x(12)-(Kf/m)*x(12)-g+uThrust*c(phi)*c(theta)/m)];

    K1RK = zDot(tOld, zOld);

    K2RK = zDot(tOld+Ts/2, zOld+Ts*K1RK/2);
    
    K3RK = zDot(tOld+Ts/2, zOld+Ts*K2RK/2);
    
    K4RK = zDot(tOld+Ts, zOld+Ts*K3RK);    
    
    zNew= zOld+...
                (Ts/6)*(K1RK + 2*K2RK + 2*K3RK + K4RK);

    dHat = [zNew(1)+lambda(1)*x(2)
                zNew(2)+lambda(2)*x(4)
                zNew(3)+lambda(3)*x(6)
                zNew(4)+lambda(4)*x(8)
                zNew(5)+lambda(5)*x(10)
                zNew(6)+lambda(6)*x(12)];

end
