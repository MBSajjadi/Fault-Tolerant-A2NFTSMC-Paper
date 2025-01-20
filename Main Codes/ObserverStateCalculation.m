function XHAT = ObserverStateCalculation...
                    (K1RK,xHat,x,u,Ts,ObserverGain,C)

     K2RK = HealthyDynamics(xHat+Ts*K1RK/2,x, u,ObserverGain,C);
    
    K3RK = HealthyDynamics(xHat+Ts*K2RK/2,x, u,ObserverGain,C);
    
    K4RK = HealthyDynamics(xHat+Ts*K3RK,x, u,ObserverGain,C);    
    
    XHAT = xHat+...
           (Ts/6)*(K1RK + 2*K2RK + 2*K3RK + K4RK);

end