function ri_u = UpperBoundsOfResidues(Residuals,Ts,T,Ku1i,Ku2i,Ku3i)

    VectorOfInnerIntegral = sum(Residuals,2);
    
    InnerIntegralX = VectorOfInnerIntegral(1)*Ts;       % Integral (r*dtau)
    InnerIntegralY = VectorOfInnerIntegral(2)*Ts;
    InnerIntegralZ = VectorOfInnerIntegral(3)*Ts;
    InnerIntegralPHI = VectorOfInnerIntegral(4)*Ts;
    InnerIntegralTHETA = VectorOfInnerIntegral(5)*Ts;
    InnerIntegralPSI = VectorOfInnerIntegral(6)*Ts;

    %% First Parameters Set
    
    Ku1X = Ku1i(1);
    Ku1Y = Ku1i(2);
    Ku1Z = Ku1i(3);
    
    Ku1PHI = Ku1i(4);
    Ku1THETA = Ku1i(5);
    Ku1PSI = Ku1i(6);
    
    %% Second Parameters Set
    
    Ku2X = Ku2i(1);
    Ku2Y = Ku2i(2);
    Ku2Z = Ku2i(3);
    
    Ku2PHI = Ku2i(4);
    Ku2THETA = Ku2i(5);
    Ku2PSI = Ku2i(6);
    
    %% Third Parameters Set
    
    Ku3X = Ku3i(1);
    Ku3Y = Ku3i(2);
    Ku3Z = Ku3i(3);
    
    Ku3PHI = Ku3i(4);
    Ku3THETA = Ku3i(5);
    Ku3PSI = Ku3i(6);
    
    %% Main Integrals
    
    MainIntegral_X = (Ku1X/T)*InnerIntegralX +...
                               (Ku2X/T)*InnerIntegralX - ...
                               (Ku2X/T^2)*InnerIntegralX*Ts + Ku3X;
                           
    MainIntegral_Y = (Ku1Y/T)*InnerIntegralY +...
                               (Ku2Y/T)*InnerIntegralY - ...
                               (Ku2Y/T^2)*InnerIntegralY*Ts + Ku3Y;
                           
    MainIntegral_Z = (Ku1Z/T)*InnerIntegralZ +...
                               (Ku2Z/T)*InnerIntegralZ - ...
                               (Ku2Z/T^2)*InnerIntegralZ*Ts + Ku3Z;
                           
    MainIntegral_PHI = (Ku1PHI/T)*InnerIntegralPHI +...
                               (Ku2PHI/T)*InnerIntegralPHI - ...
                               (Ku2PHI/T^2)*InnerIntegralPHI*Ts + Ku3PHI;
                           
    MainIntegral_THETA = (Ku1THETA/T)*InnerIntegralTHETA +...
                               (Ku2THETA/T)*InnerIntegralTHETA - ...
                               (Ku2THETA/T^2)*InnerIntegralTHETA*Ts + Ku3THETA;
                           
    MainIntegral_PSI = (Ku1PSI/T)*InnerIntegralPSI +...
                               (Ku2PSI/T)*InnerIntegralPSI - ...
                               (Ku2PSI/T^2)*InnerIntegralPSI*Ts + Ku3PSI;
              
    %% Final Output
    
    ri_u = [MainIntegral_PHI
                MainIntegral_THETA
                MainIntegral_PSI
                MainIntegral_X
                MainIntegral_Y
                MainIntegral_Z];
                           
end
