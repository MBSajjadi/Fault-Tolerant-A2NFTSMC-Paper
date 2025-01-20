function ri_L = LowerBoundsOfResidues(Residuals,Ts,T,KL1i,KL2i,KL3i)

    VectorOfInnerIntegral = sum(Residuals,2);
    
    InnerIntegralX = VectorOfInnerIntegral(1)*Ts;
    InnerIntegralY = VectorOfInnerIntegral(2)*Ts;
    InnerIntegralZ = VectorOfInnerIntegral(3)*Ts;
    InnerIntegralPHI = VectorOfInnerIntegral(4)*Ts;
    InnerIntegralTHETA = VectorOfInnerIntegral(5)*Ts;
    InnerIntegralPSI = VectorOfInnerIntegral(6)*Ts;

    %% First Parameters Set
    
    KL1X = KL1i(1);
    KL1Y = KL1i(2);
    KL1Z = KL1i(3);
    
    KL1PHI = KL1i(4);
    KL1THETA = KL1i(5);
    KL1PSI = KL1i(6);
    
    %% Second Parameters Set
    
    KL2X = KL1i(1);
    KL2Y = KL2i(2);
    KL2Z = KL2i(3);
    
    KL2PHI = KL2i(4);
    KL2THETA = KL2i(5);
    KL2PSI = KL2i(6);
    
    %% Third Parameters Set
    
    KL3X = KL3i(1);
    KL3Y = KL3i(2);
    KL3Z = KL3i(3);
    
    KL3PHI = KL3i(4);
    KL3THETA = KL3i(5);
    KL3PSI = KL3i(6);
    
    %% Main Integrals
    
    MainIntegral_X = (KL1X/T)*InnerIntegralX +...
                               (KL2X/T)*InnerIntegralX - ...
                               (KL2X/T^2)*InnerIntegralX*Ts + KL3X;
                           
    MainIntegral_Y = (KL1Y/T)*InnerIntegralY +...
                               (KL2Y/T)*InnerIntegralY - ...
                               (KL2Y/T^2)*InnerIntegralY*Ts + KL3Y;
                           
    MainIntegral_Z = (KL1Z/T)*InnerIntegralZ +...
                               (KL2Z/T)*InnerIntegralZ - ...
                               (KL2Z/T^2)*InnerIntegralZ*Ts + KL3Z;
                           
    MainIntegral_PHI = (KL1PHI/T)*InnerIntegralPHI +...
                               (KL2PHI/T)*InnerIntegralPHI - ...
                               (KL2PHI/T^2)*InnerIntegralPHI*Ts + KL3PHI;
                           
    MainIntegral_THETA = (KL1THETA/T)*InnerIntegralTHETA +...
                               (KL2THETA/T)*InnerIntegralTHETA - ...
                               (KL2THETA/T^2)*InnerIntegralTHETA*Ts + KL3THETA;
                           
    MainIntegral_PSI = (KL1PSI/T)*InnerIntegralPSI +...
                               (KL2PSI/T)*InnerIntegralPSI - ...
                               (KL2PSI/T^2)*InnerIntegralPSI*Ts + KL3PSI;
              
    %% Final Output
    
    ri_L = [ MainIntegral_PHI
                MainIntegral_THETA
                MainIntegral_PSI
                MainIntegral_X
                MainIntegral_Y
                MainIntegral_Z];
                          
end