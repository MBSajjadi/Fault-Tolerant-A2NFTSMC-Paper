function L = LyapunovEquation(A,C,DeltaLyapunov)

    n = 6;
    I = eye(2*n);
    
    AA = A'+DeltaLyapunov*I;
    BB = A;
    CC = -C'*C;
    
    P = lyap(AA,BB,CC);
    
    L = P^-1*C';
    
end