function y = RK4(funX, x, A, B, u, dt, ti)

    K1 = funX(ti, x, A, B, u);
    K2 = funX(ti, x+(dt/2)*K1, A, B, u);
    K3 = funX(ti, x+(dt/2)*K2, A, B, u);
    K4 = funX(ti, x+dt*K3, A, B, u);

    y = x + (dt/6)*(K1 + 2*K2 + 2*K3 + K4);

end