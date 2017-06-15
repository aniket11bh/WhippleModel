bicycleParam;
v = 1;
La = 1;

K_lean = zeros(1,4);
K_lean(1) = 7843.20023833066;
K_lean(3) = 424.726009043642;

K_steer = zeros(1,4);
K_steer(2) =33.0078;
K_steer(4) = 5.4516;

gainK = [K_lean; K_steer];