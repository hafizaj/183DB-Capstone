s = tf('s');
%motor transfer function 
Kt = 22.84; %torque constant 
  % velocity constant 
Kb = 13.18; % back emf constant
Ra = 2.2; %armature resistance
La = 18.5;
Jm = 0.15; %rotational inertia of the motor
omega_0 = 210;
I_0 = 8.4;
Dm = Kt*I_0/omega_0;
K = Kt/(Jm*Ra);
G = Kt/((Ra+La*s)*(Jm*s^2+Dm*s) + Kt*Kb*s)
%G = K/(s*(s + (1/Jm)*(Kb*Kt/Ra + Dm)));
L = feedback(G,1);
figure(1)
step (L);
stepinfo(L);
%need PID to reduce settling time 

