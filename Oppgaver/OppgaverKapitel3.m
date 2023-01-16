%% Oppgave 3.7

figure(1)
title("Initial")
tpoly(0, 1, 50)
% s0 (0) er start, sf(1) er slutt, m(50) er steg
figure(2)
title("High_start_velocity")
tpoly(0, 1, 50, 1, 0)

figure(3)
title("High_end_velocity")
tpoly(0, 1, 50, 0, 1)

%% Oppgave 3.8

figure(1)
title("Initial")
lspb(0, 1, 5)

figure(2)
title("Coast phase")
lspb(0, 1, 50, 0.021)

figure(3)
title("highest Coast")
lspb(0, 1, 50, 0.04)

%% Oppgave 3.9

% s    position
% sd   velocity
% sdd  akseleration
[s, sd, sdd] = tpoly(0, 1, 76);
% her kan sd max være 0.025
max(sd)

figure(1)
title("lspb")
lspb(0, 1, 42, 0.025);
%Som tpoly, men man legger inn farten.

%% Oppgave 3.10

R0 = SO3.Rz(-1) * SO3.Ry(-1);
R1 = SO3.Rz(1) * SO3.Ry(1);
figure(10)

% roll-pitch-yaw
rpy0 = R0.torpy();  rpy1 = R1.torpy();
rpy = mtraj(@tpoly, rpy0, rpy1, 50);
SO3.rpy( rpy ).animate;

% quaternions
q0 = R0.UnitQuaternion;  q1 = R1.UnitQuaternion;
q = interp(q0, q1, 50);
q.animate;

%% Oppgave 3.11

% Repeat the example of Fig. 3.7 for the case where:
%a) the interpolation does not pass through a singularity. Hint – change the start or goal pitch angle. What happens?
%b) the final orientation is at a singularity. What happens

T0 = SE3([0.4, 0.2, 0]) * SE3.rpy(0, 0, 3);
T1 = SE3([-0.4, -0.2, 0.3]) * SE3.rpy(-pi/4, pi/4, -pi/2);
Ts = interp(T0, T1, 50);
figure
P = Ts.transl;
subplot(2,2,1);
plot(P);
rpy = Ts.torpy;
subplot(2,2,3);
plot(rpy);


T0 = SE3([0.4, 0.2, 0]) * SE3.rpy(0, 0, 3);
T1 = SE3([-0.4, -0.2, 0.3]) * SE3.rpy(-pi/4, pi, -pi/2);
Ts = interp(T0, T1, 50);
P = Ts.transl;
subplot(2,2,2);
plot(P);
rpy = Ts.torpy;
subplot(2,2,4);
plot(rpy);



