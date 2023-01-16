%% Oppgave 7.1

%1. Forward kinematics for planar robot from Sect. 7.1.1.
%a) For the 2-joint robot use the teach method to determine the two sets of joint
%angles that will position the end-effector at (0.5, 0.5).
%b) Experiment with the three different models in Fig. 7.2 using the fkine and
%teach methods.
%c) Vary the models: adjust the link lengths, create links with a translation in the
%y-direction, or create links with a translation in the x- and y-direction.


import ETS3.*
a1 = 1; a2 = 2;
E = Rz('q1') * Tx(a1);
E.structure
%E.plot( [-20, 140], 'deg')

%% Oppgave 7.2
%mdl_puma560
%p560.teach()

%% Oppgave 7.3
%3. Inverse kinematics for the 2-link robot on page 206.
%a) Compute forward and inverse kinematics with a1 and a2 as symbolic rather than numeric values.
%b) What happens to the solution when a point is out of reach?
%c) Most end-effector positions can be reached by two different sets of joint angles.
%What points can be reached by only one set?

syms a1 a2 real
E = Rz('q1') * Tx(a1) * Rz('q2') * Tx(a2);
syms q1 q2 real


TE = E.fkine([q1 q2])
TI = E.ikine(TE, 'rdn')











