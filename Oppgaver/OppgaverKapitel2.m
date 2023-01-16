%% Oppgave 2.1

T = eye(3)
figure
trplot(T, 'frame', 'A')

%% Oppgave 2.5

% Lager 3d rotasjonsmatrisen R og bruker tranimate for å displaye
% rotx er rotajson om x aksen og pi/3 er hvor mye
R = rotx(pi/3)*roty(pi/2)*rotz(pi/6)
figure
tranimate(R)

% Lager og plotter vektor V
v = [2,0,0];
plot3([0 v(1)],[0 v(2)],[0 v(3)],'r')
axis([-0.5 2.5 -0.5 2.5 -0.5 2.5])
set(findall(gca, 'Type', 'Line'),'LineWidth',3);

% Transformerer vektor V med R
vt = R.*v
figure
plot3([0 vt(1)],[0 vt(2)],[0 vt(3)],'r')
axis([-0.5 2.5 -0.5 2.5 -0.5 2.5])
set(findall(gca, 'Type', 'Line'),'LineWidth',3);

% invers av R
Ri = inv(R)
vn = Ri*vt
figure
plot3([0 vn(1)],[0 vn(2)],[0 vn(3)],'r') 
axis([-0.5 2.5 -0.5 2.5 -0.5 2.5])
set(findall(gca, 'Type', 'Line'),'LineWidth',3);

% Invers og determinant
Ri*R
R*Ri

%% Oppgave 2.7

R1=eye(3); 
R2=eye(3); 
% Lager R2 til en identitetsmatrise på størelse 3. (x, y, z)
figure
subplot(2,3,1); 
%Lager et subplot med 2 * 3 plots, og legger den koden som kommer under i
%første plottet. 
trplot(R1,'view',[15 15],'noaxes','axis',[0 1 0 1 0 1]) 
% Plotter med R1 matrisen, 
% 'view' setter synsvinkelen så man ser alle 3-aksene. og [az el] az = er vinkelen av objevtet om horisonten el = er vinkelen mellom objectet og seers lokale horisont
% 'noaxes' fjerner rutenettet, så man kun ser vektroene.
% 'axis',setter hvor store aksene skal være med [xmin xmax ymin ymax zmin zmax]
subplot(2,3,4); 
trplot(R2,'view',[15 15],'noaxes','axis',[0 1 0 1 0 1])

subplot(2,3,2);
R1=R1*rotx(pi/2);
% Roterer R1 matrisen 90 grader om x aksen
trplot(R1,'view',[15 15],'noaxes','axis',[0 1 -1 0 0 1])
subplot(2,3,5);
R2=R2*roty(pi/2);
trplot(R2,'view',[15 15],'noaxes','axis',[0 1 0 1 -1 0])

subplot(2,3,3);
R1=R1*roty(pi/2);
trplot(R1,'view',[15 15],'noaxes','axis',[0 1 0 1 0 1])
subplot(2,3,6);
R2=R2*rotx(pi/2);
trplot(R2,'view',[15 15],'noaxes','axis',[0 1 -1 0 -1 0])

%Fra plottene er det tydelig at operasjonen er ikke-kommutativ, og
%rekkef lgen spiller en ro

%% Oppgave 2.15

T =[0 2 0; 
    3 1 2;
    5 4 2]

Tinv = inv(T)
T*Tinv

%% Oppgave 2.20
    
R = rotx(-pi/2)*roty(0)*rotz(0)

eul = [-pi/2 0 0];
rotmZYX = eul2rotm(eul)


