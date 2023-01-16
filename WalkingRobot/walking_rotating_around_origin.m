% set the dimensions of the two leg links


% Copyright (C) 1993-2017, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function walking_rotating_around_origin(A, B, varargin)

    opt.niterations = 500;
    opt.movie = "walking.mp4";
    
    opt = tb_optparse(opt, varargin);
    
L1 = 0.1; L2 = 0.1;

fprintf('create leg model\n');

% create the leg links based on DH parameters
%                    theta   d     a  alpha  
links(1) = Link([    0       0    0   pi/2 ], 'standard');
links(2) = Link([    0       0    L1   0   ], 'standard');
links(3) = Link([    0       0   -L2   0   ], 'standard');

% now create a robot to represent a single leg
leg = SerialLink(links, 'name', 'leg', 'offset', [pi/2   0  -pi/2]);

% define the key parameters of the gait trajectory, walking in the
% x-direction
xf = 5; xb = -xf;   % forward and backward limits for foot on ground
y = 5;              % distance of foot from body along y-axis
zu = 2; zd = 5;     % height of foot when up and down
% define the rectangular path taken by the foot
segments = [xf y zd; xb y zd; xb y zu; xf y zu] * 0.01;

% build the gait. the points are:
%   1 start of walking stroke
%   2 end of walking stroke
%   3 end of foot raise
%   4 foot raised and forward
%
% The segments times are :
%   1->2  3s
%   2->3  0.5s
%   3->4  1s
%   4->1  0.5ss
%
% A total of 4s, of which 3s is walking and 1s is reset.  At 0.01s sample
% time this is exactly 400 steps long.
%
% We use a finite acceleration time to get a nice smooth path, which means
% that the foot never actually goes through any of these points.  This
% makes setting the initial robot pose and velocity difficult.
%
% Intead we create a longer cyclic path: 1, 2, 3, 4, 1, 2, 3, 4. The
% first 1->2 segment includes the initial ramp up, and the final 3->4
% has the slow down.  However the middle 2->3->4->1 is smooth cyclic
% motion so we "cut it out" and use it.
fprintf('create trajectory\n');

segments = [segments; segments];
tseg = [3 0.25 0.5 0.25]';
tseg = [tseg; tseg];
x = mstraj(segments, [], tseg, segments(1,:), 0.01, 0.1);

% pull out the cycle
fprintf('inverse kinematics (this will take a while)...');
xcycle = x(100:500,:);
qcycle = leg.ikine(transl(xcycle), 'mask', [1 1 1 0 0 0] );

% dimensions of the robot's rectangular body, width and height, the legs
% are at each corner.
W = 0.1; L = 0.2;

% a bit of optimization.  We use a lot of plotting options to 
% make the animation fast: turn off annotations like wrist axes, ground
% shadow, joint axes, no smooth shading.  Rather than parse the switches 
% each cycle we pre-digest them here into a plotopt struct.
% plotopt = leg.plot({'noraise', 'nobase', 'noshadow', ...
%     'nowrist', 'nojaxes'});
% plotopt = leg.plot({'noraise', 'norender', 'nobase', 'noshadow', ...
%     'nowrist', 'nojaxes', 'ortho'});

fprintf('\nanimate\n');

plotopt = {'noraise', 'nobase', 'noshadow', 'nowrist', 'nojaxes', 'delay', 0};

    
% walk!
k = 1;
Ani = Animate(opt.movie);

%A=[Startx for legg 1, starty for legg 1, rotation on the robot]
%B=[Forward, sideways not used, rotation]

walkstartx = A(1)/100;
walkstarty = A(2)/100;
rotatestart = A(3) * pi/180;

walkto = B(1)/100 - walkstartx;
rotateto = B(3) * (pi/180);

walk = 0;
Rotate = 0;

%{
Legs = [0, 0, 0;
        -L, 0, 0;
        -L, -W, pi;
        0, -W, pi];
%}
Legs = [walkstartx, walkstarty, rotatestart;
        walkstartx-L*cos(rotatestart), walkstarty-L*sin(rotatestart), rotatestart;
       walkstartx-L*cos(rotatestart)+W*sin(rotatestart), walkstarty-W*cos(rotatestart)-L*sin(rotatestart), rotatestart+pi;
       walkstartx+W*sin(rotatestart), walkstarty-W*cos(rotatestart), rotatestart+pi];
   
for i=1:opt.niterations * 2
    % create 4 leg robots.  Each is a clone of the leg robot we built above,
    % has a unique name, and a base transform to represent it's position
    % on the body of the walking robot.
    
    %legs = [lx1*cos(rot1), ly1*sin(rotz1), rot1;
    %     lx2*cos(rot2), ly2*sin(rotz2), rot2;
    %     lx3*cos(rot3), ly3*sin(rotz3), rot3;
    %     lx4*cos(rot4), ly4*sin(rotz4), rot4]

    if ~(abs(walk - walkto) < 0.000001)
        walking = walkto/(opt.niterations);
        walk = walk + walking;
        %Walk matrix
        Legs = Legs + [walking*cos(rotatestart) walking*sin(rotatestart) 0;
                       walking*cos(rotatestart) walking*sin(rotatestart) 0;
                       walking*cos(rotatestart) walking*sin(rotatestart) 0;
                       walking*cos(rotatestart) walking*sin(rotatestart) 0];
    
    elseif ~(abs(Rotate - rotateto) < 0.000001)
        Rotate = Rotate+rotateto/opt.niterations;
       
        %Rotating matrix
        Legs =  [Legs(1,1) Legs(1,1) Rotate;
                -L*cos(Rotate)+Legs(1,1) -L*sin(Rotate)+Legs(1,2) Rotate;
                -L*cos(Rotate)+W*sin(Rotate)+Legs(1,1) -L*sin(Rotate)+-W*cos(Rotate)+Legs(1,2) Rotate+pi;
                 W*sin(Rotate)+Legs(1,1) -W*cos(Rotate)+Legs(1,2) Rotate+pi];
        
    end 
    
 
    legs(1) = SerialLink(leg, 'name', 'leg1', 'base', transl(Legs(1,1), Legs(1,2), 0)*trotz(Legs(1,3)));
    legs(2) = SerialLink(leg, 'name', 'leg2', 'base', transl(Legs(2,1), Legs(2,2), 0)*trotz(Legs(2,3)));
    legs(3) = SerialLink(leg, 'name', 'leg3', 'base', transl(Legs(3,1), Legs(3,2), 0)*trotz(Legs(3,3)));
    legs(4) = SerialLink(leg, 'name', 'leg4', 'base', transl(Legs(4,1), Legs(4,2), 0)*trotz(Legs(4,3)));
    
    % create a fixed size axis for the robot, and set z positive downward
    clf; axis([-0.6+Legs(1,1) 0.2+Legs(1,1) -0.6+Legs(1,2) 0.4+Legs(1,2) -0.3 0.10]); set(gca,'Zdir', 'reverse')
    hold on
    
    % draw the robot's body %
    %corner1, corner2, corner3, corner4
    %x
    %Y
    %Z
    patch([Legs(1,1) Legs(2,1) Legs(3,1) Legs(4,1)],...
          [Legs(1,2) Legs(2,2) Legs(3,2) Legs(4,2)], ...
          [0 0 0 0], ...
          'FaceColor', 'r', 'FaceAlpha', 0.5)
    
    % instantiate each robot in the axes
    for j=1:4
        legs(j).plot(qcycle(1,:), plotopt{:});
    end
    
    legs(1).animate(gait(qcycle, k, 0,   0));
    legs(2).animate(gait(qcycle, k, 100, 0));
    legs(3).animate(gait(qcycle, k, 200, 1));
    legs(4).animate(gait(qcycle, k, 300, 1));
    drawnow
    k = k+1; 
    
    Legs
    
    Ani.add();     
        
end

end
