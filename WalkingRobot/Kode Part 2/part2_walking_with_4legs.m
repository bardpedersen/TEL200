load house;

%Initiates the prm object and plans the roadmap for house
prm = PRM(house);
prm.plan('npoints', 600,'distthresh',100);

%Points to be addet to make the roadmap better
x_cord_to_add = [150,60,60,80,110,160,174,285,390,170,170,295,310,150,100,120,390,400,150,35,35,50,5,100,200,300,400,500,5,5,5,592,592,592,100,90,51,51,51,51,270,150,506,506,506,5,115,100];
y_cord_to_add = [100,160,140,90,130,40,40,100,230,110,130,40,50,115,150,180,250,290,130,75,90,130,5,5,5,5,5,5,5,100,200,300,100,200,250,265,154,150,146,142,396,396,230,237,241,300,114,114];

%Adding the points and edges with Add_node function
for i = 1:length(x_cord_to_add)
    Add_node(prm,x_cord_to_add(i),y_cord_to_add(i))
end
% selecting start and goal, and making the path with the query function
start = [100, 150]
goal = [250, 300]
path = prm.query(start,goal)

%initiating the Video maker
writerObj = VideoWriter('part2_walking.mp4','MPEG-4');
open(writerObj);
a = Animate('part2_walking_all.mp4','fps',5)

% the loop making the robot walk the path using function from
% walking_rotation_from_point.m had to add a(the animation object) to make
% the robot walk the path
A = [prm.start(1)*100,prm.start(2)*100,0]
way = 'py'
hold on

% goes through the path point to point starts from 3 since 1 and 2 is same
% as startpoint
for k = 3:(length(path))
    
    B = [path(k,1)*100,path(k,2)*100,0]
    walking_rotating_from_point_part2(A,B,way,a)
    prm.plot()
    
    A = B
    
    
end

a.close()
hold off
    
    


   
