load house;
prm = PRM(house);
prm.plan('npoints', 500,'distthresh',100);
x_cord_to_add = [150,60,60,80,110,160,174,285,390,170,170,295,310,150,100,120,390,400,150,35,35,50,5,100,200,300,400,500,5,5,5,592,592,592,100,90,51,51,51,51,270,150,506,506,506,5,115,100];
y_cord_to_add = [100,160,140,90,130,40,40,100,230,110,130,40,50,115,150,180,250,290,130,75,90,130,5,5,5,5,5,5,5,100,200,300,100,200,250,265,154,150,146,142,396,396,230,237,241,300,114,114];
for i = 1:length(x_cord_to_add)
    Add_node(prm,x_cord_to_add(i),y_cord_to_add(i))
end

%initiates the code for making mp4 file
writerObj = VideoWriter('part2.mp4','MPEG-4');
open(writerObj);

% the loop running 10 simulations from a to b 
k = 0
a = Animate('part2.mp4','fps',5)
%runs the loop 10 times and creates 10 paths
while k ~= 10
    %tries to create the path for the points
    try
        % creates the start and stop points 
        x_start = randi([1,596])
        x_stop = randi([1,596])
        y_start = randi([1,397])
        y_stop = randi([1,397])
        PlanPrm(prm, house, x_start, y_start, x_stop, y_stop)
        prm.plot()
        k = k + 1
        a.add()   
    % catches the error when points are inside the wall    
    catch
        display('point inside wall')
    end
    
end 
% closes the animate object 
a.close()
