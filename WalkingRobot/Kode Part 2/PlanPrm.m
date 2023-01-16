%function to plan prm for robot
function PlanPrm(prm,house, x_start,y_start,x_stop,y_stop)
    
    start_val = house(x_start, y_start);
    stop_val = house(x_stop, y_stop);
    %checks if start and stop points inside wall
    if start_val ~= 0
        print('The start point is inside a wall')
    
    elseif stop_val ~= 0
        print('The stop point is inside a wall')
    else
        start = [x_start,y_start];
        stop = [x_stop, y_stop];
        %creates the path
        prm.query(start,stop)
         
    end 
    
end
