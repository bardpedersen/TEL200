function a = part3()    
    pg = PoseGraph('killian.g2o', 'laser');
    a = pg.scanmap();
    pg.plot_occgrid(a);
     
    %Creates empty list for plotting
    time = [];
    walkx = [];
    walky = [];
    speedx = [];
    speedy = [];
    yaw = [];
    yaw_speed = [];
    
    for i=1:3872 %Runs thru all nodes -1, because the icp function checks the one iteration +1
        p1 = pg.scanxy(i); %scans around each node
        p2 = pg.scanxy(i+1);
        T = icp(p1, p2, 'verbose' , 'T0', transl2(0.5, 0), 'distthresh', 3); %Returns a transformation matrix
        time(end+1) = (pg.time((i+1))-pg.time(i)); %Adds the time between nodes to list
        walkx(end+1) = T(1,3); %Adds the distance in x-direction between nodes to list
        walky(end+1) = T(2,3); %Adds the distance in y-direction between nodes to list
        yaw(end+1) = acos(T(1,1)); %Adds the rotation around the z-axis between nodes to list
        speedx(end+1) = abs(T(1,3)) / (pg.time((i+1))-pg.time(i)); %calculates the speed in x-direction
        speedy(end+1) = abs(T(2,3))/ (pg.time((i+1))-pg.time(i)); %calculates the speed in y-direction
        yaw_speed(end+1) = abs(acos(T(1,1))) /(pg.time((i+1))-pg.time(i)); %calculates the speed of rotation

    end
    time = cumsum(time); %add each element before that one element in list we get the time spaced out
                         %Example, list = [1 1 1 2 1] now becomes list = [1 2 3 5 6]
    walkx = cumsum(walkx);
    walky = cumsum(walky);
    yaw = cumsum(yaw);
    
    subplot(2,2,1); %Creates a 2X2 plott and plots in the upper left
    plot(time,walkx) %postion
    hold on 
    plot(time,walky)
    title('Position')
    xlabel('time[s]') 
    ylabel('Position[m]') 
    legend({'x','y'},'Location','southwest')
    hold off
    
    subplot(2,2,3);
    plot(time,speedx) %speed
    hold on 
    plot(time,speedy)
    title('Linear velocity')
    xlabel('time[s]') 
    ylabel('speed[m/s]') 
    legend({'V_x','V_y'},'Location','southwest')
    hold off 
    
    subplot(2,2,2);
    plot(time,yaw) %rotation
    title('Rotation')
    xlabel('time[s]') 
    ylabel('orientation[rad]') 
    legend({'yaw'},'Location','southwest')
    
    subplot(2,2,4);
    plot(time,yaw_speed) %rotation speed
    title('Rotation velocity')
    xlabel('time[s]') 
    ylabel('roation speed[rad/s]') 
    legend({'W_yaw'},'Location','southwest')
end
