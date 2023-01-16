%function to add node to roadmap and connect with edge code copyied from
%PRM.m
function Add_node(prm,x,y)
    new = [x; y];
    %adds the points/nodes to the map
    vnew = prm.graph.add_node(new); 
    [d,v] = prm.graph.distances(new);
    %function from the PRM.m file that checks the path between points 
    function c = testpath(prm, p1, p2)
        p = bresenham(p1, p2);

        for pp=p'
            if prm.isoccupied(pp)
                c = false;
                return;
            end
        end
        c = true;
    end

    % Function that adds the edges between the new points and other points
    % on the map. Code copied from PRM.m
    for i=1:length(d)
        if d(i) > prm.distthresh
            continue; % it's too far
        end
        if ~testpath(prm,new, prm.graph.coord(v(i)))
            continue; % no path
        end
                    
        % add an edge from the found node to new
        prm.graph.add_edge(v(i), vnew);
    end
    
end
  
   
