function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient(-f);


%%% All of your code should be between the two lines of stars.
% *******************************************************************
route = start_coords;
current_coords = start_coords;
norm = sqrt(gx.*gx + gy.*gy);
gx = gx./norm;
gy = gy./norm;



for i = 1:max_its
    a = end_coords-current_coords;
    norm_a = sqrt(a(1).*a(1) + a(2).*a(2));
    current_pos(1) = round(current_coords(1));
    current_pos(2) = round(current_coords(2));
    if norm_a >=2       
        next_coords = [current_coords(1)+gx(current_pos(2),current_pos(1)),current_coords(2)+gy(current_pos(2),current_pos(1))];
        route = [route;next_coords];
        next_coords;
        current_coords = next_coords;
    else
        break;
    end
end

% *******************************************************************
end
