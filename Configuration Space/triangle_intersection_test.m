function flag = triangle_intersection_test(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************

flag = Checkpoint(P1, P2);
if flag == false
    flag=Checkpoint(P2,P1);
end

    function Point = Checkpoint(Po1, Po2)
        Point = false;
        A = polyxpoly(Po1(:,1), Po1(:,2), Po2(:,1), Po2(:,2),'unique');
        signal = isempty(A);
        if signal == 0
            Point = true;
            return 
        end
    end
end
            
        
        
        
