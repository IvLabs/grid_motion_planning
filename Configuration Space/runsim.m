x1 = [1,2,3]; 
y1 = [4,3,4];
P1 = [x1;y1]';

x2 = [1,4,2];
y2 = [5,4,6];
P2 = [x2;y2]';

line([P1(:,1)' P1(1,1)],[P1(:,2)' P1(1,2)],'Color','r')
line([P2(:,1)' P2(1,1)],[P2(:,2)' P2(1,2)],'Color','b')

flag = triangle_intersection(P1,P2);
if flag == 1
    disp("Intersection")
else
    disp("No Intersection")
end
