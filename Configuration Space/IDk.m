% this define the vertexes (x,y) of first triangle
trian1x =[ 0 0.5 1 0]; 
trian1y =[ 0 1 0 0]; 
% this define the vertexes (x,y) of second triangle  
trian2x =[ 0 0.25 0.5 0]; 
trian2y =[ 0 0.5 0 0]; 
% (xi,yi) are the intersection points  
[xi,yi] = polyxpoly(trian1x,trian1y,trian2x,trian2y,'unique');
figure 
plot(trian1x,trian1y,'b')
hold on 
plot(trian2x,trian2y,'r')
hold on 
plot(xi,yi,'*g')