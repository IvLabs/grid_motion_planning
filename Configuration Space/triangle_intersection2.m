function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise


flag=true;
for i=0:2
    x1=P1(rem(i,3)+1,1);
    x2=P1(rem(i+1,3)+1,1);
    y1=P1(rem(i,3)+1,2);
    y2=P1(rem(i+1,3)+1,2);
    coefficients = polyfit([x1,x2],[y1,y2],1);
    a = coefficients (1);
    b = coefficients (2);   
    eq1= P2(1,2)-(a*P2(1,1))-b;
    eq2= P2(2,2)-(a*P2(2,1))-b;
    eq3= P2(3,2)-(a*P2(3,1))-b;
    eq4= P1(rem(i+2,3)+1,2)-(a*P1(rem(i+2,3)+1,1))-b;
    if ((eq1>0 && eq2>0 && eq3>0 && eq4<0)||(eq1<0 && eq2<0 && eq3<0 && eq4>0))
        flag=false;
        break
        
    end
end
if flag == true
    for i=0:2
          x1=P2(rem(i,3)+1,1);
          x2=P2(rem(i+1,3)+1,1);
          y1=P2(rem(i,3)+1,2);
          y2=P2(rem(i+1,3)+1,2);
          coefficients = polyfit([x1,x2],[y1,y2],1);
          a = coefficients (1);
          b = coefficients (2);   
          eq1= P1(1,2)-(a*P1(1,1))-b;
          eq2= P1(2,2)-(a*P1(2,1))-b;
          eq3= P1(3,2)-(a*P1(3,1))-b;
          eq4= P2(rem(i+2,3)+1,2)-(a*P2(rem(i+2,3)+1,1))-b;
         if ((eq1>0 && eq2>0 && eq3>0 && eq4<0)||(eq1<0 && eq2<0 && eq3<0 && eq4>0))
             flag=false;
             break
         end
    end
end
end

        
    
   
    
    
