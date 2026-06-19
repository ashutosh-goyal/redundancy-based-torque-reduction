function [Data] = forward_kinematics_kuka(q)
syms theta alph d a;
q=q';
a1 = 0.36;
a2 = 0.42;
a3 = 0.4;
a4 = 0.126;
M = [cos(theta), -sin(theta)*cos(alph), sin(theta)*sin(alph), a*cos(theta);sin(theta),cos(theta)*cos(alph),-cos(theta)*sin(alph), a*sin(theta); 0,sin(alph),cos(alph),d; 0, 0, 0, 1];
A1 = subs(M,{a,alph,d,theta},{0,-pi/2,a1,q(1,:)});
A2 = subs(M,{a,alph,d,theta},{0,pi/2,0,q(2,:)});
A3 = subs(M,{a,alph,d,theta},{0,pi/2,a2,q(3,:)});
A4 = subs(M,{a,alph,d,theta},{0,-pi/2,0,q(4,:)});
A5 = subs(M,{a,alph,d,theta},{0,-pi/2,a3,q(5,:)});
A6 = subs(M,{a,alph,d,theta},{0,pi/2,0,q(6,:)});
A7 = subs(M,{a,alph,d,theta},{0,0,a4,q(7,:)});
T01 = A1;
T02 = T01*A2;
T03 = T02*A3;
T04 = T03*A4;
T05 = T04*A5;
T06 = T05*A6;
T07 = T06*A7;
R = [T07(1,1),T07(1,2),T07(1,3);T07(2,1),T07(2,2),T07(2,3);T07(3,1),T07(3,2),T07(3,3)];
x = T07(1,4);
y = T07(2,4);
z = T07(3,4);
angle_y = pi/2;
angle_x = 0;
angle_z = 0;
Data = [x,y,z,angle_x,angle_y,angle_z];

% angle_y = asin(-R(3,1));
% if cos(angle_y)<=3.672*(10)^(-6) && cos(angle_y)>=-3.672*10^(-6)
%     angle_x = 0;
%     angle_z = 0;
%     
% else
%    
%     angle_x = asin(R(3,2)/cos(angle_y));
%     angle_z = acos(R(1,1)/cos(angle_y));
% end
% 
% Data=[x,y,z,angle_x,angle_y,angle_z];
end