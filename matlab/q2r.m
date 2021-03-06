function [R] = q2r(q)

t2 = q(1)*q(2);
t3 = q(1)*q(3);
t4 = q(1)*q(4);
t5 =-q(2)*q(2);
t6 = q(2)*q(3);
t7 = q(2)*q(4);
t8 =-q(3)*q(3);
t9 = q(3)*q(4);
t1 =-q(4)*q(4);

R = [1+2*(t8+t1),  2*(t6-t4),  2*(t3+t7);...
       2*(t4+t6),1+2*(t5+t1),  2*(t9-t2);...
       2*(t7-t3),  2*(t2+t9),1+2*(t5+t8)];
end