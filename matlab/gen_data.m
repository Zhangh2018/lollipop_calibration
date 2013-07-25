function [] = gen_data()

c0 = [0 0 0 1 0 0 0];
c1 = [0 0 3 cos(pi/2) 0 sin(pi/2) 0]; % offset by 1-x and look backwards

b0 = [ 1.0 0.0  2.0]';
b1 = [-1.0 0.0  1.0]';

nsr = 0.00;
n   = 50;
r   = 0.1275;

p00 = gen_points(c0, b0, n, nsr, r);
p01 = gen_points(c0, b1, n, nsr, r);

p10 = gen_points(c1, b0, n, nsr, r);
p11 = gen_points(c1, b1, n, nsr, r);

fd = fopen('out.txt', 'w');

fprintf(fd, '2 2 %d\n', n);
fprintf(fd, '%f %f %f %f %f %f %f\n', c0(1), c0(2), c0(3), c0(4), c0(5), c0(6), c0(7));
fprintf(fd, '%f %f %f %f %f %f %f\n', c1(1), c1(2), c1(3), c1(4), c1(5), c1(6), c1(7));
fprintf(fd, '%f %f %f\n', b0(1), b0(2), b0(3));
fprintf(fd, '%f %f %f\n', b1(1), b1(2), b1(3));
for i=1:n
  fprintf(fd, '%f %f %f\n', p00(1,i), p00(2,i), p00(3,i));
end
for i=1:n
  fprintf(fd, '%f %f %f\n', p01(1,i), p01(2,i), p01(3,i));
end
for i=1:n
  fprintf(fd, '%f %f %f\n', p10(1,i), p10(2,i), p10(3,i));
end
for i=1:n
  fprintf(fd, '%f %f %f\n', p11(1,i), p11(2,i), p11(3,i));
end
fclose(fd);
%plot3(p00(1,:), p00(2,:), p00(3,:), 'b.', p01(1,:), p01(2,:), p01(3,:), 'r.');
%axis equal
end

function [P] = gen_points(c, b, n, nsr, r)


% transform center in world frame to camera frame
R   = q2r([c(4) -c(5:7)]);
b_c = R*(b- c(1:3)');

% 
d = norm(b_c);

c_rd = acos(r/d);

theta = 2*pi*rand([1,n]);  % 0 - 2pi
phi   = (pi-c_rd) + c_rd*rand([1,n]);
noise = nsr * randn([1,n]);

z = (d + r*cos(phi)).*(1+noise);
x = r*sin(phi).*sin(theta).*(1+noise);
y = r*sin(phi).*cos(theta).*(1+noise);

% rotate the points to the correct direction
zp = b_c/d;
yp = cross([0 0 1], zp)'; yp = yp/norm(yp);
xp = cross(yp, zp);
R  = [xp, yp, zp];

P = R*[x; y; z];
end