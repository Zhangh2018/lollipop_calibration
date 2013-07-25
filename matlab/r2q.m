function [q] = r2q(r)

[m,n] = size(r);
assert(m==3 && n==3);

tr = trace(r);

if tr>0
  l = sqrt(1+tr);
  s = 0.5/l;
  w = 0.5*l;
  x = (r(3,2) - r(2,3))*s;
  y = (r(1,3) - r(3,1))*s;
  z = (r(2,1) - r(1,2))*s;
else
  l = sqrt(1+r(1,1)-r(2,2)-r(3,3));
  s = 0.5/l;
  w = (r(3,2)-r(2,3))*s;
  x = 0.5*l;
  y = (r(1,2)+r(2,1))*s;
  z = (r(3,1)+r(1,3))*s;
end

if w>0
  q = [w x y z];
else
  q = [-w -x -y -z];
end