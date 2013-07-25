d = 2.0;
R = 0.3;

r = 2.0;

q = 0.0:0.001:d;

E = zeros(size(q));

for i=1:numel(q)
  p = sqrt(d^2- q(i)*q(i));
  if q(i) < R
    E(i) = p - sqrt(R*R-q(i)*q(i)) - r;
  else
    E(i) = p-r;%sqrt((p-r)*(p-r) + (q(i)-R)*(q(i)-R));
  end
end

plot(q, E);