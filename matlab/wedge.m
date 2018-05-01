function wedged = wedge(xi)
% Computes the wedge of the thing
v = xi(1:3);
w = xi(4:6);

wedged = [skew(w), v;
          [0 0 0 0]];
end

