function [Cost] = lagrange_cost(v, i, phi)
Kv = 0.119;     
Ra = 0.07;      
rr = 0.254;     

u = i*Ra + Kv*(v*phi/rr);

Cost = u*i;
end
