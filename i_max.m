function [Iup] = i_max(v, i, phi)
Kv = 0.119;     
Ra = 0.07;      
Vbat = 42;     
rr = 0.254;     

I_MAX = (Vbat-Kv*(v*phi/rr))/Ra;

Iup = i - I_MAX;
end