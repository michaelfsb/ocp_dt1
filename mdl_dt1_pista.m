function [mdl_out] = mdl_dt1_pista(x, v, i, phi) 

Kt = 0.119;     
rr = 0.254;     
ro = 1.22;      
Af = 0.26;      
Cd = 0.164;     
G  = 9.81;      
mi = 0.0024;    
Jr = 0.015;      
Jm = 0.0625e-3; 
mv = 36;        
mp = 50;        
ef = 0.95;      

theta = atan(0.012293*cos(0.004363*x-0.228758)+...
    0.003041*cos(0.000236*x+0.399758)+...
    0.005818*cos(0.008726*x-2.193381)+...
    0.002926*cos(0.000246*x+3.490690)+...
    0.003958*cos(0.017449*x-3.058604)+...
    0.005365*cos(0.026184*x+0.579933)+...
    0.004361*cos(0.021814*x+1.627015)+...
    0.005539*cos(0.030552*x-1.400496));
                 
mr = 3*(Jr/rr^2) + Jm*phi^2/rr^2;
M = mv + mp + mr;

Ft = ((Kt*i)*ef*phi)/rr;           
Fa = (ro*Af*Cd*v^2)/2;             
Fr = (mv + mp)*G*mi*cos(theta);    
Fg = (mv + mp)*G*sin(theta);       

x_dot = v;
v_dot = (Ft-Fa-Fr-Fg)/M;

mdl_out = [x_dot; v_dot; Ft; Fa; Fr; Fg; theta];
end