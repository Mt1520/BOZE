function [tot_fus_S,S_fusf,A4]  = fuselageAreaCalc(l)

a1 = 1.2/2;
b1 = 0.6/2;
h1 = 1.2;

a2 = 1.2;
b2 = 0.51/2;
c2 = 1.5;

A1 = pi*(a1 + b1)*h1;
A2 = pi * ( ((a2*b2)^1.6 + (a2*c2)^1.6 + (b2*c2)^1.6) / 3)^1.6;
S_fusf = A1+A2;


%A3 = ( (pi*q*sin(alpha))/(cos(alpha)^2) )*( 1/(1 - tan(alpha)^2 * tan(beta)^2)^(3/2) );

%% back of fuselage calc
thetas = 0:0.1:pi;
a = a1;
b = b1;
x = a*sin(thetas);
y = b/2*(cos(thetas)+1);
% plot(x,y)
A4 = [];

for i = 1:length(l)
    h = sqrt(l(i).^2 + x.^2 + y.^2);
    ds_dtheta = sqrt((a*cos(thetas)).^2 + (-b/2 * sin(thetas)).^2);
    A4(i) = trapz(thetas, h/2 .* ds_dtheta);
end

tot_fus_S = A4+S_fusf;

end

