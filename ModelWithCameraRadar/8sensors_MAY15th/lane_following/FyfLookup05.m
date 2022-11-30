m = 2272;   %mass
Iz = 4600;  %rotational inertia
lf = 1.11;  % distance from front axle to the mass center
lr = 1.67;  % distance from rear axle to the mass center
Cf = 1.3e5; % rear cornering stiffness 

%Enviroment parameters
miu = 0.7; %frction cooeficient 0.7 for dry road and 0.4 for wet
g = 9.81; % gravity

Fz = m*g*lr/(lf+lr)/2; % the load that beared by the front tire
threshold = atan(3*miu*Fz/Cf);     % threshold for the tire linear region

SlipAngle1 = linspace(-threshold, threshold, 57);
SlipAngle1 = -SlipAngle1;
Fyf1=zeros(1,57);
for i =1:57
    Fyf1(i) = -Cf*tan(SlipAngle1(i))+Cf^2/(3*miu*Fz)*abs(tan(SlipAngle1(i)))*tan(SlipAngle1(i))-Cf^3/(27*miu^2*Fz^2)*tan(SlipAngle1(i))^3;
end