m = 2272;   %mass
Iz = 4600;  %rotational inertia
lf = 1.11;  % distance from front axle to the mass center
lr = 1.67;  % distance from rear axle to the mass center
C_a = 1.822e5; % rear cornering stiffness 

%Enviroment parameters
miu = 0.7; %frction cooeficient 0.7 for dry road and 0.4 for wet
g = 9.81; % gravity

Fz = m*g*lr/(lf+lr); % the load that beared by the front tire
threshold = atan(3*miu*Fz/C_a);     % threshold for the tire linear region

SlipAngle = linspace(-threshold, threshold, 57);
SlipAngle = -SlipAngle;
Fyf=zeros(1,57);
for i =1:57
    Fyf(i) = -C_a*tan(SlipAngle(i))+C_a^2/(3*miu*Fz)*abs(tan(SlipAngle(i)))*tan(SlipAngle(i))-C_a^3/(27*miu^2*Fz^2)*tan(SlipAngle(i))^3;
end

% Fyf_end = miu*Fz;
% Fyf = [-Fyf_end Fyf Fyf_end];
% SlipAngle = [0.16 SlipAngle -0.16];