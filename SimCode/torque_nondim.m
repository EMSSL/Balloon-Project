clc
clear all
m=6.6;
L=0.5*1.225*5.4*5.4*1*1.6*0.3  % Lift 
FG= m*9.81 % force due to gravity 
theta=asind(L/FG) % sail angle 
TG=FG*0.8*sind(theta)  % torque due to gravity 
cg=TG/13.7169           