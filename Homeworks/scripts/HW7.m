%% HW7
% Modelling, Control, Parameter Identification and State Estimation
close all

%% Parameters
m = 0.5;
g = 9.8;
r = 0.2;
l = 0.3;
Tx = 0.5;
Ty = 0.5;

%% Q1.d
tau = 0;
[t,res] = ode45(@(t,x) free_swing(t,x,m,g,r,tau),[0:0.001:5],[0.3*pi;0]);
figure;
plot(t,res(:,1));
title('Angular Position');
figure;
plot(t,res(:,2));
title('Angular Velocity');

%% Q1.e
figure
plot(t,-(g/r).*sin(res(:,1)));
title('Angular Acceleration');

%% Q1.f, Q1.g
x = l*sin(res(:,1))+Tx;
y = -l*cos(res(:,1))+Ty;
figure
plot(x,y)
title('Position of the motion capture marker');

%% Q1.h
% covariance of the Gaussian noise
rho = 0.01;
x = x + rho*rand(size(x,1),size(x,2));
y = y + rho*rand(size(y,1),size(y,2));
figure
plot(x,y)
title('Position of the motion capture marker (with Gaussian noise)');

%% Q1.i

%% Example 1
alph = 10; bet = 2; theta_d = -0.3*pi;
[t,res] = ode45(@(t,x) controller1i(t,x,m,g,r,alph,bet,theta_d),[0:0.001:10],[0.3*pi; 0]);
figure;
plot(t,res(:,1));
title('Angular Position');
figure;
plot(t,res(:,2));
title('Angular Velocity');

%% Example 2
alph = 10; bet = 4; theta_d = -0.3*pi;
[t,res] = ode45(@(t,x) controller1i(t,x,m,g,r,alph,bet,theta_d),[0:0.001:10],[0.3*pi; 0]);
figure;
plot(t,res(:,1));
title('Angular Position');
figure;
plot(t,res(:,2));
title('Angular Velocity');

%% Example 3
alph = 10; bet = 6; theta_d = -0.3*pi;
[t,res] = ode45(@(t,x) controller1i(t,x,m,g,r,alph,bet,theta_d),[0:0.001:10],[0.3*pi; 0]);
figure;
plot(t,res(:,1));
title('Angular Position');
figure;
plot(t,res(:,2));
title('Angular Velocity');

%% Q2.a
% We add the constant term in the controller in order to cancel out the
% torque of gravity acting on the system. In order to make the error of
% angle converges as soon as possible, our system needs to be critically
% damped, this gives us a hint to find appropriate parameters of
% controller.
alph = 10; bet = 6; theta_d = 7/8*pi;
[t,res] = ode45(@(t,x) controller2a(t,x,m,g,r,alph,bet,theta_d),[0:0.001:10],[0.3*pi; 0]);
figure;
plot(t,res(:,1));
title('Angular Position');
figure;
semilogy(t,abs(res(:,1)-theta_d));
title('Angular Position Error');
figure;
plot(t,res(:,2));
title('Angular Velocity');

%% Q3.a
tau = 0;
[t,res] = ode45(@(t,x) free_swing(t,x,m,g,r,tau),[0:0.001:5],[0.3*pi;0]);
x = l*sin(res(:,1))+Tx;
y = -l*cos(res(:,1))+Ty;
rho = 0.01;
x = x + rho*rand(size(x,1),size(x,2));
y = y + rho*rand(size(y,1),size(y,2));

rList = [0.1:0.01:1];
errorList = zeros(length(rList),1);
for i = 1:length(rList)
	r_est = rList(i);
	[t,res_est] = ode45(@(t,x) free_swing(t,x,m,g,r_est,0),[0:0.001:5],[0.3*pi;0]);
	x_est = l*sin(res_est(:,1))+Tx;
	y_est = -l*cos(res_est(:,1))+Ty;
	errorList(i) = norm(x_est - x)^2;
end
figure;
plot(rList,errorList);
title('Error in function of estimated r')

%% Comments of Q3.a
% We can observe that the computed value of r is 0.2m and there is no
% apparent error, since we use sampling comaparison instead of lsqnonlin,
% which might generates local minimum.

%% Functions Used
function dxdt = free_swing(t,x,m,g,r,tau)
dxdt = zeros(2,1);
dxdt(1) = x(2);
dxdt(2) = -(g/r)*sin(x(1)) + tau/m/(r^2);
end

function dxdt = controller1i(t,x,m,g,r,alph,bet,theta_d)
dxdt = zeros(2,1);
tau = -m*(r^2)*(alph*(x(1)-theta_d) + bet*x(2));
dxdt(1) = x(2);
dxdt(2) = -(g/r)*sin(x(1)) + tau/m/(r^2);
end

function dxdt = controller2a(t,x,m,g,r,alph,bet,theta_d)
dxdt = zeros(2,1);
tau = -m*(r^2)*(alph*(x(1)-theta_d) + bet*x(2) - (g/r)*sin(x(1)));
dxdt(1) = x(2);
dxdt(2) = -(g/r)*sin(x(1)) + tau/m/(r^2);
end



