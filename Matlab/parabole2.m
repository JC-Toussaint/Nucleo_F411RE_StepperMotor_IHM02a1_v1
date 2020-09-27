function [taux, tauy]=parabole2
clc
close all
clear all

%t=linspace(0, 10, 1000);
ustep=1/128;
xtarget=10000*ustep;
ytarget=10000*ustep;
theta=atan2(ytarget, xtarget);

ax=1000; dx=10000; % step/s2
ay=10000; dy=1000;

if (tan(theta)<1.0)
	accX=ax;	
	accY=ay*tan(theta);	
    decX=dx;	
	decY=dy*tan(theta);
else
	accX=ax/tan(theta);
	accY=ay;
    decX=dx/tan(theta);
	decY=dy;
end

taux=sqrt(0.5*(accX+decX)/accX/decX*xtarget)
tauy=sqrt(0.5*(accY+decY)/accY/decY*ytarget)

t=linspace(0, 2, 1000);

x=pos(t, taux, accX, decX);
y=pos(t, tauy, accY, decY);
vx=speed(t, taux, accX, decX);
vy=speed(t, tauy, accY, decY);

subplot(2, 2, 1)
plot(x, y, '-');
p=polyfit(x, y, 1)
p(2)=0;
p(1)=ytarget/xtarget;
fit=polyval(p, x);
grid on
axis equal
hold on
plot(x, fit, 'r')
subplot(2, 2, 4)
plot(t, fit-y, 'b')
grid on

subplot(2, 2, 2)
plot(t, vx, 'r');
hold on
plot(t, vy, 'b');
grid on
subplot(2, 2, 3)
%plot(t, vx, 'r');
plot(y+x, fit-y, 'r')
grid on

figure
plot(y+x, fit-y, 'r')
grid on
end


function x=pos(t, tau, acc, dec)
alpha=2*dec/(acc+dec);
alpha2=alpha*alpha;
x=0.5*acc*(t.^2).*(t<alpha*tau);
x=x+(0.5*acc*alpha2*tau^2+0.5*dec*(alpha-2)^2*tau^2-0.5*dec*(t-2*tau).^2).*(t>=alpha*tau & t<2*tau);
x=x+(0.5*acc*alpha2*tau^2+0.5*dec*(alpha-2)^2*tau^2).*(t>2*tau);
end

function v=speed(t, tau, acc, dec)
alpha=2*dec/(acc+dec);
alpha2=alpha*alpha;
v=acc*t.*(t<alpha*tau);
v=v+dec*(2*tau-t).*(t>=alpha*tau & t<2*tau);
end