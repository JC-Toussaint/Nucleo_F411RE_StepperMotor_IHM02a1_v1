function [t, x, y, taux, tauy]=parabole3
clc
close all
clear all

%t=linspace(0, 10, 1000);
ustep=1/128;
xtarget=10000*ustep;
ytarget=10000*ustep;
theta=atan2(ytarget, xtarget);

ax= 1000; dx= 7000; % step/s2
ay= 7000; dy= 1000;

ax=1000; ay=7000;
dx=xtarget/ytarget*ay-ax
idy=(1/dx+1/ax)*xtarget/ytarget-1/ay;
dy=1/idy

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

t=linspace(0, 1, 1000);

[x, alphax]=pos(t, taux, accX, decX);
[y, alphay]=pos(t, tauy, accY, decY);
alphax
alphay
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
subplot(1, 2, 1)
plot(y+x, fit-y, 'r')
grid on
subplot(1, 2, 2)
plot(t, x, 'r')
hold on
plot(t, y, 'b')
grid on

xend=max(x);
x=[x 2*xend-x x 2*xend-x]
y=[y y -y -y]
figure
subplot(2, 2, 1)
circle(xend, 0, xend, [0 0 0])
hold on
plot(x, y, 'r')
grid on
axis equal
subplot(2, 2, 2)
circle(xend, 0, xend, [0 0 0])
hold on
plot(x, sqrt((x-xend).^2+y.^2), 'r')
axis equal
grid on
subplot(2, 2, 3)
plot(x, y, 'r')
grid on
axis equal
end


function [x, alpha]=pos(t, tau, acc, dec)
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