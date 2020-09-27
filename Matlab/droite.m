function straight1
clc
close all
clear all

t=linspace(0, 3, 1000);
taux=1.05;
tauy=1.0;

xtarget= 10000;
ytarget=10000;

x=xtarget*fun(t/taux);
y=ytarget*fun(t/tauy);

subplot(1, 2, 1)
plot(x, y, '-');
p=polyfit(x, y, 1)
p(2)=0;
p(1)=ytarget/xtarget;
fit=polyval(p, x);
grid on
axis equal
hold on
plot(x, fit, 'r')
subplot(1, 2, 2)
plot(t, fit-y, 'b')
grid on
end


function x=fun(u)

x=0.5*(u.^2).*(u<1)+(1-0.5*(2-u).^2).*(u>=1 & u<2) + (u>=2);

end
