clc
clear all
close all

s = serial('COM37','BaudRate', 115200, 'Timeout', 5)

s.UserData = struct('t',[],'x',[], 'y', [], 'vx', [], 'vy', [])
fopen(s)

disp('*** RESET STM32 BOARD ***')
while true
    str = fscanf(s);
    
    try
        if str(1)=='%'
            continue
        end
    catch me
        break;
    end
    data = strsplit(str, ' ');
    
    ustep=1/128;
    s.UserData.t (end+1) = str2double(data(1));
    s.UserData.x (end+1) = str2double(data(2))*ustep;
    s.UserData.y (end+1) = str2double(data(3))*ustep;
    s.UserData.vx(end+1) = str2double(data(4));
    s.UserData.vy(end+1) = str2double(data(5));
    %     subplot(2, 2, 1)
    %     plot(s.UserData.x(2:end), s.UserData.y(2:end), 'k');
    %     axis equal
    %     grid on
    %     subplot(2, 2, 2)
    %     plot(s.UserData.t(2:end), s.UserData.vx(2:end), 'r');
    %     hold on
    %     plot(s.UserData.t(2:end), s.UserData.vy(2:end),  'b');
    %     grid on
    %     subplot(2, 2, 3)
    %     plot(s.UserData.t(2:end), s.UserData.vy(2:end), 'k');
    %
    %
    %     drawnow()
    
end

fclose(s)
p=polyfit(s.UserData.x(2:end), s.UserData.y(2:end), 1)
p(2)=0;
p(1)=s.UserData.y(end)/(s.UserData.x(end)+1e-6);
fit=polyval(p, s.UserData.x(2:end));
subplot(2, 2, 1)
plot(s.UserData.x(2:end), s.UserData.y(2:end), 'k');
axis equal
grid on
hold on
plot(s.UserData.x(2:end), fit, 'r')
grid on
subplot(2, 2, 4)
tx=s.UserData.x(end);  ty=s.UserData.y(end);
tnorm=sqrt(tx^2+ty^2);
tx=tx/tnorm; ty=ty/tnorm;
HMx=s.UserData.x(2:end)-(tx*s.UserData.x(2:end)+ty*s.UserData.y(2:end))*tx;
HMy=s.UserData.y(2:end)-(tx*s.UserData.x(2:end)+ty*s.UserData.y(2:end))*ty;
plot(s.UserData.t(2:end), -HMx*ty+HMy*tx, 'b')
grid on
subplot(2, 2, 2)
plot(s.UserData.t(2:end), s.UserData.vx(2:end), 'r');
hold on
plot(s.UserData.t(2:end), s.UserData.vy(2:end), 'b');
grid on
subplot(2, 2, 3)
%plot(s.UserData.t(2:end), s.UserData.vy(2:end), 'k');
plot(tx*s.UserData.x(2:end)+ty*s.UserData.y(2:end), -HMx*ty+HMy*tx, 'b')

figure
plot(s.UserData.x(2:end), s.UserData.y(2:end), 'k.');
hold on
plot(s.UserData.x(2:end), fit, 'r')
axis equal
grid on

