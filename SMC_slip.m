%%Robust Control of WineRobot 1 - Sliding mode Control

syms th
a1 = 0; %the wheel-soil interaction entry angle,
a2 = 45;%the departure angle
shs = 1;%shear stress
ns = 1;%normal stress
h = 1;
d = 1; %screw pitch
L = 10; %length of the winch roller

%reference value
sr = 0.2; %desired slip ratio
vr = 20; %desired velocity
xr = 1:100;%x coordinate
yr = 2;%y coordinate
sir = 10;%orientaion (yaw angle)
st = 0;%steering angle

sl = 45; %slope in degrees
m = 300; %Mass of the robot in Kg
g = 9.8; %acceleration due to gravity
I = 0.1; %Inertia
sd = 10; %sinkage
r = 300; %radius of wheel in mm
b = 800; %width of wheel in mm
rs = 149;
Tr = 1;%resistance moment
s = [0 0]';
v = [1 1]';
x = [0 0]';
%calculate current states
t = 0;%time
dt = 1; %sampling time
ig = 1/4;%gear ratio
id = 0.14; %differenti gear ratio
eff = 0.99;
x = 0;
u = [0 0]';
p = 1;
N = 1;

ra = 0.01; %radius of rope
ca = pi*ra^2; %cross sectional area
ds = 8056; %density of steel
vol = ds*ca; %volume released
mode = 1;

for i = 1:100
    
    %drawpull
    Fdp = 2*( -r*b*int(ns*sin(th),a1,a2) - g*sin(sl) + r*b*int(shs*cos(th),a1,a2));
    
    %slipping ratio
    for j = 1:2
        %         S(j) = 1 - s(j,i);
        %         error_s = s(j,i) - sr;
        error_v = v(j,i) - vr;
        %         error_x = x(i) - xr(i);
        %         error_y = y(i) - yr;
        %  error_si =si(i)- sir;
        %         f(j) = -(S(j)/(m*v(j,i)))*(Fdp-m*g*sin(sl)) - rs*(S(j)^2)*Tr/(I*v(j,i));
        %         g(j) = rs*(S(j)^2)/(I*v(j,i));
        %         d(j) = S(j)*m*g*sin(sd)/(m*v(j,i));
    end
    
    %dynamics
    %     fs = -(S(j)/(m*v(j,i)))*(Fdp-m*g*sin(sl)) - rs*(S(j)^2)*Tr/(I*v(j,i));
    %     gs = rs*(S(j)^2)/(I*v(j,i));
    %     ds = S(j)*m*g*sin(sd)/(m*v(j,i));
    fv = -g*sin(sl);
    gv = (ig*id*eff/(rs*m));
    dv = -g*sin(sd);
    
    
    %sliding surface
    %     Sm = p*(error_s);
    Smv = p*(error_v);
    % Smk = p*(error_x+error_y+error_si);
    
    %control law
    %     u(:,i) = (1/gs)*(-fs-ds-Sm);
    uv1(:,i)= (1/2)*(1/gv)*(-fv-dv-Smv);
    uv2(:,i)= (1/2)*(1/gv)*(-fv-dv-Smv);
    
    %uv(:,i) = uv1(:,i) + uv2(:,i);
    
    %slip ratio derivative
    %     del_s = fs + gs*u(:,i) + ds;
    del_v = fv + gv*uv1(:,i)+ gv*uv2(:,i) + dv;
    
    %new slip ratio
    %     s(:,i+1) = s(:,i) + del_s;
    v(:,i+1) = v(:,i) + del_v;
    x(i+1) = x(i) + v(1,i+1)*dt;    
    
%     Rd(i+1) = -h*x(i+1)*d/(2*pi*R(i)*L); %rate of change of winding radius
%     R(i+1) = R(i) + Rd(i+1);
    
if mode == 1
    m = m - vol*v(1,i+1)*dt
else
    m = m + vol*v(1,i+1)*dt;
end

    t(i+1) = t(i) + 1;
end

% figure
% plot(t,s(1,:))

figure
plot(t,v(1,:))
hold on

% figure
% plot(t,R)
% hold on

figure
plot(t,x)
hold on

