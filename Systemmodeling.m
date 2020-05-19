
%Basic Principals of 2DOFBB
clc
Lt=.275;                  %length of table
g=9.81;                   %gravity                    
rarm=.0254;               %radius of arm
mball=.003;               %Mass of ball
rball=39.25 / 2 / 1000;   %radius of ball
jball=2/3*mball*rball^2;  %Inertia of ball
Tmin=-.523;               %Theta min
Tmax=.523;                %Theta max

K_bb = 6/5*g*rarm/Lt;

%Postion Dynamics of Ball w.r.t plate angle
num=K_bb
den=[1 0 0]
Pball=tf(num,den)

pzmap(Pball)
step(Pball) %Plot of position with 1 radian step input
axis([0 6 0 30])
ylabel('Position')



% Servo Plant Calculations 
%Tau and k calculation variables
Beq= 15e-3;
Rm=2.6;
eta_g=.9;
eta_m=.69;
km=.804/1000*60/(2*3.14);
kt=1.088*.278*.0254;
Kg=14*5;
jrotor=5.23e-5*.278*.0254
jtach=1e-5*.278*.0254
jm=jrotor+jtach
jg=3*.03*(1.5/2*.0254)^2/2
jext=.5*.04*(.05)^2
j1=jg+jext
Jeq=Kg^2*jm*eta_g+j1
Beq_v = ( Beq*Rm + eta_g*eta_m*km*kt*Kg^2 ) / Rm;% Viscous damping relative to motor
Am = eta_g*eta_m*kt*Kg / Rm;% Actuator gain 

K = Am / Beq_v      % Steady-state gain (rad/s/V)

tau = Jeq / Beq_v   % Time constant (s)




nums=K
dens=[tau 1 0]
Ps=tf(nums,dens) %Transfer function for servo plant
%step(Ps)
P=tf(Pball*Ps)   %Transfer Function for servo plant and ball position
%pzmap(P)
Da=20            %Angle input in degress
Ra=Da*pi/180     %Angle input in radians
step(Ra*P)       %Step input at inputed angle in radians
axis([0 7 0 30])
ylabel('Position')

%We can assume that the plant of the servo is negligable

%Specifications
PO=10;
ts= 3;
cs=.02; %settling time percentage
sse=.005; %Steady state error (m)

zeta = -log(PO/100) * sqrt( 1 / ( ( log(PO/100) )^2 + pi^2 ) )
wn = -log( cs * (1-zeta^2)^(1/2) ) / (zeta * ts)% Natural frequency from specifications (rad/s)
kd = 2*zeta*wn/K_bb% Velocity gain (rad.s/m) 
kp = wn^2/K_bb% Proportional gain (rad/m)



%Step Response with just proportional control
num1=kp*K_bb
den1=[1 0 kp*K_bb]
T=tf(num1, den1)
step(15*T)
axis([0 7 0 30])
ylabel('Position')

%Step response with proportional and derivative control 
numc=kp*K_bb
denc=[1 K_bb*kd K_bb*kp]
Y=tf(numc,denc)
step(Y)
ylabel('Position')

%State Space Representation of PD control
[A,B,C,D]=tf2ss(numc,denc)  %Controller canonical form









