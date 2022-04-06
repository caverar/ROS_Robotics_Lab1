%% Iniciar instancia de nodo maestro
clear
clc
rosinit;


%%
clc
poseSub = rossubscriber('/turtle1/cmd_vel','geometry_msgs/Twist');
pause(1)
poseSub.LatestMessage
%% Ejemplo de la guia
clc
velPub = rospublisher('/turtle1/cmd_vel','geometry_msgs/Twist');
velMsg = rosmessage(velPub);

velMsg.Linear.X=1;
send(velPub,velMsg)
pause(1)


%% b. Desarrollo de ejercicio 
%% b.1
clc
poseSub = rossubscriber('/turtle1/pose','turtlesim/Pose');
pause(1)
poseSub.LatestMessage


%% b.2
clc
% Parametros
X = 3;
Y = 3;
theta = pi/3;
linearVelocity = 22;
AngularVelocity = 22;

% Cacular Velocidad lineal en cada direccion
Vx = linearVelocity * cos(theta);
Vy = linearVelocity * sin(theta);

%Teleport service
teleportClient = rossvcclient('/turtle1/teleport_absolute');
teleportMsg = rosmessage(teleportClient);
teleportMsg.X = X;
teleportMsg.Y = Y;
teleportMsg.Theta = theta;
call(teleportClient,teleportMsg)

%Movement publication
movementPub = rospublisher('/turtle1/cmd_vel','geometry_msgs/Twist');
movementMsg = rosmessage(movementPub);
disp(movementMsg)
movementMsg.Linear.X=Vx;
movementMsg.Linear.Y=Vy;
movementMsg.Angular.Z=AngularVelocity;
send(movementPub,movementMsg)


pause(1)

%poseMsg.LinearVelocity = 1;
%teleportPub = rospublisher('/turtle1/teleport_absolute', 'turtlesim/TeleportAbsoluteRequest');

%% Cerrar instancia de nodo maestro
clc
rosshutdown;