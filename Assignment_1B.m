% RTDE UR5e  model for part B

startup_rvc;

a = [0,-0.425,-0.3922,0,0,0];
d = [0.1625,0,0,0.1333,0.0997,0.0996];
alpha = [pi/2, 0 0, pi/2, -pi/2,0];
L = [];

for i=1:6
    L(i) = Link('revolute', 'd',d(i),'a',a(i),'alpha',alpha(i),'offset',0);
end

robot = SerialLink(L,'name','UR5e');
Matrix = robot.fkine([theta]);
