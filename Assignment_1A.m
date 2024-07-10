// Part A MATLAB CODE
% assignment1PartA.m
% MTRN4230 Assignment 1 24T2
% Name: Jasmine Zeng
% Zid:  z5311791

clear; clc;

host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
% host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;
rtde = rtde(host, port);

disp("Enter the pickup position")
pickupJointConfiguration = readConfiguration();

clc;
disp("Move robot to dropoff position")
dropoffJointConfiguration = readConfiguration();

clc;
disp("Calculated pickup pose: ")
pickupPose = convertJointToPose(pickupJointConfiguration)
disp("Calculated dropoff pose: ")
dropoffPose = convertJointToPose(dropoffJointConfiguration)

disp("Set robot to remote control mode then click enter")
input('');

rtde.movel(pickupPose'+[0 0 20 0 0 0], 'pose')
rtde.movel(pickupPose', 'pose')
rtde.movel(pickupPose'+[0 0 20 0 0 0], 'pose')

rtde.movel(dropoffPose'+[0 0 20 0 0 0], 'pose')
rtde.movel(dropoffPose', 'pose')
rtde.movel(dropoffPose'+[0 0 20 0 0 0], 'pose')

% Function to convert user input to array
function configuration = readConfiguration()
    configuration = [];

    in = input('Enter joint configuration exactly in the form "j1,j2,j3,j4,j5,j6": ', 's');
    joints = split(in, ",");

    for joint = joints
        configuration = [configuration, str2double(joint)];
    end
end

% You must implement the following function
function outputPose = convertJointToPose(jointConfiguration)
    j = jointConfiguration;
    % Replace this with your implementation
    a = [0,-0.425,-0.3922,0,0,0];
    d = [0.1625,0,0,0.1333,0.0997,0.0996];
    alpha = [pi/2, 0 0, pi/2, -pi/2,0];
    T = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];

    for i=1:6
        newT = [cos(j(i)) -sin(j(i))*cos(alpha(i)) sin(j(i))*sin(alpha(i)) a(i)*cos(j(i));
                sin(j(i)) cos(j(i))*cos(alpha(i)) -cos(j(i))*sin(alpha(i)) a(i)*sin(j(i));
                0 sin(alpha(i)) cos(alpha(i)) d(i);
                0 0 0 1];
        T = T*newT;
    end 

    T = T*[0;0;0;1]*1000;
    angles = tr2rpy(T)';
    outputPose = [T(1) T(2) T(3) angles(1) angles(2) angles(3)];
    
    % You must not use RTDE at all in this implementation (it
    % must be done from first principles)
end
