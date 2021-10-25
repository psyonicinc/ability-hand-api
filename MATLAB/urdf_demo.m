%%
robot = importrobot('ability_hand.urdf');


%% Show the robot, with constraints placed on the driven joints

%finger angles, in degrees.
%index, middle, ring, pinky, thumb flexor, thumb rotator
%thumb rotator angle range is 10 to -120
%finger range is 10-110
q = [60,0,0,0,0,0];  

robot.Bodies{1,9}.Joint.HomePosition = q(6)*pi/180;
robot.Bodies{1,10}.Joint.HomePosition = q(5)*pi/180;
robot.Bodies{1,1}.Joint.HomePosition = q(1)*pi/180;
robot.Bodies{1,2}.Joint.HomePosition = get_abh_4bar_driven_angle(robot.Bodies{1,1}.Joint.HomePosition);
robot.Bodies{1,3}.Joint.HomePosition = q(2)*pi/180;
robot.Bodies{1,4}.Joint.HomePosition = get_abh_4bar_driven_angle(robot.Bodies{1,3}.Joint.HomePosition);
robot.Bodies{1,5}.Joint.HomePosition = q(4)*pi/180;
robot.Bodies{1,6}.Joint.HomePosition = get_abh_4bar_driven_angle(robot.Bodies{1,5}.Joint.HomePosition);
robot.Bodies{1,7}.Joint.HomePosition = q(3)*pi/180;
robot.Bodies{1,8}.Joint.HomePosition = get_abh_4bar_driven_angle(robot.Bodies{1,7}.Joint.HomePosition);

show(robot,'visuals','on','collision','off', 'frames', 'off');

zlim([-150e-3,50e-3]);
xlim([-100e-3,100e-3]);
ylim([-50e-3,50e-3]);
% view(az,el);
% zoom(2);
