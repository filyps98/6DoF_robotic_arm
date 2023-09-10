clear;
close all

%Declaring DH parameters scaled compared to real life
dhparams = [2*0.014   	pi/2	2*0.096   	0;
            2*0.12	pi       0       pi/2;
            2*0.0055	pi/2	2*0.0095	pi;
            2*0.005   	-pi/2	2*0.1228	pi;
            2*0.13 -pi/2 2*0.005 -pi/2];

%Creating BodyTree
robot = rigidBodyTree;

%Definition of robot initial angles
theta = [0; pi/2; pi; pi; -pi/2];
theta_ard = [pi/2; pi/2; pi/2; pi/2; pi/2];

%Robot construction from bodies and joints using DH parameters
bodies = cell(5,1);
joints = cell(5,1);
for i = 1:5
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};
    if i == 1 % Add first body to base
        addBody(robot,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end


%Displaying robot simulation
showdetails(robot)
config = homeConfiguration(robot);

%Initial configuration
for k=2:5
    config(k).JointPosition = theta(k);
end

%Definition of arduino port and Pins
a = arduino('/dev/cu.usbmodem101', 'Uno');
Pin = {'D3';'D5';'D6';'D9';'D10';'D11'}

%Moving the robot to initial configuration
for k=1:6
    servoMotor(k) = servo(a, Pin{k});

    if k<6
        writePosition(servoMotor(k), (90)/180);
    else
        writePosition(servoMotor(k), (40)/180);
    end
    
end

show(robot,config);
pause(1);


%Initializing Inverse Kinematics and its Position and Joint limits 
gik = generalizedInverseKinematics;
gik.RigidBodyTree = robot;

gik.ConstraintInputs={'position','joint'};

%Define the joint limits
JointLimits = [theta(1)-pi/2 theta(1)+pi/2;
theta(2) - pi/6 theta(2) + pi/6;
theta(3) - pi/3 theta(3) + pi/3;
theta(4) - pi/2 theta(4) + pi/2;
theta(5) - pi/2 theta(5) + pi/2];

jointConst = constraintJointBounds(robot);
jointConst.Bounds = JointLimits;

posTgt = constraintPositionTarget('body5');

%x = [0.45, 0.5, 0.45, 0.38, 0.4, 0.4];
%y = [0.090, 0.08, 0.17, 0.13, 0.09 , 0.09];
%z = [0.4376, 0.3, 0.26, 0.35, 0.05 , 0.2];

% Coordinates of the points of trajectory
x = [0.45, 0.5, 0.45, 0.38, 0.4, 0.4];  
y = [0.090, 0.1, 0.2, 0.25, 0.35 , 0.35];
z = [0.25, 0.5, 0.4, 0.2, 0.05 , 0.2];

%Asking the user for parameters for path planning 
prompt = "Can you insert the number of interpolated points you want to pass? Speed of robot [0,1]";
time_interval = input(prompt);

prompt = "Can you insert the amount of smoothing of the trajectory?";
smoothing = input(prompt);

%Initialize the simulation figure
fig = figure(Name="Arduino Model trajectory interpolation and smoothing")

% Set the dimensions of the figure window
width = 800;    % Width of the figure window in pixels
height = 600;   % Height of the figure window in pixels

% Get the screen size
screenSize = get(0, 'ScreenSize');
screenWidth = screenSize(3);
screenHeight = screenSize(4);

% Set the figure window position and size
set(fig, 'OuterPosition', [(screenWidth - width) / 2, (screenHeight - height) / 2, width, height]);

%Sampling of the initial trajectory
t = 0:numel(x)-1;
tInterp = 0:time_interval:numel(x)-1;
xInterp = interp1(t, x, tInterp);
yInterp = interp1(t, y, tInterp);
zInterp = interp1(t, z, tInterp);

%Smoothing of the trajectory
smooth_x = smoothdata(xInterp,"gaussian",smoothing);
smooth_y = smoothdata(yInterp,"gaussian",smoothing);
smooth_z = smoothdata(zInterp,"gaussian",smoothing);

%iterate on each point of the trajectory
for j=1:length(tInterp)
    
    %Joint angle solution from IK given a target position
    posTgt.TargetPosition = [smooth_x(j)    smooth_y(j)   smooth_z(j)];
    [q,solutionInfo] = gik(config, posTgt, jointConst);
    
    %Sending signals to the motors to perform movements
    for k=1:5
        final_ard(k) = rad2deg((q(k).JointPosition - config(k).JointPosition) + theta_ard(k))/180;
        writePosition(servoMotor(k), final_ard(k));
        pause(0.01);
    end
    
    %Show robot and its trajectory step by step
    show(robot,q);

    hold on
    plot3(xInterp, yInterp, zInterp, 'b','LineWidth', 0.5);
    hold on
    plot3(smooth_x, smooth_y, smooth_z, 'r', 'LineWidth', 2);
    dummy1 = plot(NaN, NaN, 'b', 'LineWidth', 2);
    dummy2 = plot(NaN, NaN, 'r', 'LineWidth', 2);
    zlim([0, 1]);

    % Set the legend with custom colors
    legend([dummy1, dummy2], 'Original Interpolation', 'Smoothed Interpolation');
    % Create note string with parameters
    note = sprintf('Parameters:\n speed = %.2f \n smoothing = %.2f', time_interval, smoothing);

    % Add annotation to the plot
    annotation('textbox', [0.77, 0.74, 0.1, 0.1], 'String', note, 'BackgroundColor', 'white', 'EdgeColor', 'black');
    hold off;
    
    %Updating to new position
    for k=1:5
        theta_ard(k) = q(k).JointPosition - config(k).JointPosition + theta_ard(k);
    end
    

    config = q;

end


show(robot,config);