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

%Define the joint limits
JointLimits = [theta(1)-pi/2 theta(1)+pi/2;
theta(2) - pi/6 theta(2) + pi/6;
theta(3) - pi/3 theta(3) + pi/3;
theta(4) - pi/2 theta(4) + pi/2;
theta(5) - pi/2 theta(5) + pi/2];

%Robot construction from bodies and joints using DH parameters
bodies = cell(5,1);
joints = cell(5,1);
for i = 1:5
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    joints{i}.PositionLimits = JointLimits(i,:)
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};
    if i == 1 % Add first body to base
        addBody(robot,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end

%Defining Collision area on each body of the robot
collisionObj = collisionCylinder(0.05,0.15);

addCollision(robot.Base,collisionObj);
for i = 1:robot.NumBodies

    addCollision(robot.Bodies{i},collisionObj);
end

%Displaying robot simulation
showdetails(robot)
figure(Name="Arduino Model")
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


%Showing the initial position of the robot
show(robot,config);
axis([-1,1,-1,1,0,1])
pause(5);

%Initializing the inverse Kinematiks
gik = generalizedInverseKinematics;
gik.RigidBodyTree = robot;

%Define the costraints
gik.ConstraintInputs={'position','joint'};

jointConst = constraintJointBounds(robot);
jointConst.Bounds = JointLimits;

posTgt = constraintPositionTarget('body5');

%Define and show the point map obstacle
env = {collisionBox(0.1, 0.1, 0.5)}
env{1}.Pose(1:3, end) = [0.5 0.2 0.25];

hold on
show(env{1});
axis([-1 1, -1 1, 0 1]);

%Defining and determining initial position the robot should reach
%using ik
posTgt.TargetPosition = [0.3, -0.3, 0.15];
[startconfig,solutionInfo] = gik(config, posTgt, jointConst);

%Moving to initial position
for i=0:0.1:1

    for k=1:5
        final_ard(k) = rad2deg((startconfig(k).JointPosition - config(k).JointPosition)*i + theta_ard(k))/180;
        writePosition(servoMotor(k), final_ard(k));
        pause(0.01);
    end

end

%Updating to new position
for k=1:5
    theta_ard(k) = startconfig(k).JointPosition - config(k).JointPosition + theta_ard(k);
end

config = startconfig

%I calculate the inverse kinematics for a specific final target point 
posTgt.TargetPosition = [0.1, 0.5, 0.4376];
[goalconfig,solutionInfo] = gik(startconfig, posTgt, jointConst);

%Initialize the RRT algorithm and its parameters
planner = manipulatorRRT(robot,env);
planner.ValidationDistance=0.1;
planner.MaxConnectionDistance=0.2;

%Determine the path from initial position to final position, interpolate
%and smooth
plannedpath = plan(planner,[startconfig.JointPosition],[goalconfig.JointPosition]);
interpoalatedpath = interpolate(planner,plannedpath);
smoothedpath(:,1:5) = smoothdata(interpoalatedpath(:,1:5),"gaussian",10);

%iterate on each point of the trajectory
s = 1;
for j=1:length(interpoalatedpath)

    q = smoothedpath(j,:);
        
    step = struct(config);
        
    for i=0:1:1

            %Sending signals to the motors to perform movements
            for k=1:5
                step(k).JointPosition = config(k).JointPosition + (q(k) - config(k).JointPosition)*(i);
                final_ard(k) = rad2deg(step(k).JointPosition - config(k).JointPosition + theta_ard(k))/180;
                writePosition(servoMotor(k), final_ard(k));
                pause(0.01);
            
            end

            %Calculate the trajectory of the end-effector
            transform = getTransform(robot,step,"body5");
            alpha = transform*[0 0 0 1]';
            trajectory(s,:) = alpha;
            s = s + 1;
            
            %Displaying the trajectory of the end-effector
            show(robot,step);
            axis([-1 1, -1 1, 0 1]);
            hold on
            show(env{1});
            hold on
            plot3(trajectory(:,1), trajectory(:,2),  trajectory(:,3), 'r', 'LineWidth', 2);
            pause(0.05)

            hold off
       
    end
    
    %Updating to new position
    for k=1:5
        theta_ard(k) = step(k).JointPosition - config(k).JointPosition + theta_ard(k);
    end
    config = step;

end

show(robot,config);
hold on
curve = cscvn(trajectory.');
fnplt(curve)
pause(2)
hold on
show(env{1});
hold off