% Mobile Base with 6-DoF arm & gripper.  Tele-op using HEBI Mobile I/O App.
%
% Dave Rollinson
% July 2018

%%
function rosieDemo_GE( omni )

    HebiLookup.initialize();

    % Optional step to limit the lookup to a set of interfaces or modules
    % HebiLookup.setLookupAddresses('10.10.10.255');
    enableLogging = true;

    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Setup Arm and Gripper %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    robotFamily = 'Rosie';
    [ arm, armParams, gripper ] = setupArmWithGripper(robotFamily);
    
    %%
    %%%%%%%%%%%%%%%%%%%%%
    % Setup Mobile Base %
    %%%%%%%%%%%%%%%%%%%%%
    switch lower('omni')
        case 'omni'
            [chassisParams, chassisTrajGen] = setupOmniBase();
        case 'diff-drive'
            [chassisParams, chassisTrajGen] = setupDiffDriveBase();
        case 'mecanum'
            [chassisParams, chassisTrajGen] = setupMecanumBase();
        otherwise
            disp('Base type not recognized.'); 
            disp('Please choose: OMNI, DIFF-DRIVE, or MECANUM');
            return;
    end

    % Not used anymore? TODO: remove?
% %     wheelRadius = chassisParams.wheelRadius; 
% %     wheelBase = chassisParams.wheelBase;  
% %     chassisCoM = chassisParams.chassisCoM;  
   
    % Max speed
    maxLinSpeed = chassisParams.maxLinSpeed; 
    maxRotSpeed = chassisParams.maxRotSpeed; 
    
    % Maps linear (XYZ) chassis velocities to wheel velocities
    chassisToWheelVelocities = chassisParams.wheelVelocityMatrix;
    chassisEffortsToWheelEfforts = chassisParams.wheelEffortMatrix;
    chassisMass = chassisParams.chassisMass; 
    chassisInertiaZZ = chassisParams.chassisInertiaZZ; 
    chassisMassMatrix = diag( [chassisMass; chassisMass; chassisInertiaZZ] );

    % Create Group
    wheelGroup = HebiLookup.newGroupFromNames(robotFamily, chassisParams.wheelModuleNames);
    HebiUtils.sendWithRetry(wheelGroup, 'gains', chassisParams.wheelGains);
    wheelCmd = CommandStruct();

    %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Setup Mobile Phone Input %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Button Mapping
    startPoseButton = 'b1';
    resetPoseButton = 'b2';
    driveModeButton = 'b5';
    quitDemoButton = 'b8';
    xVelAxis = 'a8'; % Right Pad Up/Down
    yVelAxis = 'a7'; % Right Pad Left/Right
    rotVelAxis = 'a1'; % Left Pad Left/Right
    
    % Setup IO Command Struct for Input Mobile Device
    phoneCmd = IoCommandStruct();

    % Search for phone controller. Allow retry because phones tend to
    % drop out when they aren't used (i.e. sleep mode)
    phoneName = 'mobileIO';
    while true
        try
            fprintf('Searching for phone Controller...\n');
            phoneGroup = HebiLookup.newGroupFromNames(robotFamily, phoneName);
            disp('Phone Found.  Starting up');
            break;
        catch
            pause(1.0);
        end
        
        % Alternate Cyan and Magenta LEDs to signal "Searching for Controller"
        wheelGroup.send('led','c');
        pause(1.0);
        wheelGroup.send('led','m');
    end
    
    
    %%
    %%%%%%%%%%%%%%%%%%%%%%%
    % Begin the demo loop %
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Start background logging
    if enableLogging
        arm.group.startLog('dir','logs');
        wheelGroup.startLog('dir','logs');
    end
    
    % This outer loop is what we fall back to anytime we 're-home' the arm
    abortFlag = false;
    while ~abortFlag
        phoneCmd.e1 = 1;  % Highlight Start button
        phoneCmd.e2 = 1;  % Highlight Reset button
        phoneCmd.e5 = 1;  % Highlight Drive Mode button
        phoneCmd.e8 = 1;  % Highlight quit button   
        phoneCmd.(startPoseButton) = 1; % Toggle Button
        phoneCmd.(resetPoseButton) = 1; % Toggle Button
        phoneCmd.(driveModeButton )= 1; % Toggle Button
        
        % Mobile IO Homing Procedure
        controllerColor = 'b';
        
        HebiUtils.sendWithRetry(phoneGroup, ...
            'led', controllerColor, ... % send LED command
            'IoCommand', phoneCmd); % send button and slider settings
        
        phoneGroup.send('clearLog',true);
        phoneGroup.send('appendLog','Robot Homing Sequence');
        phoneGroup.send('appendLog','Please wait...');
    
        % LEDs to Magenta to signal "Homing Procedure"
        wheelGroup.send('led','m');
        
        % Exaggerate Z-Axis by 2x, X-Y are 1-to-1. 
        xyzScale = [1 1 2]';
        
        % Move to current coordinates
        xyzTarget_init = [0.34; 0.0; 0.23];
        rotMatTarget_init = R_z(pi)*R_y(-pi/2);  % Gripper down
        
        ikPosition = arm.kin.getIK(...
            'xyz', xyzTarget_init, ...
            'so3', rotMatTarget_init, ...
            'initial', armParams.ikSeedPos );
        
        % Slow trajectory timing for the initial move to home position
        arm.trajGen.setSpeedFactor( 0.5 );
        arm.trajGen.setMinDuration( 1.0 );
        
        % Move to initial position
        arm.update();
        arm.clearGoal(); % in case we run only this section
        arm.setGoal(ikPosition);
        while ~arm.isAtGoal
            arm.update();
            arm.send();
        end
        
        % Reset behavior to normal speed
        arm.trajGen.setSpeedFactor(armParams.defaultSpeedFactor);
        arm.trajGen.setMinDuration(armParams.minTrajDuration);
        
        % Grab initial pose from phone
        fbkPhoneMobile = phoneGroup.getNextFeedbackMobile();
        fbkPhoneIO = phoneGroup.getNextFeedbackIO();
        %{
        q = [ fbkPhoneMobile.arOrientationW ...
              fbkPhoneMobile.arOrientationX ...
              fbkPhoneMobile.arOrientationY ...
              fbkPhoneMobile.arOrientationZ ];     
        R_init = HebiUtils.quat2rotMat( q );

        xyz_init = [ fbkPhoneMobile.arPositionX;
                     fbkPhoneMobile.arPositionY; 
                     fbkPhoneMobile.arPositionZ ];
        xyzPhoneNew = xyz_init;
        %}
        
        phoneGroup.send('appendLog','Arm Homing Sequence');
        phoneGroup.send('appendLog','Please wait...');
    
        % Move to initial position
        arm.update();
        arm.clearGoal(); % in case we run only this section
        arm.setGoal(ikPosition);
        
        % Initialize a trajectory for the base. (By passing velocities
        % as positions we can compute a velocity trajectory. The constant
        % time vector makes it act like a 'minimum-jerk' lowpass on the 
        % input)
        timeNow = arm.state.time;
        chassisTrajStartTime = timeNow;
        
        velocities = zeros(2, 3);
        chassisTraj = chassisTrajGen.newJointMove( velocities, ...
            'Time', [0 chassisParams.rampTime]);
        
        % Initialize wheel position to current feedback and then
        % integrate from there.
        wheelFbk = wheelGroup.getNextFeedbackFull();
        wheelCmd.position = wheelFbk.position;
        
        phoneGroup.send('appendLog','Arm Homing Sequence');
        phoneGroup.send('appendLog','Please wait...');
        
        % Move to initial position
        arm.update();
        arm.clearGoal(); % in case we run only this section
        arm.setGoal(ikPosition);
        
        % Set Mobile IO To "Control Mode" Screen Setup
        controllerColor = 'g';
    
        HebiUtils.sendWithRetry(phoneGroup, ...
            'led', controllerColor, ... % send LED command
            'IoCommand', phoneCmd); % send button and slider settings
    
        wheelGroup.send('led',[]);

        phoneGroup.send('clearLog',true);
        phoneGroup.send('appendLog','Robot Ready to Control');
        phoneGroup.send('appendLog','HEBI-GE DEMO');
        phoneGroup.send('appendLog','B1: Start Demo');
        phoneGroup.send('appendLog','B2: Reset Demo');
        phoneGroup.send('appendLog','B5: Robot Drive Mode');
        phoneGroup.send('appendLog','B8: Quit');
    
        phoneGroup.send('appendLog','A8/A7 Controls Direction');
        phoneGroup.send('appendLog','A1 Controls Orientation');
        
        moveCounter = 0;
        moveDir = 1;
        while ~abortFlag

            %%%%%%%%%%%%%%%%%%%
            % Gather Feedback %
            %%%%%%%%%%%%%%%%%%%
            try
                arm.update();
                arm.send();
                
                % Wheel feedback. Uncomment if used
                % wheelFbk = wheelGroup.getNextFeedback();
            catch
                disp('Could not get robot feedback!');
                break;
            end 

            % We get feedback from the phone into the existing structs. The
            % timeout of 0 means that the method returns immediately and won't
            % wait for feedback. If there was no feedback, the method returns
            % empty ([]), and the data in the passed in structs does not get
            % overwritten.
            % We do this because the mobile device is typically on wireless and
            % might drop out or be really delayed, in which case we would
            % rather keep running with an old data instead of waiting here for
            % new data.
            hasNewPhoneFbk = ~isempty(phoneGroup.getNextFeedback( ...
                fbkPhoneIO, fbkPhoneMobile, ... % overwrite existing structs
                'timeout', 0 )); % prevent blocking due to bad comms
            

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Read/Map Joystick Inputs %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Check for quit command
            if fbkPhoneIO.(quitDemoButton)
                abortFlag = true;
                break;
            end
       
            % Check for restart command
            %{
            if fbkPhoneIO.(resetPoseButton)
                % - Move arm back to intial position %%
                % - init arm pose -> setGoal %%
                % Move to current coordinates
                disp('Resetting Pose')
                xyzTarget_init = [0.34; 0.0; 0.23];
                rotMatTarget_init = R_z(pi)*R_y(-pi/2);  % Gripper down

                ikPosition = arm.kin.getIK(...
                    'xyz', xyzTarget_init, ...
                    'so3', rotMatTarget_init, ...
                    'initial', armParams.ikSeedPos );

                % Slow trajectory timing for the initial move to home position
                arm.trajGen.setSpeedFactor( 0.5 );
                arm.trajGen.setMinDuration( 1.0 );

                % Move to initial position
                arm.update();
                arm.clearGoal(); % in case we run only this section
                arm.setGoal(ikPosition);
                while ~arm.isAtGoal
                    arm.update();
                    arm.send();
                end
                
                % Reset behavior to normal speed
                arm.trajGen.setSpeedFactor(armParams.defaultSpeedFactor);
                arm.trajGen.setMinDuration(armParams.minTrajDuration);
                % Move robot frame back to initial position
                % wheel 0 position -> setGoal
            end
      
            %}
            
            %%%%%%%%%%%%%%%
            % Arm Control %
            %%%%%%%%%%%%%%%
            
            % Check for restart command
            if fbkPhoneIO.(resetPoseButton)
                % - Move arm back to intial position %%
                % - init arm pose -> setGoal %%
                % Move to current coordinates
                disp('Resetting Pose')
                xyzTarget_init = [0.34; 0.0; 0.23];
                rotMatTarget_init = R_z(pi)*R_y(-pi/2);  % Gripper down

                ikPosition = arm.kin.getIK(...
                    'xyz', xyzTarget_init, ...
                    'so3', rotMatTarget_init, ...
                    'initial', armParams.ikSeedPos );

                % Slow trajectory timing for the initial move to home position
                arm.trajGen.setSpeedFactor( 0.5 );
                arm.trajGen.setMinDuration( 1.0 );

                % Move to initial position
                arm.update();
                arm.clearGoal(); % in case we run only this section
                arm.setGoal(ikPosition);
                while ~arm.isAtGoal
                    arm.update();
                    arm.send();
                end
                
                % Reset behavior to normal speed
                arm.trajGen.setSpeedFactor(armParams.defaultSpeedFactor);
                arm.trajGen.setMinDuration(armParams.minTrajDuration);
                % Move robot frame back to initial position
                % wheel 0 position -> setGoal
                
            % Check for Drive Mode Command
            elseif fbkPhoneIO.(driveModeButton) == 1
                %disp('Drive Mode')
                xyzTarget_init = [0.34;  0.0; 0.23];
                rotMatTarget_init = R_z(pi)*R_y(-pi/2);  % Gripper down

                ikPosition = arm.kin.getIK(...
                    'xyz', xyzTarget_init, ...
                    'so3', rotMatTarget_init, ...
                    'initial', armParams.ikSeedPos );

                % Move to initial position
                arm.update();
                arm.clearGoal(); % in case we run only this section
                arm.setGoal(ikPosition); 
                %{
                while ~arm.isAtGoal
                        arm.update();
                        arm.send();
                end
                %}
                 % Map joystick input to linear speeds       
                xVel = maxLinSpeed * fbkPhoneIO.(xVelAxis);
                yVel = -maxLinSpeed * fbkPhoneIO.(yVelAxis);
                rotVel = maxRotSpeed * fbkPhoneIO.(rotVelAxis); 
                desiredChassisVel = [xVel yVel rotVel];

                % Find the current point in chassis trajectory
                % (linear velocity) we plan a smooth transition
                t = min(timeNow - chassisTrajStartTime, chassisTraj.getDuration);
                [chassisVel, chassisAccel, chassisJerk] = chassisTraj.getState(t); 

                % Compute a trajectory that smoothly transitions to 
                % the new desired velocities
                velocities = [chassisVel; desiredChassisVel];
                accelerations = [chassisAccel; zeros(1,3) ];
                jerks = [chassisJerk; zeros(1,3) ];
                chassisTraj = chassisTrajGen.newJointMove( velocities, ...
                    'Velocities', accelerations, ...
                    'Accelerations', jerks, ...
                    'Time', [0 chassisParams.rampTime] );
                chassisTrajStartTime = timeNow;

                % Compute dt since the last update for integrating position
                timeLast = timeNow;
                timeNow = arm.state.time;
                dt = timeNow - timeLast;

                % Convert linear velocities into wheel/joint velocities
                wheelCmd.velocity = (chassisToWheelVelocities * chassisVel')';
                wheelCmd.position = wheelCmd.position + wheelCmd.velocity * dt;
                wheelCmd.effort = (chassisEffortsToWheelEfforts * ...
                                (chassisMassMatrix * chassisAccel'))';
                wheelGroup.send(wheelCmd);
                 %Allows the user to drive the robot around
            % Check for Start Raster Command
             elseif fbkPhoneIO.(startPoseButton) == 1 ... 
                     && fbkPhoneIO.(resetPoseButton) == 0 ...
                     && fbkPhoneIO.(driveModeButton) == 0
                % Raster Pattern
                % A -> B
                %      |
                % D <- C
                % |
                % E -> F
                %      |
                % H <- G
                
                x_const = 0.34; % constant distance arm is extended to
                % - Defining Grid Robot will Raster over - %
                ydist = 0.280; %m
                zstart = 0.55; %m
                zdist = 0.45; %m
                
                % I hardcoded this but this should definately be a for loop
                A = [ydist/2, zstart];
                B = [-ydist/2, zstart];
                C = [-ydist/2, zstart - zdist/3];
                D = [ydist/2, zstart - zdist/3];
                E = [ydist/2, zstart - 2*zdist/3];
                F = [-ydist/2, zstart - 2*zdist/3];
                G = [-ydist/2, zstart - zdist];
                H = [ydist/2, zstart - zdist];
                
                % - Creating XYZ waypoints the robot raster over - %
                nWay = 5;
                
                AB = [linspace(A(1),B(1),nWay); linspace(A(2),B(2),nWay)];
                BC = [linspace(B(1),C(1),nWay); linspace(B(2),C(2),nWay)];
                CD = [linspace(C(1),D(1),nWay); linspace(C(2),D(2),nWay)];
                DE = [linspace(D(1),E(1),nWay); linspace(D(2),E(2),nWay)];
                EF = [linspace(E(1),F(1),nWay); linspace(E(2),F(2),nWay)];
                FG = [linspace(F(1),G(1),nWay); linspace(F(2),G(2),nWay)];
                GH = [linspace(G(1),H(1),nWay); linspace(G(2),H(2),nWay)];
                HA = [linspace(H(1),A(1),nWay); linspace(H(2),A(2),nWay)];
                
                yzTraj = [AB';BC';CD';DE';EF';FG';GH';HA'];
                rotMatTarget = R_z(pi)*R_y(-pi/2);  % Gripper down
                
                
                for index = 1:length(yzTraj)
                    fbkPhoneIO = phoneGroup.getNextFeedbackIO();
                    disp('reset Button:')
                    disp(fbkPhoneIO.(resetPoseButton))
                    if fbkPhoneIO.(startPoseButton) == 0
                        disp('Triggered Pause Demo')
                        break;
                    end
                    if fbkPhoneIO.(resetPoseButton) == 1
                        disp('Triggered Reset')
                        break;
                    end
                    if fbkPhoneIO.(driveModeButton) == 1
                        disp('Triggered Drive Mode')
                        break;
                    end
                    if fbkPhoneIO.(quitDemoButton) == 1
                        disp('Triggered Quit Demo')
                        break;
                    end
                    y_const = yzTraj(index, 1);
                    z_const = yzTraj(index, 2);
                    target_xyz = [x_const, y_const, z_const];
                    disp(target_xyz)
                    
                    ikPosition = arm.kin.getIK(...
                    'xyz', target_xyz, ...
                    'so3', rotMatTarget, ...
                    'initial', arm.state.fbk.position );
                    % Slow trajectory timing for the initial move to home position
                    arm.trajGen.setSpeedFactor( 0.5 );
                    arm.trajGen.setMinDuration( 1.0 );
                    
                    % Move to initial position
                    arm.update();
                    arm.clearGoal(); % in case we run only this section
                    arm.setGoal(ikPosition);
                    while ~arm.isAtGoal
                        arm.update();
                        arm.send();
                
                    end
                    
                end
             end
        end
    end
    
    % Set Mobile IO to "Demo Ended" State
    phoneCmd.f4 = 0; % Reset sslider
    phoneCmd.f5 = 0; % Reset slider
    phoneCmd.f3 = 0; % Reset slider
    phoneCmd.f6 = 0;  % Reset slider

    controllerColor = 'r';
    wheelGroup.send('led','r');
    
    HebiUtils.sendWithRetry(phoneGroup, ...
    'led', controllerColor, ... % send LED command
    'IoCommand', phoneCmd); % send button and slider setting

    phoneGroup.send('clearLog',true);
    phoneGroup.send('appendLog','Demo Stopped');
    phoneGroup.send('appendLog','Restart Robot and Mobile App');
    disp('Demo Stopped.')

end

