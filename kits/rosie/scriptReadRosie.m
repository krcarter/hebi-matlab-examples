% reset everything to the initial state
HebiLookup.initialize()

% rebuild only the device table
HebiLookup.clearModuleList();
pause(0.5); % allow time to re-build

family = 'Rosie';
kin = HebiUtils.loadHRDF('hrdf/6-DoF_arm_w_gripper');
group = HebiLookup.newGroupFromNames(family, {
    'J1_base'
    'J2_shoulder'
    'J3_elbow'
    'J4_wrist1'
    'J5_wrist2'
    'J6_wrist3' });

while true
    fbk = group.getNextFeedbackFull();
    % calculate transforms from the world frame to the output frames of the individual bodies
    frames = kin.getForwardKinematics('OutputFrame', fbk.position);
    H_end = frames(:,:,end); % display end effector output
    pose = H_end(1:3,4);
    disp(pose')

    % calculate transforms from the world frame to the centers of mass of the individual bodies
    %CoM = kin.getForwardKinematics('CoMFrame', fbk.position);

    % calculate transform to only the end effector
    %endEffectorFrame = kin.getForwardKinematicsEndEffector();
end