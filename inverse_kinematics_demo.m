function inverse_kinematics_demo(test_case)
% This function demonstrates how to calculate the inverse kinematics for
% arm movement for a right arm reaching movement from lower right to upper
% left (relative to the shoulder)
%
% WW/PM&R/Pitt  10/20/2010
% Last Update   10/20/2010


close all
upper_arm_len = 1;
fore_arm_len = 1;
which_hand = 'right';

if ~exist('test_case', 'var')
    test_case = 0;
end
switch test_case
    case 0 % a reaching from lower right to upper left
        start_point = [1 0.5 -1];
        stop_point = [-1 1.8 0.5];
    case 1 % internal rotation with elbow at 90 degrees
        start_point = [0 fore_arm_len -upper_arm_len];
        stop_point = [-fore_arm_len*sin(deg2rad(60)) fore_arm_len*cos(deg2rad(60)) -1];
    case 2 % elbow flexion
        start_point = [0 fore_arm_len -upper_arm_len];
        stop_point = [0 fore_arm_len*sin(deg2rad(30)) -upper_arm_len+fore_arm_len*cos(deg2rad(30))];
    case 3 % some mix of shoulder adduction etc
        start_point = [0 fore_arm_len -upper_arm_len];
        stop_point = [upper_arm_len fore_arm_len 0];
    case 4
        start_point = [0.1 0 0];
        stop_point = [1 0 0];        
end

num_points = 50;

x_traj = linspace(start_point(1), stop_point(1), num_points);
y_traj = linspace(start_point(2), stop_point(2), num_points);
z_traj = linspace(start_point(3), stop_point(3), num_points);

figure('Color', 'k')
for i = 1:num_points
    if x_traj(i)>=0
        twisting_angle = 0;
    elseif x_traj(i)<=-1
        twisting_angle = 60;
    else
        twisting_angle = -60*x_traj(i);
    end
    if test_case > 0
        twisting_angle = 0;
    end
    if test_case==4
        twisting_angle = i/num_points*90;
    end
    end_point = [x_traj(i) y_traj(i) z_traj(i)];
    [shoulder_flexion(i), shoulder_adduction(i), shoulder_internal_rotation(i), elbow_flexion(i), misc_items] = inverse_kinematics(end_point, twisting_angle, upper_arm_len, fore_arm_len, which_hand);
    
    % Now plot the triangle formed by upper arm, lower arm
    shoulder = [0 0 0];
    elbow = misc_items.elbow_point;
    hand = end_point;
    
    clf
    patch([shoulder(1) elbow(1) hand(1)], [shoulder(2) elbow(2) hand(2)], [shoulder(3) elbow(3) hand(3)], 'c')
    hold on
    plot3([shoulder(1) elbow(1)], [shoulder(2) elbow(2)], [shoulder(3) elbow(3)], 'r', 'LineWidth', 8)
    plot3([elbow(1) hand(1)], [elbow(2) hand(2)], [elbow(3) hand(3)], 'r', 'LineWidth', 8)
    plot3([shoulder(1) hand(1)], [shoulder(2) hand(2)], [shoulder(3) hand(3)], 'w', 'LineWidth', 8)
    view(50, 10)
    x_lim = get(gca, 'XLim');
    y_lim = get(gca, 'YLim');
    z_lim = get(gca, 'YLim');
    x_lim = 1.5; %max(abs(x_lim));
    y_lim = 1.5; %max(abs(y_lim));
    z_lim = 1.5; %max(abs(z_lim));
    set(gca, 'XLim', [-x_lim x_lim], 'YLim', [-y_lim, y_lim], 'ZLim', [-z_lim, z_lim], 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    grid on
    xlabel('X Right')
    ylabel('Y Forward')
    zlabel('Z Up')
    title('Inverse Kinematics Demo', 'Color', 'w')
%     plot3([0 perp_vector1(1)], [0 perp_vector1(2)], [0 perp_vector1(3)], 'm', 'LineWidth', 5)
%     plot3([0 perp_vector2(1)], [0 perp_vector2(2)], [0 perp_vector2(3)], 'y', 'LineWidth', 5)
%     plot3([0 twist_vector(1)], [0 twist_vector(2)], [0 twist_vector(3)], 'b', 'LineWidth', 5)

    plot3([0 misc_items.upper_arm_x_vector(1)], [0 misc_items.upper_arm_x_vector(2)], [0 misc_items.upper_arm_x_vector(3)], 'm', 'LineWidth', 5)
    plot3([0 misc_items.upper_arm_y_vector(1)], [0 misc_items.upper_arm_y_vector(2)], [0 misc_items.upper_arm_y_vector(3)], 'y', 'LineWidth', 5)
    plot3([0 misc_items.upper_arm_z_vector(1)], [0 misc_items.upper_arm_z_vector(2)], [0 misc_items.upper_arm_z_vector(3)], 'b', 'LineWidth', 5)

    pause(0.1)
end

figure
plot(shoulder_flexion, 'r', 'LineWidth', 2)
hold on
plot(shoulder_adduction, 'g', 'LineWidth', 2)
plot(shoulder_internal_rotation, 'b', 'LineWidth', 2)
plot(elbow_flexion, 'k', 'LineWidth', 2)
legend('shoulder flexion', 'shoulder adduction', 'shoulder internal rotation', 'elbow flexion')
legend('boxoff')
xlabel('Time Step')
ylabel('Joint angles (degrees)')
return
