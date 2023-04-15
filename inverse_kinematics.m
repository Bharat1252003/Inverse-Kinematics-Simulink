function [shoulder_flexion, shoulder_adduction, shoulder_internal_rotation, elbow_flexion, misc_items] = inverse_kinematics(end_point, twisting_angle, upper_arm_len, fore_arm_len, which_hand)
% This function calculate the inverse kinematics to map 3D hand endpoint
% position to appropriate arm postures that involves 4D joint angles, a
% mission impossible that was made possible
% 
% Input: upper arm length, forearm length, hand 3D position, which hand,
% twisting angle in degrees (how much the elbow twist/swing between
% pointing down to pointing to the right/left)
%
% Output: the four joint angles, as well as the intermediate vectors
% used in calculation
%
% Note that we assumes that the shoulder is at orgin (0,0,0)
% X, Y, Z are Right, Forward, and Up
%
% WW/PM&R/Pitt  10/20/2010
% Last Update   06/24/2011 (This update replaces asin with atan2 functions)

pointing_vector = end_point;
pointing_vector_len = norm(pointing_vector);

% Find a vector that is perpendicular to the vertical plane that cut through the shoulder-hand vector
global_z = [0 0 1];
perp_vector1 = normalize3d(cross(pointing_vector, global_z));

% Find a vector that is perpendicular to pointing_vector and the
% perp_vector, and it should be in the vertical plane, as well
perp_vector2 = normalize3d(cross(perp_vector1, pointing_vector));

% We want to find a thrid vector that is in parallel to the twisting/swing
% of the elbow
temp_vector1 = perp_vector1*sin(deg2rad(twisting_angle));
temp_vector2 = -perp_vector2*cos(deg2rad(twisting_angle));
if 1 %(lower(which_hand(1))=='l') % Mirror for left hand
    temp_vector1 = -temp_vector1;
end
twist_vector = normalize3d(temp_vector1 + temp_vector2);

% Now, the twist_vector and the pointing vector form a plane that passes
% through the shoulder, hand, and elbow.

% Now figure out the three angles formed by the arm and pointing_vector
a = upper_arm_len;
b = fore_arm_len;
c = pointing_vector_len;
if c>=(a+b)
    error('not possible arm')
end

% NOTE: elbow angle is the angle between upper and fore arms inside the
% triangle formed by the upper and fore arms.  If you are going to use it
% in a conventional rotational matrix, you need to do 180-elbow_angle to
% get the right elbow flexion in your system (e.g. in Unity3D game engine).
% And this is by WW/PMR/Pitt  03/14/2011
elbow_angle = acos((a^2+b^2-c^2)/(2*a*b));
shoulder_angle = acos((a^2+c^2-b^2)/(2*a*c));
% wrist_angle = acos((b^2+c^2-a^2)/(2*b*c));

% Now find out the actual vector that points from the "mid-point of the
% shoulder-hand vector to the elbow, which gives us the elbow 3D location.
elbow_vector_len = upper_arm_len*sin(shoulder_angle);
mid_point_len = upper_arm_len*cos(shoulder_angle);
mid_point_vector = normalize3d(pointing_vector)*mid_point_len;
elbow_vector = twist_vector*elbow_vector_len;
elbow_point = elbow_vector + mid_point_vector;

% Now find out the three vectors defines the bone rigid body reference
% frame attached to the humerus/upper arm.
% In a neutral position, the humerus is with the arm down and attached to
% the side of the body with the palm facing the side of the leg.  This is
% slightly different from anatomical neutral position.
upper_arm_z_vector = normalize3d(-elbow_point);
upper_arm_x_vector = normalize3d(cross(pointing_vector, upper_arm_z_vector));
upper_arm_y_vector = normalize3d(cross(upper_arm_z_vector, upper_arm_x_vector));

% Now we have the bone axis of the humerus, and we can calculate the joint
% angles
% This part of the code is based on the original code from Dr. Daniel W.
% Moran at Washington University in St. Louis.
%   This function assumes the following relationship
%   |--Bone Axis n+1 --|   |-- Bone Axis n  --|  |--          --|  
%   | Bone  Bone  Bone |   | Bone  Bone  Bone |  |              |  
%   | Axes  Axes  Axes | = | Axes  Axes  Axes |  |  Rot_matrix  |  
%   |   1     2     3  |   |   1     2     3  |  |              |  
%   |--              --|   |--              --|  |--          --|  
bone_axes = [upper_arm_x_vector' upper_arm_y_vector' upper_arm_z_vector'];
%rot_mat = eye(3) \ bone_axes;
rot_mat = bone_axes;

% This is based on: http://en.wikipedia.org/wiki/Rotation_matrix
% That webpage was also saved in the Reference folder inside the folder
% containing this script
% SHOULDER ADDUCTION
% Note: For shoulder adduction, we made a strong assumption that it
% typically operate in the 0 to 90 degrees range and never really gets to
% 90 degrees. This allows us to safely use the asin function without
% causing any ambiguity in shoulder adduction angles. This assumption also
% makes the calculation of the other two shoulder angles with atan2 much
% easier to do.
% WW/PMR/Pitt  06/24/2011
shoulder_adduction = asin(-rot_mat(3,1));
% SHOULDER FLEXION
% Note: the atan2 implementation for shoulder_flexion is critical because
% it returns to full -180 to +180 range, while the previous implementation
% used asin, which only returns -90 to +90 range and creates ambiguity in
% shoulder flexion angle.
% Obsolete: shoulder_flexion = asin(rot_mat(3,2)/(cos(shoulder_adduction)));
% Please also note that this again relies on the assumption that
% cos(shoulder_adduction) is never 0 or the shoulder_adduction angle will
% not be 90 degrees. (0 degree is for the neutral position where the arm is
% on the side of the body)
% WW/PMR/Pitt  06/24/2011
shoulder_flexion = atan2(rot_mat(3,2), rot_mat(3,3));
% SHOULDER INTERNAL ROTATION
% Note: This angle is now also calculated using the atan2 function to avoid
% ambiguity, and again we assumes that the shoulder adduction angle is not
% going to be 90 degrees.
% Obsolete: shoulder_internal_rotation = asin(rot_mat(2,1)/(cos(shoulder_adduction)));
% WW/PMR/Pitt  06/24/2011
shoulder_internal_rotation = atan2(rot_mat(2,1), rot_mat(1,1));
elbow_flexion = elbow_angle;

% Convert to degrees
shoulder_internal_rotation = rad2deg(shoulder_internal_rotation);
shoulder_adduction = rad2deg(shoulder_adduction);
shoulder_flexion = rad2deg(shoulder_flexion);
elbow_flexion = rad2deg(elbow_flexion);

% Pack other intermidate calculation output in the structure
misc_items.elbow_point = elbow_point;
misc_items.perp_vector1 = perp_vector1;
misc_items.perp_vector2 = perp_vector2;
misc_items.twist_vector =  twist_vector;
misc_items.upper_arm_x_vector = upper_arm_x_vector;
misc_items.upper_arm_y_vector = upper_arm_y_vector;
misc_items.upper_arm_z_vector = upper_arm_z_vector;
misc_items.rot_mat = rot_mat;

return

function out = normalize3d(in)
out = in/norm(in);
return