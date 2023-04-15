function [joint_angles] = calc_7DOF_ang8 (arm, num_tot_bins, baseline, marker_data)
% Note: this function is not really used.  But it is here for good
% reference.  It is done in Dr. Daniel W. Moran's lab at Washington
% University in St. Louis.
%
% Calculates joint angles based on marker positions.
% Based off calc_7DOF_ang6.c by Dan Moran
% 1. Inputs: Arm name (arm) = 'L' or 'R'
%            Number of bins of data (num_tot_bins)
%            Markers' baseline positions (Baseline) = num_markers x 3
%            Marker data (marker_data) = num_markers x 3 x num_tot_bins
%
% 2. Calculates initial bone axes, transforms marker data to baseline,
% calculates the marker axes, calculates the marker to bone transform, then
% finally calculates the cardanic bone angles.
%
% 3. Outputs: Joint angles = DOF x num_tot_bins
%
% Coordinate system for the Optotrak marker data is
%     X: Backwards
%     Y: Down
%     Z: Right
%
% Coordinate system for the arm is 
%   1. Segment number
%   2. Bone Axes: X(fwd), Y(up) or Z(right for right arm)
%   3. Coordinates: X(fwd), Y(up) or Z(right for right arm)
% 
% Rotation directions for the arm are as follows:
%   1. Shoulder adduction (+x)
%   2. Shoulder internal rotation (+y)
%   3. Shoulder flexion (+z)
%   4. Elbow flexion (+z)
%   5. Pronation (+y)
%   6. Wrist flexion (+x)
%   7. Wrist abduction (+z)
% 
% Usage: [joint_angles] = calc_7DOF_ang8 (arm, num_tot_bins, baseline, marker_data)
%
% Variable names and sizes
% baseline: num_markers x 3
% marker_data: num_markers x 3 x num_tot_bins
% joint_angles: DOF x num_tot_bins
% Angles: DOF x num_tot_bins
% Marker_pos: num_markers x 3
% Marker_axes: num_segments x 3 x 3
% Initial_axes = num_segments x 3 x 3
% TransMatrix: num_segments x 3 x 3
% Bone_axes: num_segments x 3 x 3
%
% Created by Sherwin Chan
% Date: 2/19/2004
% Last modified : 4/14/2004 SSC
% Revision History
%   5/18/04 SSC
%       -Changed how angles are calculated so things are consistent.
%   6/9/04 SSC (v 1.0.1)
%       -Rotated axes so that they are vertical.
%       -Added functions to try to interpolate missing points (only handles
%           1)

num_markers = 8;
num_segments = 4;
RADtoDEG = 180/pi;
alpha = 0.0977;
beta = 0.0977;

%--------------------------------------------------------------------------
%Initial bone axes are set up to be in perfect alignment such that the
% elbow is flexed 90 degrees and the forearm is pronated so that the thumb
% points straight up.  This is termed the modified anatomical position. The 
% convention is that axes are represented as columns.
% |-- Initial Axes --|
% | Axes  Axes  Axes |
% |   1     2     3  |
% |                  |
% |--              --|

Initial_axes(1,:,:) = [1 0 0; 0 1 0; 0 0 1];
for i = 2:4
    Initial_axes(i,:,:) = [0 -1 0; 1 0 0; 0 0 1];
end

if (arm == 'R')
    % Transform between coordinate systems
    Marker_pos(:,1) = -baseline(:,1);
    Marker_pos(:,2) = -baseline(:,2);
    Marker_pos(:,3) =  baseline(:,3);
elseif (arm =='L')
    Marker_pos(:,1) =  baseline(:,1);
    Marker_pos(:,2) = -baseline(:,2);
    Marker_pos(:,3) = -baseline(:,3);
else
    disp(['ERROR: Letter ', arm, ' does not correspond to (R)ight or (L)eft arm.']);
    return;
end

% Calculate the marker-bone axes based upon marker positions
Marker_axes = calc_marker_axes(Marker_pos);

% --------------------------------------------------------------------------
% Calculates the transformation matrix between the marker coordinate system
% and the bone coordinate system [bone] = [trans][marker]
%
%   |--     Bone     --|   |--    Marker    --| |--          --|
%   | Bone  Bone  Bone |   | Axes  Axes  Axes | |              |  
%   | Axes  Axes  Axes | = |   1     2     3  | | Trans_matrix | 
%   |   1     2     3  |   |                  | |              |  
%   |--              --|   |--              --| |--          --|  
%
% Note: Initial_axes = Trans_matrix * Marker_axes

% for i = 1 : num_segments
%     Trans_matrix(i,:,:) = squeeze(Initial_axes(i,:,:)) / squeeze(Marker_axes(i,:,:));
% end

for i = 1 : num_segments
    Trans_matrix(i,:,:) = squeeze(Marker_axes(i,:,:)) \ squeeze(Initial_axes(i,:,:));
end

% Use the transformation matrix between marker and bones to determine bone vectors.
% for i = 1 : num_segments
%     Bone_axes(i,:,:) =  squeeze(Trans_matrix(i,:,:)) * squeeze(Marker_axes(i,:,:));
% end

for i = 1 : num_segments
    Bone_axes(i,:,:) =  squeeze(Marker_axes(i,:,:)) * squeeze(Trans_matrix(i,:,:));
end

alpha_beta = [alpha beta];

% Do a sample calculation to check that everything is correct
Angles = calc_cardanic_angles (Bone_axes);
% Print out the results of the calculations to the screen
% sAngle = 'Angles are:   ';
% for i = 1 : size(Angles, 2);
%     sAngle = [sAngle, sprintf('  %5.1f   ', Angles(i)*RADtoDEG)];
% end
% disp(sAngle);


for i = 1 : num_tot_bins
    if (arm == 'R')
        Marker_pos(:,1) = -marker_data(:,1,i);
        Marker_pos(:,2) = -marker_data(:,2,i);
        Marker_pos(:,3) =  marker_data(:,3,i);
    elseif (arm =='L')
        Marker_pos(:,1) =  marker_data(:,1,i);
        Marker_pos(:,2) = -marker_data(:,2,i);
        Marker_pos(:,3) = -marker_data(:,3,i);
    else
        disp(['ERROR: Letter ', arm, ' does not correspond to (R)ight or (L)eft arm.']);
        return;
    end
    
    if (sum(sum(isnan(Marker_pos))) == 0)
        % Calculate marker axes from marker positions
        Marker_axes = calc_marker_axes(Marker_pos);
        % Use transformation to go from marker axes to bone axes
%         for j = 1 : num_segments
%             Bone_axes1(j,:,:) =  squeeze(Trans_matrix1(j,:,:)) * squeeze(Marker_axes(j,:,:));
%         end
        for j = 1 : num_segments
            Bone_axes(j,:,:) =  squeeze(Marker_axes(j,:,:)) * squeeze(Trans_matrix(j,:,:));
        end
        % Calculate the bone angles
        Angles = calc_cardanic_angles (Bone_axes);
        % Assign the bone angles to output variable
        joint_angles(:,i) = Angles' * RADtoDEG;
    else
        joint_angles(:,i) = NaN*ones(7,1);
    end
    
%     % Print out the results of the calculations to the screen
%     sAngle = ['Bin #', sprintf('%5.1f', i), '  Angles are:   '];
%     for i = 1 : size(Angles, 2);
%         sAngle = [sAngle, sprintf('  %5.1f   ', Angles(i)*RADtoDEG)];
%     end
%     disp(sAngle);
end

% If any of the markers were missing for one time period, then interpolate
% for that time period.
for i = 1 : num_tot_bins
    if (isnan(joint_angles(1,i)))
        if i == 1
            joint_angles(:,i) = joint_angles(:,i+1);
        elseif i == num_tot_bins
            joint_angles(:,i) = joint_angles(:,i-1);
        else
            joint_angles(:,i) = mean([joint_angles(:,i-1) joint_angles(:,i+1)], 2);
        end
    end
end
return

function marker_axes = calc_marker_axes (marker_pos)

% Humerus Y points from elbow to shoulder
marker_axes(1,:,2) = marker_pos(8,:) - marker_pos(7,:);
marker_axes(1,:,2) = marker_axes(1,:,2) / norm(squeeze(marker_axes(1,:,2)));

% Ulna Y points from wrist to elbow
marker_axes(2,:,2) = marker_pos(6,:) - marker_pos(5,:);
marker_axes(2,:,2) = marker_axes(2,:,2) / norm(squeeze(marker_axes(2,:,2)));

% Humerus Z is (Ulna Y x Humerus Y)
marker_axes(1,:,3) = cross(marker_axes(2,:,2), marker_axes(1,:,2));
marker_axes(1,:,3) = marker_axes(1,:,3) / norm(squeeze(marker_axes(1,:,3)));

% Humerus X is (Humerus Y x Humerus Z)
marker_axes(1,:,1) = cross(marker_axes(1,:,2), marker_axes(1,:,3));
marker_axes(1,:,1) = marker_axes(1,:,1) / norm(squeeze(marker_axes(1,:,1)));

% Ulna Z is equal to Humerus Z
marker_axes(2,:,3) = marker_axes(1,:,3);
marker_axes(2,:,3) = marker_axes(2,:,3) / norm(squeeze(marker_axes(2,:,3)));

% Ulna X is (Ulna Y x Ulna Z)
marker_axes(2,:,1) = cross(marker_axes(2,:,2), marker_axes(2,:,3));
marker_axes(2,:,1) = marker_axes(2,:,1) / norm(squeeze(marker_axes(2,:,1)));

% Radius X points from the pinky side of the wrist to the thumb side
marker_axes(3,:,1) = marker_pos(4,:) - marker_pos(5,:);
marker_axes(3,:,1) = marker_axes(3,:,1) / norm(squeeze(marker_axes(3,:,1)));

% Radius Z is approx (Radius X x Ulna Y)
marker_axes(3,:,3) = cross(marker_axes(3,:,1), marker_axes(2,:,2));
marker_axes(3,:,3) = marker_axes(3,:,3) / norm(squeeze(marker_axes(3,:,3)));

% Radius Y is (Radius Z x Radius X)
marker_axes(3,:,2) = cross(marker_axes(3,:,3), marker_axes(3,:,1));
marker_axes(3,:,2) = marker_axes(3,:,2) / norm(squeeze(marker_axes(3,:,2)));

% Temporary Hand X points from the pinky to the thumb
marker_axes(4,:,1) = marker_pos(2,:) - marker_pos(3,:);
marker_axes(4,:,1) = marker_axes(4,:,1) / norm(squeeze(marker_axes(4,:,1)));

% Hand Y points from the distal end of the hand to the proximal part of the hand.
marker_axes(4,:,2) = (marker_pos(2,:) + marker_pos(3,:))/2 - marker_pos(1,:);
marker_axes(4,:,2) = marker_axes(4,:,2) / norm(squeeze(marker_axes(4,:,2)));

% Hand Z is (Hand X x Hand Y)
marker_axes(4,:,3) = cross(marker_axes(4,:,1), marker_axes(4,:,2));
marker_axes(4,:,3) = marker_axes(4,:,3) / norm(squeeze(marker_axes(4,:,3)));

% Final Hand X points from the pinky to the thumb (Hand Y x Hand Z)
marker_axes(4,:,1) = cross(marker_axes(4,:,2), marker_axes(4,:,3));
marker_axes(4,:,1) = marker_axes(4,:,1) / norm(squeeze(marker_axes(4,:,1)));

return

function angles = calc_cardanic_angles(bone_axes)
%   This function assumes the following relationship
%   |--Bone Axis n+1 --|   |-- Bone Axis n  --|  |--          --|  
%   | Bone  Bone  Bone |   | Bone  Bone  Bone |  |              |  
%   | Axes  Axes  Axes | = | Axes  Axes  Axes |  |  Rot_matrix  |  
%   |   1     2     3  |   |   1     2     3  |  |              |  
%   |--              --|   |--              --|  |--          --|  
options = optimset('Display', 'off');
beta = .0977;           % angle beta in radians (5.6 degrees)

rot_mat = eye(3) \ squeeze(bone_axes(1,:,:));
angles(2) = asin(rot_mat(1,3));                           % shoulder internal rotation
angles(1) = asin(rot_mat(2,3)/(-cos(angles(2))));                           % shoulder adduction
angles(3) = asin(rot_mat(1,2)/(-cos(angles(2))));                           % shoulder flexion

% q2_init = asin(rot_mat(1,3));
% q1_init = asin(rot_mat(2,3)/(-cos(q2_init)));
% q3_init = asin(rot_mat(1,2)/(-cos(q2_init)));
% q123_init = [q1_init; q2_init; q3_init];
% ref_n_a = rot_mat;
% q123_soln = lsqnonlin(@calc_q123, q123_init, -pi, pi, options, ref_n_a);
% angles(2) = q123_soln(2);                           % shoulder internal rotation
% angles(1) = q123_soln(1);                           % shoulder adduction
% angles(3) = q123_soln(3);                           % shoulder flexion

rot_mat = squeeze(bone_axes(1,:,:)) \ squeeze(bone_axes(2,:,:));
angles(4) = atan2(-rot_mat(1,2), rot_mat(1,1));     % elbow flexion

rot_mat = squeeze(bone_axes(2,:,:)) \ squeeze(bone_axes(3,:,:));
angles(5) = asin(rot_mat(1,3)/cos(beta));           % supination

rot_mat = squeeze(bone_axes(3,:,:)) \ squeeze(bone_axes(4,:,:));
angles(6) = asin(-rot_mat(2,3));                    % wrist flexion
angles(7) = asin(-rot_mat(1,2));                    % wrist abduction

return

%Code from Michael Fine
%Last Edited: 4/15/2003

function[err_n_a] = calc_q123(q1_q2_q3, ref_n_a);

n_a = zeros(3,3);
q1 = q1_q2_q3(1);
q2 = q1_q2_q3(2);
q3 = q1_q2_q3(3);

n_a(1,1) = cos(q2)*cos(q3);
n_a(1,2) = -cos(q2)*sin(q3);
n_a(1,3) = sin(q2);
n_a(2,1) = sin(q1)*sin(q2)*cos(q3)+cos(q1)*sin(q3);
n_a(2,2) = -sin(q1)*sin(q2)*sin(q3)+cos(q1)*cos(q3);
n_a(2,3) = -sin(q1)*cos(q2);
n_a(3,1) = -cos(q1)*sin(q2)*cos(q3)+sin(q1)*sin(q3);
n_a(3,2) = cos(q1)*sin(q2)*sin(q3)+sin(q1)*cos(q3);
n_a(3,3) = cos(q1)*cos(q2);

calc_q123_s = n_a;
ref_n_a_s = ref_n_a;
err_n_a = (calc_q123_s - ref_n_a_s);

return