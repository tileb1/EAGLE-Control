function q = deg2quat(yaw, pitch, roll)
% DEG2QUAT is a function that transforms a set of yaw-pitch-roll euler 
% angels to a normalised quaternion. The input euler angles are assumed to
% be expressed in degrees
%
% Syntax
% q = deg2quat(yaw, pitch, roll)
%
% Input
% yaw      yaw euler angle
% pitch    pitch euler angle
% roll     roll euler angle
q = quatnormalize(angle2quat(deg2rad(yaw),deg2rad(pitch),deg2rad(roll)));