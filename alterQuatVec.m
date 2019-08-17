function alteredVec = alterQuatVec(vec, noiseVec,angleNoise)
% ALTERQUATVEC is a function that alters a given vector vec, using a certain 
% noise vector. The first four components are assumed to represent a 
% quaternion q. q is given random noise, the amplitude of this noise is 
% determinded by input parameter angleNoise. The other components of the 
% given vector are altered using the corresponding components of
% inputparameter noiseVec.
    dangles = sqrt(2) * randn(3,1) / angleNoise; % Noise on Euler angles
    dquat = deg2quat(dangles(1), dangles(2), dangles(3)); % Convert angle noise to quaternion noise
    q = quatmultiply(vec(1:4)', dquat); % Add noise on the quaternion 
    alteredVec(1:4) = quatnormalize(q)'; % Normalize quaternion
    alteredVec(5:size(vec, 1)) = noiseVec(5:size(vec, 1)) + vec(5:size(vec, 1));
end
