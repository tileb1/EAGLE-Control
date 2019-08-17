function diff = quatDiff(x, y)
% QUATDIFF is a function that computes the difference between two vectors.
% The first four components are assumed to represent quaternions. The
% quaternion difference is calculated using the hamilton product.
    qError = quatnormalize(quatmultiply(x(1:4)',quatconj(y(1:4)')));
    diff = [qError(1:4)'; x(5:end)-y(5:end)];
end