function vecres = quatrotate(q,vec)
%QUATROTATE rotate a given vector using a quaternion to provide a rotated point
sizevec = size(vec,1);%Number of rows
qvec = [zeros(sizevec,1) vec];
vecres = quatmultiply(q, quatmultiply(qvec, quatinv(q)));
vecres(:,1) = [];
end
