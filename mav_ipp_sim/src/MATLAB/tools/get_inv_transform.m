function [T_inv] = get_inv_transform(T)
% Calculate inverse of a homogeneous transform matrix.
% http://mathematica.stackexchange.com/questions/106257/how-do-i-get-the-inverse-of-a-homogeneous-transformation-matrix

rot = tform2rotm(T);
trans = tform2trvec(T);

rot_inv = rot';
trans_inv = -rot'*trans';

T_inv = [rot_inv, trans_inv; 0, 0, 0, 1];

end

