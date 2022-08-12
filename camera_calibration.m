% Clearing the window and workspace
clc;
clear;


% Intrinsic parameters: focal length, image center, scale factors
% Extrinsic parameters: pose (rotation and translation)

% World coordinates
x_w_1 = 6; y_w_1 = 0; z_w_1 = 4;
x_w_2 = 6; y_w_2 = 0; z_w_2 = 6;
x_w_3 = 4; y_w_3 = 0; z_w_3 = 6;
x_w_4 = 0; y_w_4 = 6; z_w_4 = 4;
x_w_5 = 0; y_w_5 = 6; z_w_5 = 6;
x_w_6 = 0; y_w_6 = 4; z_w_6 = 6;
% Corresponding image coordinates
x_i_1 = 796; y_i_1 = 814;
x_i_2 = 796; y_i_2 = 748;
x_i_3 = 844; y_i_3 = 748;
x_i_4 = 1080; y_i_4 = 811;
x_i_5 = 1080; y_i_5 = 745;
x_i_6 = 1033; y_i_6 = 745;

% A matrix (Known values)
coeff_matrix = [x_w_1, y_w_1, z_w_1, 1 , 0, 0, 0, 0, -(x_i_1)*(x_w_1), -(x_i_1)*(y_w_1), -(x_i_1)*(z_w_1);
    0, 0, 0, 0, x_w_1, y_w_1, z_w_1, 1 , -(y_i_1)*(x_w_1), -(y_i_1)*(y_w_1), -(y_i_1)*(z_w_1);
    x_w_2, y_w_2, z_w_2, 1 , 0, 0, 0, 0, -(x_i_2)*(x_w_2), -(x_i_2)*(y_w_2), -(x_i_2)*(z_w_2);
    0, 0, 0, 0, x_w_2, y_w_2, z_w_2, 1 , -(y_i_2)*(x_w_2), -(y_i_2)*(y_w_2), -(y_i_2)*(z_w_2);
    x_w_3, y_w_3, z_w_3, 1 , 0, 0, 0, 0, -(x_i_3)*(x_w_3), -(x_i_3)*(y_w_3), -(x_i_3)*(z_w_3);
    0, 0, 0, 0, x_w_3, y_w_3, z_w_3, 1 , -(y_i_3)*(x_w_3), -(y_i_3)*(y_w_3), -(y_i_3)*(z_w_3);
    x_w_4, y_w_4, z_w_4, 1 , 0, 0, 0, 0, -(x_i_4)*(x_w_4), -(x_i_4)*(y_w_4), -(x_i_4)*(z_w_4);
    0, 0, 0, 0, x_w_4, y_w_4, z_w_4, 1 , -(y_i_4)*(x_w_4), -(y_i_4)*(y_w_4), -(y_i_4)*(z_w_4);
    x_w_5, y_w_5, z_w_5, 1 , 0, 0, 0, 0, -(x_i_5)*(x_w_5), -(x_i_5)*(y_w_5), -(x_i_5)*(z_w_5);
    0, 0, 0, 0, x_w_5, y_w_5, z_w_5, 1 , -(y_i_5)*(x_w_5), -(y_i_5)*(y_w_5), -(y_i_5)*(z_w_5);
    x_w_6, y_w_6, z_w_6, 1 , 0, 0, 0, 0, -(x_i_6)*(x_w_6), -(x_i_6)*(y_w_6), -(x_i_6)*(z_w_6);
    0, 0, 0, 0, x_w_6, y_w_6, z_w_6, 1 , -(y_i_6)*(x_w_6), -(y_i_6)*(y_w_6), -(y_i_6)*(z_w_6)];

b = [x_i_1; y_i_1; x_i_2; y_i_2; x_i_3; y_i_3; x_i_4; y_i_4; x_i_5; y_i_5; x_i_6; y_i_6];

% Getting the pseudo-invese of A
coeff_matrix_plus = (inv((coeff_matrix')*coeff_matrix))*(coeff_matrix');
p = (coeff_matrix_plus)*b;

projection_matrix = [0 0 0 0; 0 0 0 0; 0 0 0 1];
i = 1;
j = 1;
for count = 1:11
    if (count == 5) || (count == 9)
        i = i + 1;
        j = 1;
    end
    projection_matrix(i, j) = p(count);
    j = j + 1;
end

% Computing the norm of r3 and dividing the projection matrix by the norm
norm = sqrt((projection_matrix(3, 1)^2) + (projection_matrix(3, 2)^2) + (projection_matrix(3, 3)^2));
projection_matrix = projection_matrix / norm;

% Finding K
B = projection_matrix(:, 1:3);
A = B*(B');

if A(3, 3) ~= 1
    A = A / A(3, 3);
end

u0 = A(1, 3);
v0 = A(2, 3);
beta = sqrt(A(2, 2) - (v0)^2);
s = (A(1, 2) - ((u0)*(v0))) / beta;
alpha = sqrt(A(1, 1) - (u0)^2 - (s)^2);
K = [alpha, s, u0; 0, beta, v0; 0, 0, 1]

% Finding R (rotation) matrix
R = K \ B

% Finding t (translation) vector
t = K \ projection_matrix(:, 4)






