function reconstruction3D()
% 3D reconstruction from 3 views of depth cameras
%
% compute the camera poses using the plane intersections
% discard distant points to the camera
load('intersection_points');
max_dist = 3000;
ind = (sqrt(pts1(:,1).^2 + pts1(:,2).^2 + pts1(:,3).^2)<=max_dist) & ...
    (sqrt(pts2(:,1).^2 + pts2(:,2).^2 + pts2(:,3).^2)<=max_dist) & ...
    (sqrt(pts3(:,1).^2 + pts3(:,2).^2 + pts3(:,3).^2)<=max_dist);

[R21,t21] = extrinsic_parameters(pts1(ind,:),pts2(ind,:));
[R31,t31] = extrinsic_parameters(pts1(ind,:),pts3(ind,:));

% read the images and compute the clouds of points
img1 = double(imread('pablo/cam1_img002.png'));
img2 = double(imread('pablo/cam2_img002.png'));
img3 = double(imread('pablo/cam3_img002.png'));
pts1 = points3d(img1);
pts2 = points3d(img2);
pts3 = points3d(img3);
n = size(pts1,1);

% aling the points to a common reference system 
% and join the points from the three views
ptres = pts1;
P = R21*pts2' + repmat(t21,1,n);
ptres = [ptres; P'];
P = R31*pts3' + repmat(t31,1,n);
ptres = [ptres; P'];
x = ptres(:,1);
y = ptres(:,2);
z = ptres(:,3);

% set a bounding box of interest and plot the result
ind = (x>=-500) & (x<=500) & (z>=800) & (z<=1600);
x = x(ind);
y = y(ind);
z = z(ind);
figure(1); plot3(z,x,-y,'b.','markersize',1); grid on; axis('equal');
xlabel('Z (mm)');  ylabel('X (mm)');  zlabel('Y (mm)');

% compute the depth values for a grid of points applying a median filter
dm = 10;
[xm,ym] = meshgrid(-300:dm:400,-400:dm:500);
[rs,cs] = size(xm);
zm = zeros(rs,cs);
for i=1:rs
    for j=1:cs
        ind = (x>=xm(i,j)-dm) & (x<=xm(i,j)+dm) & (y>=ym(i,j)-dm) & (y<=ym(i,j)+dm);
        if( sum(ind)>0 )
            zm(i,j) = median(z(ind));
        end
    end
end
% plot the result
zm(zm(:)==0) = NaN;
figure(2); surf(zm, xm, -ym); axis('equal');
xlabel('Z (mm)');  ylabel('X (mm)');  zlabel('Y (mm)');

%====================================================
function [R,t] = extrinsic_parameters(pts1,pts2)
% compute the camera pose using minimal squares
n = size(pts1,1);
M = [pts2, zeros(n,6), ones(n,1), zeros(n,2); ...
    zeros(n,3), pts2, zeros(n,4), ones(n,1), zeros(n,1); ...
    zeros(n,6), pts2, zeros(n,2), ones(n,1)];
N = [pts1(:,1); pts1(:,2); pts1(:,3)];
s = (M'*M)\(M'*N);
Q = reshape(s(1:9),3,3)';
[U,S,V] = svd(Q);
R = U*V';
P = R*pts2';
t = mean(pts1-P')';

%====================================================
function pts = points3d(img)
% compute the 3D points from a depth image
fov = 58.5;  %horizontal field of view of the kinect for Xbox 360 depth sensor
cols = 640;
rows = 480;
f = 0.5*cols/tand(0.5*fov);  %focal length
cc = 0.5*[cols rows];  %center of the image
[um,vm] = meshgrid(1:cols,1:rows);
Z = img(:);
X = Z.*(um(:)-cc(1))/f;
Y = Z.*(vm(:)-cc(2))/f;
pts = [X, Y, Z];