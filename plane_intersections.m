function [Xp,Yp,Zp] = plane_intersections()
%
% this script finds the intersections of a plane placed at different
% orientations from images captured by the kinect depth sensor
ncam = 1;  %camera number
fov = 58.5;  %horizontal field of view of the kinect for Xbox 360 depth sensor
cols = 640;
rows = 480;
f = 0.5*cols/tand(0.5*fov);  %focal length
cc = 0.5*[cols rows];  %center of the image
[um,vm] = meshgrid(1:cols,1:rows);

fittol = 5;  %tolerance to find the plane in mm
pars = zeros(10,4);  % plane parameters
for nimg=0:9
    %read and show an image
    fn = sprintf('plano/cam%d_img%03d.png',ncam,nimg);
    im = double(imread(fn));
    imagesc(im);
    %select points to fit a plane
    fprintf('draw a polygon covering most of the calibration plane ...');
    imb = roipoly();
    ind = find((imb==1) & (im>0));
    Z = im(ind);
    X = Z.*(um(ind)-cc(1))/f;
    Y = Z.*(vm(ind)-cc(2))/f;
    %find the plane parameters
    pars(nimg+1,:) = fit_plane(X,Y,Z,fittol);
    fprintf('  image %d of 10 -> ok\n', nimg+1);
end

%find all the intersections
[Xp,Yp,Zp] = find_intersections(pars);
plot3(Xp,Yp,Zp,'bo'); grid on;
set(gca,'fontsize',14);
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

%================================================
function pars = fit_plane(X,Y,Z,fittol)
%fit a plane from the 3D points using
%the M-estimator SAmple Consensus(MSAC) algorithm
ptCloud = pointCloud([X Y Z]);
model = pcfitplane(ptCloud, fittol);
pars = model.Parameters;

%=========================================================
function [Xp,Yp,Zp] = find_intersections(pars)
%find all intersections
Xp = [];
Yp = [];
Zp = [];
n = size(pars,1);
for i=1:n-2
    for j=i+1:n-1
        for k=j+1:n
            %find the intersection of the plane in three different
            %orientations
            M = pars([i; j; k],1:3);
            N = -pars([i; j; k],4);
            s = M\N;
            Xp = [Xp; s(1)];
            Yp = [Yp; s(2)];
            Zp = [Zp; s(3)];
        end
    end
end