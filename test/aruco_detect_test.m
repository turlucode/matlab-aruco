%% Test 1
img_00 = imread('aruco_test_img_00.jpg');

[imgOut_00] = aruco_detect(img_00, 'DICT_ARUCO_ORIGINAL');

figure
imshow(imgOut_00)

%% Test 2

img_01 = imread('aruco_test_img_01.jpg');
load('cameraParams.mat')

cameraMatrix = cameraParams.IntrinsicMatrix';
distCoeffs = [cameraParams.RadialDistortion(1)
              cameraParams.RadialDistortion(2)
              cameraParams.TangentialDistortion(1)
              cameraParams.TangentialDistortion(2)
              cameraParams.RadialDistortion(3)]';
markerLength = 0.1695; % unit can be anything. return values will be in that unit. (here is meters)
          
[imgOut_01, markers] = aruco_detect(img_01, 'DICT_ARUCO_ORIGINAL', cameraMatrix, distCoeffs, markerLength);

fprintf('Number of markers detected: %d\n', size(markers,2));
for i=1:size(markers,2)
    markers(i).R
    markers(i).t
end

figure
imshow(imgOut_01)