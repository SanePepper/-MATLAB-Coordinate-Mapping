% Created on 29-May-2019 with reference to sample code provided
%-------------------------------------------------------

% Define images to process
imageFileNames = {'C:\Users\pwc02\Downloads\image184-ok\img184-000 (2).bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-000.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-001.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-003.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-004.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-005.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-006.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-007.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-008.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-009.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-035.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-042.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-048.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-055.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-061.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-063.bmp',...
    'C:\Users\pwc02\Downloads\image184-ok\img184-063 (2).bmp',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);
imageSize = mrows*ncols;

% Generate world coordinates of the corners of the squares
squareSize = 38.5;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera using fisheye parameters
[cameraParams, imagesUsed, estimationErrors] = estimateFisheyeParameters(imagePoints, worldPoints, ...
    [mrows, ncols], ...
    'EstimateAlignment', false, ...
    'WorldUnits', 'millimeters');

% View reprojection errors
%h1=figure; showReprojectionErrors(cameraParams);
% Visualize pattern locations
%h2=figure; 
%showExtrinsics(cameraParams, 'CameraCentric');
% Display parameter estimation errors
%displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
[undistortedImage,Intrinsics] = undistortFisheyeImage(originalImage, cameraParams.Intrinsics);
figure
imshow(undistortedImage);
title("Undistorted Image");

wholeImagePoints = zeros(imageSize,2,"uint8");
for i = 1:imageSize
   wholeImagePoints(i,1:2) = [mod((i-1),ncols)+1,floor((i-1)/ncols)+1];
end
undistortedPoints = undistortFisheyePoints(wholeImagePoints,cameraParams.Intrinsics);
min1 = min(undistortedPoints(:,1)); 
min2 = min(undistortedPoints(:,2)); 
max1 = max(undistortedPoints(:,1)); 
max2 = max(undistortedPoints(:,2));
undistortedProjection = zeros(int32(max1-min1), int32(max2-min2));
for i = 1:imageSize
   undistortedProjection(int32(undistortedPoints(i,1))-min1,int32(undistortedPoints(i,2))-min2) = originalImage(floor((i-1)/ncols)+1, mod((i-1),ncols)+1);
   %if undistortedPoints(i,1) >= 1 && undistortedPoints(i,1) >= ncols && undistortedPoints(i,2) >= 1 && undistortedPoints(i,2) <= mrows
   %    undistortedProjection(int32(undistortedPoints(i,2)),int32(undistortedPoints(i,1))) = originalImage(floor((i-1)/ncols)+1, mod((i-1),ncols)+1);
   %end
end
figure
imshow(undistortedProjection);
title("mapping from original image to undistorted image")
originHeight = 0;
[pitch, yaw, roll, height] = estimateMonoCameraParameters(cameraParams.Intrinsics,imagePoints(:,:,1), worldPoints, originHeight);
%sensor = monoCamera(Intrinsics,height,'pitch',pitch,'yaw',yaw,'roll',roll);
sensor = monoCamera(Intrinsics,19.8,'pitch',pitch,'yaw',0,'roll',0);

distAhead = 150;
spaceToOneSide = 100;
bottomOffset = 0;
outView = [bottomOffset,distAhead,-spaceToOneSide,spaceToOneSide];
outImageSize = [NaN,mrows];
birdsEye = birdsEyeView(sensor,outView,outImageSize);
BEV = transformImage(birdsEye,undistortedImage);
figure
imshow(BEV)
title('Bird''s-Eye-View Image')

wholeWorldPoints = imageToVehicle(sensor,single(wholeImagePoints));
wholeWorldPoints = int32(wholeWorldPoints);
resultWorldIMG = zeros(200,500,"uint8");
for i = 1:imageSize
    if wholeWorldPoints(i,1)>=1 && wholeWorldPoints(i,1)<=200 && wholeWorldPoints(i,2)>-250 && wholeWorldPoints(i,2)<=250
        resultWorldIMG(200-wholeWorldPoints(i,1)+1,wholeWorldPoints(i,2)+250) = undistortedImage(floor((i-1)/ncols)+1, mod((i-1),ncols)+1);
    end
end 
figure
imshow(resultWorldIMG)
title("Mapping from undistorted image");

result = zeros(200, 500, "uint8");
worldMapping = zeros(imageSize, 2, "single");
for i = 1:imageSize
    if undistortedPoints(i,1) >= 1 && undistortedPoints(i,1) <= ncols && undistortedPoints(i,2) >= 1 && undistortedPoints(i,2) <= mrows
        worldMapping(i,1:2) = wholeWorldPoints(int32(undistortedPoints(i,1)+(undistortedPoints(i,2)-1)*ncols),1:2); 
    else
        worldMapping(i,1:2) = NaN(1,2);
    end
end
for i = 1:imageSize
    if worldMapping(i,1)>=1 && worldMapping(i,1)<=200 && worldMapping(i,2)>-250 && worldMapping(i,2)<=250
        result(201-int32(worldMapping(i,1)), int32(worldMapping(i,2))+250) = originalImage(floor((i-1)/ncols)+1, mod((i-1),ncols)+1);
    end
end
figure
imshow(result)
title("result of coordinate mapping");

fileID = fopen('C:\Users\pwc02\Desktop\ImageTest.txt','w');
fprintf(fileID,'{');
fprintf(fileID,'{%d,%d},',int32(worldMapping));
fprintf(fileID,'}');
fclose(fileID);
