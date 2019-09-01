% Created on 29-May-2019 with reference to sample code provided
%-------------------------------------------------------
displayResult = true;
% Define images to process, used relative path to the .m file
% Feed in checkerboard images captured by the camera,
% the first image would be used as a reference for perspective fix
imageFileNames = {'image184-ok\img184-000 (2).bmp',...
    'image184-ok\img184-000.bmp',...
    'image184-ok\img184-001.bmp',...
    'image184-ok\img184-003.bmp',...
    'image184-ok\img184-004.bmp',...
    'image184-ok\img184-005.bmp',...
    'image184-ok\img184-006.bmp',...
    'image184-ok\img184-007.bmp',...
    'image184-ok\img184-008.bmp',...
    'image184-ok\img184-009.bmp',...
    'image184-ok\img184-035.bmp',...
    'image184-ok\img184-042.bmp',...
    'image184-ok\img184-048.bmp',...
    'image184-ok\img184-055.bmp',...
    'image184-ok\img184-061.bmp',...
    'image184-ok\img184-063.bmp',...
    };
squareSize = 38.5;  % actual size of each checkerboard grid, in units of 'millimeters'
%unit is not defined here, have to be congruent throughout the program
fieldImage = imread('.\checkerboard711\img-000.bmp');

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);
imageSize = mrows*ncols;

% Generate world coordinates of the corners of the squares
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera using fisheye parameters
[cameraParams, imagesUsed, estimationErrors] = estimateFisheyeParameters(imagePoints, worldPoints, ...
    [mrows, ncols], ...
    'EstimateAlignment', false, ...
    'WorldUnits', 'millimeters');

% Use the calibration data to remove effects of lens distortion.
[undistortedImage,Intrinsics] = undistortFisheyeImage(originalImage, cameraParams.Intrinsics);
wholeImagePoints = zeros(imageSize,2,"uint8");
for i = 1:imageSize
   wholeImagePoints(i,1:2) = [mod((i-1),ncols)+1,floor((i-1)/ncols)+1];
end
undistortedPoints = undistortFisheyePoints(wholeImagePoints,cameraParams.Intrinsics);
min1 = min(undistortedPoints(:,1)); 
min2 = min(undistortedPoints(:,2)); 
max1 = max(undistortedPoints(:,1)); 
max2 = max(undistortedPoints(:,2));
undistortedProjection = zeros(int32(max2-min2+1), int32(max1-min1+1), "uint8");
for i = 1:imageSize
   undistortedProjection(int32(undistortedPoints(i,2)-min2)+1,int32(undistortedPoints(i,1)-min1)+1) = originalImage(floor((i-1)/ncols)+1, mod((i-1),ncols)+1);
end
if (displayResult)
    figure
    undistortedProjection = insertMarker(undistortedProjection,[ncols-min1, mrows-min2]);
    undistortedProjection = insertMarker(undistortedProjection,[ncols-min1, 1-min2]);
    undistortedProjection = insertMarker(undistortedProjection,[1-min1, mrows-min2]);
    undistortedProjection = insertMarker(undistortedProjection,[1-min1 1-min2]);
    imshow(undistortedProjection);
    title("mapping from original image to undistorted image")
end
originHeight = 0;
[pitch, yaw, roll, height] = estimateMonoCameraParameters(cameraParams.Intrinsics,imagePoints(:,:,1), worldPoints, originHeight);
%Obtain the camera angle using the first image
%You can fine tune by entering constants rather than detected variables for
%roll, pitch and yaw
sensor = monoCamera(Intrinsics,height,'pitch',pitch,'yaw',yaw,'roll',roll);

distAhead = 1500;
spaceToOneSide = 1000;
bottomOffset = 0;
outView = [bottomOffset,distAhead,-spaceToOneSide,spaceToOneSide];
outImageSize = [NaN,mrows];
birdsEye = birdsEyeView(sensor,outView,outImageSize);
BEV = transformImage(birdsEye,undistortedImage);
if (displayResult)
    figure
    imshow(BEV)
    title('Bird''s-Eye-View Image')
end
wholeWorldPoints = imageToVehicle(sensor,single(wholeImagePoints));
%{
resultWorldIMG = zeros(200,500,"uint8");
for i = 1:imageSize
    if wholeWorldPoints(i,1)>=1 && wholeWorldPoints(i,1)<=200 && wholeWorldPoints(i,2)>-250 && wholeWorldPoints(i,2)<=250
        resultWorldIMG(200-int32(wholeWorldPoints(i,1))+1,int32(wholeWorldPoints(i,2))+250) = undistortedImage(floor((i-1)/ncols)+1, mod((i-1),ncols)+1);
    end
end 
if not(testing)
    figure
    imshow(resultWorldIMG)
    title("Mapping from undistorted image");
end
%}

result = zeros(200, 500, "uint8");
worldMapping = zeros(imageSize, 2, "single");
for i = 1:imageSize
    if undistortedPoints(i,1) >= 1 && undistortedPoints(i,1) <= ncols && undistortedPoints(i,2) >= 1 && undistortedPoints(i,2) <= mrows
        worldMapping(i,1:2) = wholeWorldPoints(int32(undistortedPoints(i,1))+int32(undistortedPoints(i,2)-1)*ncols,1:2); 
    else
        worldMapping(i,1:2) = NaN(1,2);
    end
end

for i = 1:imageSize
    if worldMapping(i,1)>=1 && worldMapping(i,1)<=2000 && worldMapping(i,2)>-2500 && worldMapping(i,2)<=2500
        result(201-int32(worldMapping(i,1)/10), int32(worldMapping(i,2)/10)+250) = fieldImage(floor((i-1)/ncols)+1, mod((i-1),ncols)+1);
    end
end
if (displayResult)
    figure
    imshow(result)
    title("result of coordinate mapping");
end

fileID = fopen('mapping.txt','w');
fprintf(fileID,'{');
fprintf(fileID,'{%d,%d},',worldMapping);
fprintf(fileID,'}');
fclose(fileID);
