# Coordinate Mapping with distortion fix and worldview transform using Matlab
A Matlab program to perform distortion fix and worldview transform. Maps the coordinate from the captured image to world coordinates.

### Prerequisites
```
Matlab with the Camera Calibrator App
```

### Demo
![](https://github.com/SanePepper/-MATLAB-Coordinate-Mapping/blob/master/original.jpg)

Capture multiple images of an asymetric checkerboard from different angle.

![](https://github.com/SanePepper/-MATLAB-Coordinate-Mapping/blob/master/fisheyefix.jpg)

The program would fix the lens distortion.

![](https://github.com/SanePepper/-MATLAB-Coordinate-Mapping/blob/master/birdeyeview.JPG)

And use the first image as a reference of camera angle for performing perspective fix.

The coordinate mapping would be saved as "mapping.txt" and can be applied on other images taken with the same lens and camera angle.

![](https://github.com/SanePepper/-MATLAB-Coordinate-Mapping/blob/master/field.jpg)

Test image.

![](https://github.com/SanePepper/-MATLAB-Coordinate-Mapping/blob/master/result.jpg)

Result of perspective fix and distortion fix.
