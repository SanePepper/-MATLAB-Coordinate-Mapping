# Coordinate Mapping with distortion fix and worldview transform using Matlab
A Matlab program to perform distortion fix and worldview transform. Maps the coordinate from the captured image to world coordinates.

### Prerequisites
```
Matlab with the Camera Calibrator App
```

### Demo
<img src="https://github.com/SanePepper/-MATLAB-Coordinate-Mapping/blob/master/original.jpg" width="184"/>

Capture multiple images of an asymetric checkerboard from different angle.

<img src="https://github.com/SanePepper/-MATLAB-Coordinate-Mapping/blob/master/fisheyefix.jpg" width="184"/>

The program would fix the lens distortion.

<img src="https://github.com/SanePepper/-MATLAB-Coordinate-Mapping/blob/master/birdeyeview.JPG" width="184"/>

And use the first image as a reference of camera angle for performing perspective fix.

The coordinate mapping would be saved as "mapping.txt" and can be applied on other images taken with the same lens and camera angle.

<img src="https://github.com/SanePepper/-MATLAB-Coordinate-Mapping/blob/master/field.jpg" width="184"/>

Test image.

<img src="https://github.com/SanePepper/-MATLAB-Coordinate-Mapping/blob/master/result.jpg" width="500"/>

Result of perspective fix and distortion fix.
