News！This work has been integrated into OpenCV contrib modules, see below for detail:
https://github.com/opencv/opencv_contrib/tree/master/modules/rapid

# GOSTracker
Related Publication: 
Guofeng Wang etc. Global Optimal Searching for Textureless 3D Object Tracking. 
The Visual Computer, 2015.

The project now has include levmar and freeglut, so you only need to set OpenCV path in the CMake.
The levmar is not built with lapack and blas , so the speed is not fast. 
If your system has lapack and blas, you can set HAVE_LAPACK 1, it will be more faster.

The test data can be found in the data folder
