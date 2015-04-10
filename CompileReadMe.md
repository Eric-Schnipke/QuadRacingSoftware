How to compile within Xcode.
=============================
- In Project Navigator, left-click on project to arrive at setup location.
- Must include these linker flags in both debug and release directories: -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_contrib -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_ml -lopencv_objdetect -lopencv_video
- Use this header search path: /usr/local/include
- Use this library search path: /usr/local/lib /usr/local/Cellar/opencv/2.4.7.1/lib
