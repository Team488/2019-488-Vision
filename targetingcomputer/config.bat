cd %~dp0
mkdir build
cd build
cmake .. -G "NMake Makefiles" -Drealsense2_DIR="%realsense2_DIR%"
nmake