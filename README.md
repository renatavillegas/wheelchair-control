# wheelchair-control

Project structure:

build/ : Directory containing the build instructions of the project.

lib/ : All object source and header files. The wheelchair control is formed by: - A camera object to handle the image captured by the webcam. - A maker object to handle the door detection and tracking. - A camera calibration object to do the calibration when needed.

main.cpp: responsable for door detection and start simulation. Requires a camera calibration file. If it's not present, the user has the option to do the calibration.

Example of running: ./Main cameraFile.txt

