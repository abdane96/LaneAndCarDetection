# LaneAndCarDetection

# Motivation
The purpose of this code is to use image processing to detect lane lines in a side at an intersection. In addition, this code detects cars passing by in that side of the intersection. This is used for a smart IoT traffic intersection to count cars in all four sides of an intersection, and depending on the car density in each side, the code alters the Green/Red/Yellow/LeftTurn signals accordingly

# ScreenShots
![ScreenShotForReadme](https://user-images.githubusercontent.com/35541525/54621268-df389a80-4a3d-11e9-8dd8-8a40397c02b9.png)

# Tech/Framework Used
The code is written in C++ and uses OpenCV library for image processing

# Code Example
capVideo.open("YourVideoHere.mp4");
capVideo2.open("YourVideoHere.mp4");
capVideo3.open("YourVideoHere.mp4");
capVideo4.open("YourVideoHere.mp4");
