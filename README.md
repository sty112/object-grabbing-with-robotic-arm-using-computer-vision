# object-grabbing-with-robotic-arm-using-computer-vision

Introduction:
This is a capstone project for my mechanical engineering course, the purpose of the project is to aid childrens in tidying up their table after they finish playing their legos or blocks as children would always have the tendency to just leave it there. So our solution is to make a robotic arm that can detect colors and shapes of the block so that it could classify them and put them back into their respective places.

procedure:
1. We took a week or 2 to plan out what we want to use for our robotic arm and came up with the plan of using webcam for our color and shape detection, and for the grabbing we planned to use suction method as what we wish to grab is rather light so suction does the trick and also sucking it does not damage it, for we decide to use a 4dof(degree of freedom) arm controlled by servo motors, we also decided to make a platform to hold the arm and to hold the camera so everything like detecting,grabbing,sorting will be done on the platform.

2. After planning we then start buying materials for our arm and platform. And we did some of the metal cutting, wood cutting and all sorts using the cnc machince provided by our university workshop. Then we assembled the robotic arm and the platform.

3. After doing the hardware stuff, I did most of the programming. I programmed the camera to detect shapes and colors and also calculate the x,y,z distance of the center of the block from the center of the camera. The x,y,z distance are input to the inverse kinematics equation to get the angles that each servo should rotate to get to the desired position(position where the arm is able to suck the block). The details are in the object_recognition_and_inverse_kinematics.py. After getting the angle which all the servo should move from the inverse kinematics and knowing the shape and colour of the block. I input all the information to the arduino so that it controls the arm to grab the block and classfies it.

result of our project
https://drive.google.com/file/d/1GfFEaVmWslBWgRzcu7SBPEcwTRSz7XZk/view?usp=sharing

color and shape detection
https://drive.google.com/file/d/19EYIesPoYLWcO8J0zTPoYtYav52ZOwXR/view?usp=sharing

x,y distant calculation
https://drive.google.com/file/d/1LNPCpmlf5B3y9Jb9uIlFQFEqcZu-E7Hx/view?usp=sharing
