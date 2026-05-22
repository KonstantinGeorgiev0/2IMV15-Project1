Compilation and execution instructions for the code


Prerequisites
This project is built using C++ and OpenGL/GLUT. It supports environment with a C++ compiler and the necessary OpenGL/GLUT framework development libraries.
Compilation Steps
Commands for build: 
make clean
make

The compilation process generates the executable binary inside the target binaries folder.
Running the Simulation
To launch the application we run the executable from the project root directory:
./bin/project1.exe

Runtime Controls
The simulation initializes in a paused configuration. Use the keys mentioned below to control the scenes and interactions respectively:
v
Open parameter modification menu in console
1/2/3     
Switch solver (Euler/Midpoint/RK4)
 p/o       
 Increase/decrease dt (timestep)
i/u       
Increase/decrease mouse spring stiffness
k/j       
Increase/decrease mouse damping
s       
 Switch between scenes
r               
Toggle between sqrt and squared formula for RodConstraint
w        
Toggle wind force
f         
Toggle fixing top row of cloth
c         
Clear/reset simulation
d         
Toggle frame dumping
space     
Toggle simulation/construction mode
q        
Quit



