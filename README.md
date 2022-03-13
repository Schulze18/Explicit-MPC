# Explicit MPC

Files from my undegraduate thesis **Off-line Model Predictive Control applied to Robotic Systems**. The work is split in different folders:

*/Regulation Problem*\
Algorithms for implementation the Explicit MPC presented in **The explicit linear quadratic regulator for constrained systems** and **An algorithm for multi-parametric quadratic programming and explicit mpc solutions**.

*/Non_Zero_Set_Point_Regulation*\
Project of a setpoint regulator for the quadcopter Iris 3DR. I recommend to use this files as the base of your application, the degeneration problem is full solved in this codes. 

*/Binary_search_tree*\
Algorithms to representate the PWA from Explicit MPC using Binary Search Tree as presented in **Computation and Approximation of Piecewise Affine Control Laws
via Binary Search Trees**. There is also C++ libraries for convert from .mat to .txt
and to implement the BST. Copy or edit the files *binary_tree_generator_region.m* and *iris_mimo_implementation.m* with the data from your system.

A plugin with the BST implemented for the Quadrotor Iris 3DR on Gazebo is available [here](https://github.com/Schulze18/iris_plugin_explicit_mpc).

If you want to read the full-text from my thesis, check my Research Gate [here](https://www.researchgate.net/publication/343360836_Off-line_Model-Based_Predictive_Control_aplicado_a_Sistemas_Roboticos). I also published a [paper](https://ieeexplore.ieee.org/document/9480185) comparing this controller to a conventional MPC.

If you have any doubts, feel free to ask me.

## Software Versions:
MATLAB R2016a\
Simulink 8.7\
ROS Kinetic\
Gazebo 7

## Dependencies:
plotregion (optional) - [Link](https://www.mathworks.com/matlabcentral/fileexchange/9261-plot-2d-3d-region?focused=5143921&tab=function)\
YALMIP - [Link](https://yalmip.github.io/download/)\
SeDuMi - [Link](http://sedumi.ie.lehigh.edu/)\
SDTP3 - [Link](http://www.math.nus.edu.sg/~mattohkc/sdpt3.html)\
GLPK - [Link](https://www.gnu.org/software/glpk/)
