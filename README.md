motion-solver
=============

The goal of motion planning is to find a collision-free path for a robot that moves in an environment populated with obstacles. This software can be used to solve these motion planning problems.

The software assumes that the robot is a rigid body that moves in a three-dimensional environment that contains static obstacles. A quite basic probabilistic roadmap algorithm is implemented in the software but the source code is structured in such a way that it is fairly easy to extend it to support more sophisticated methods.

The software is written in C++11 and it uses Qt and Eigen. It can be compiled with the GNU C++ compiler. The software uses a simple ColDet library to detect collisions. It can easily be replaced with a more powerful library.
