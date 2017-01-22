**findThatObject project**


Simple object recognition in point clouds. It performs a coarse recognition moving the object in the scene following a gaussian distribution using the center of mass of the scene as mean and its variance as sigma. The transformation with the maximum number of correspondences is kept as initial guess for ICP.


**To install it type in terminal:**


1)git clone https://github.com/canematto/findThatObject

2)cd findThatObject

3)mkdir build

4)cd build

5)cmake ..

6)make
