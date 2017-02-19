**findThatObject project**


Simple object recognition in point clouds. It performs a coarse recognition moving the object in the scene following a gaussian distribution using the center of mass of the scene as mean and its variance as sigma. The transformation with the maximum number of correspondences is kept as initial guess for ICP.

**Dependencies**

To run the program you need PCL (Point Cloud Library). Please refere to the following link.

http://www.pointclouds.org/downloads/

If you fail some dependencies to install PCL please refere to the following link.

https://github.com/huningxin/install_pcl_deps/blob/master/install_pcl_deps.sh


**How to install**

To install the program type in terminal:


1)git clone https://github.com/canematto/findThatObject

2)cd findThatObject

3)mkdir build

4)cd build

5)cmake ..

6)make -j4


**Output files**

The program generates three different files that show the evolution of the least square optimization. Also, for each one, a gnuplot snippet is included for easily visualization. 


**Typical usage:**

./findThatObject `<options>` `<path_to_scene_file>` `<path_to_object_file>` 


**Example:**

./findThatObject -v 0.015 ../globe-scene.pcd ../globe.pcd
