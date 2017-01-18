#ifndef FINDTHATOBJECT_KDTREE_H
#define FINDTHATOBJECT_KDTREE_H

/************** Simple didactic implementation of a search tree for unorganized point sets ***********/

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>

// uncomment the row below if you want to see debug messages
// #define _VERBOSE_BUILD_

using namespace Eigen;
using namespace std;

// typedef for defining a vector variable sized points
typedef std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> >
        VectorXdVector;

/**
   BaseTreeNode class. Represents a base class for a node in the search tree.
   It hasa dimension (given on construction).
*/

class BaseTreeNode{
public:
    //! ctor
    BaseTreeNode(int dimension_);
    //! dtor
    virtual ~BaseTreeNode(){}
    //! function to search for the neighbot
    //! @param answer: the neighbor found
    //! @param query: the point to search
    //! @param maximum distance allowed for a point
    //! @returns the distance of the closest point. -1 if no point found within range
    virtual double findNeighbor(Eigen::VectorXd& answer,
                                const Eigen::VectorXd& query,
                                const double max_distance) const =0;

    //dimension accessor;
    inline int dimension() const {return _dimension;};

protected:
    int _dimension; //< the dimension of a node in the tree
};

/**
   Leaf node: it contains a vector of points on which.
   The search function performs a linear search in the list
*/

class LeafNode: public BaseTreeNode{
public:
    //! ctor
    LeafNode(int dimension_):BaseTreeNode(dimension_){}

    //! const accessor to the point vector
    const VectorXdVector& points() const;

    //! accessor to the point vector
    VectorXdVector& points();

    //! function to search for the neighbors. Performs a linear search in the vector
    //! @param answer: the neighbor found
    //! @param query: the point to search
    //! @param maximum distance allowed for a point
    //! @returns the distance of the closest point. -1 if no point found within range
    virtual double findNeighbor(Eigen::VectorXd& answer,
                                const Eigen::VectorXd& query,
                                const double max_distance) const;


protected:
    VectorXdVector _points; //< points stored in the leaf
};


/**
   Middle node: it represents a splitting plane, and has 2 child nodes,
   that refer to the set of points to the two sides of the splitting plane.
   A splitting plane is parameterized as a point on a plane and as a normal to the plane.
*/
class MiddleNode: public BaseTreeNode{
public:
    //! ctor
    //! @param mean_: point on the split plane
    //! @param normal_: normal vector to the plane
    //! param left_child: left subtree
    //! param right_child: right subtree
    MiddleNode(int dimension_,
               const Eigen::VectorXd& mean_,
               const Eigen::VectorXd& normal_,
               BaseTreeNode* left_child=0,
               BaseTreeNode* right_child=0) :BaseTreeNode(dimension_){
        assert(normal_.rows()==dimension());
        assert(mean_.rows()==dimension());
        _normal=normal_;
        _mean=mean_;
        _left_child=left_child;
        _right_child=right_child;
    }

    //! dtor
    virtual ~MiddleNode() ;

    //! mean const accessor
    inline const Eigen::VectorXd& mean() const ;

    //! normal const accessor
    inline const Eigen::VectorXd& normal() const ;

    //! checks if a point lies to the left or right of a plane
    //! @param query_point: the point to be checked
    //! @returns true if a point lies to the left, false otherwise
    inline bool side(const Eigen::VectorXd& query_point) const ;


    //! binary search for the neighbors. Performs a linear search in the vector
    //! @param answer: the neighbor found
    //! @param query: the point to search
    //! @param maximum distance allowed for a point
    //! @returns the distance of the closest point. -1 if no point found within range
    virtual double findNeighbor(Eigen::VectorXd& answer,
                                const Eigen::VectorXd& query,
                                const double max_distance) const;


protected:
    Eigen::VectorXd _normal;
    Eigen::VectorXd _mean;
    BaseTreeNode* _left_child;
    BaseTreeNode* _right_child;
};


/**
Partitions a point vector in two vectors, computing the splitting plane
as the largest eigenvalue of the point covariance
@param mean: the returned mean of the splitting plane
@param normal: the normal of the splitting plane
@param left: the returned left vector of points
@param right: the returned right vector of points
@param points: the array of points
@returns the distance of the farthest point from the plane
*/
double splitPoints(VectorXd& mean, VectorXd& normal,
                   VectorXdVector& left, VectorXdVector& right,
                   const VectorXdVector& points);

//! helper function to build a tree
//! @param points: the points
//! @param max_leaf_range: specify the size of the "box" below which a leaf node is generated
//! returns the root of the search tree
BaseTreeNode* buildTree(const VectorXdVector& points,
                        double max_leaf_range) ;
#endif //FINDTHATOBJECT_KDTREE_H
