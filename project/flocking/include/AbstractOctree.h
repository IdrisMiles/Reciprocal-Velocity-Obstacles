#ifndef ABSTRACTOCTREE__H_
#define ABSTRACTOCTREE__H_
#include <vector>
#include <iostream>
#include <boost/smart_ptr.hpp>
#include <cstdlib>
#include <ngl/Vec2.h>
#include <ngl/Vec3.h>
#include <ngl/Vec4.h>
/// @file AbstractOctree.h
/// @brief Octree structure for collision detection the user must impliment the
/// pure virtual checkCollisionOnNode method for appropriate collision
/// also the templated class must have a getPosition method that returns a vector of the
/// correct template type and
/// a getRadius method to return a bounding radius of the object as a float
/// may update this to do bounding radius or bounding box at some stage
/// @author Xiaosong Yang modifed by Jon Macey and Idris Miles
/// @version 1.0
/// @date 11/3/15
/// @class AbstractOctree
/// @brief this is the class for perfect octree tree, 3 levels,


class BoundingBox
{
public :
    ngl::Real m_minx;
    ngl::Real m_maxx;
    ngl::Real m_miny;
    ngl::Real m_maxy;
    ngl::Real m_minz;
    ngl::Real m_maxz;
};


template <class T, class VEC> class AbstractOctree
{
protected :

  template <class V>class TreeNode
  {
  public:
      BoundingBox m_limit;
      int m_height;
      std::vector< boost::shared_ptr<V> > m_objectList;
      TreeNode *m_child[8];
  };

public:
    /// @brief constructor
    /// @param[in] _height the height of the octree, should be 3 in this example
    /// @param[in] _limit the Bounding box of the tree
    AbstractOctree(int _height, BoundingBox _limit)
    {
      if(_height<=0)
      {
        std::cerr<<"The height of the tree is not valid\n";
        exit(EXIT_FAILURE);
      }
      m_root = new TreeNode <T>;
      createTree(m_root, _height, _limit);
    }

    /// @brief destructor
    virtual ~AbstractOctree()
    {
        deleteTreeNode(m_root);
        delete m_root;
    }

    void deleteTreeNode(TreeNode <T> *_node)
    {
      for(int i=0;i<8;++i)
      {
        if(_node->m_child[i]!=0)
        {
          deleteTreeNode(_node->m_child[i]);
          delete _node->m_child[i];
        }
      }
    }

    /// @brief create perfect tree, it is leaves level when _height==1
    void createTree(TreeNode <T> *_parent,  int _height, BoundingBox _limit)
    {
      _parent->m_limit = _limit;
      _parent->m_height = _height;
      _parent->m_objectList.clear();
      _height--;
      if(_height == 0) // current level is leaves, no children
      {
        for(int i=0;i<8;++i)
        {
         _parent->m_child[i]= 0;
        }
      }
      else
      {
        BoundingBox childLimit;
        for(int i=0;i<8;++i)
        {
          _parent->m_child[i] = new TreeNode <T>;
          // Task 1: compute the correct bounding limit (childLimit) for this child

          if(i < 4)
          {
              childLimit.m_maxz = _limit.m_minz + (_limit.m_maxz - _limit.m_minz)*0.5;
              childLimit.m_minz = _limit.m_minz;
          }
          else
          {
              childLimit.m_maxz = _limit.m_maxz;
              childLimit.m_minz = _limit.m_minz + (_limit.m_maxz - _limit.m_minz)*0.5;
          }

          if(i==0 || i==1 || i==4 || i==5)
          {
              childLimit.m_maxy = _limit.m_miny + (_limit.m_maxy - _limit.m_miny)*0.5;
              childLimit.m_miny = _limit.m_miny;
          }
          else
          {
              childLimit.m_maxy = _limit.m_maxy;
              childLimit.m_miny = _limit.m_miny + (_limit.m_maxy - _limit.m_miny)*0.5;
          }

          if(i==0 || i==2|| i==4 || i==6)
          {
              childLimit.m_maxx = _limit.m_minx + (_limit.m_maxx - _limit.m_minx)*0.5;
              childLimit.m_minx = _limit.m_minx;
          }
          else
          {
              childLimit.m_maxx = _limit.m_maxx;
              childLimit.m_minx = _limit.m_minx + (_limit.m_maxx - _limit.m_minx)*0.5;
          }


          createTree(_parent->m_child[i], _height, childLimit);
        } // end for
      }// end else
    }

    /// @brief add a T into the tree, add the partile to all the leaves collide with
    void addObject(boost::shared_ptr<T> _p)
    {
      addObjectToNode(m_root, _p);
    }
    /// @brief some template specialization to hand different vector types
    bool checkBounds (const ngl::Vec3 &_pos, ngl::Real _r, const BoundingBox &_limit)
    {
        // Task2

      return( _pos.m_x+_r < _limit.m_minx ||
              _pos.m_y+_r < _limit.m_miny ||
              _pos.m_z+_r < _limit.m_minz ||
              _pos.m_x-_r > _limit.m_maxx ||
              _pos.m_y-_r > _limit.m_maxy ||
              _pos.m_z-_r > _limit.m_maxz   );




    }

    /// @brief some template specialization to hand different vector types
    bool checkBounds (const ngl::Vec4 &_pos, ngl::Real _r, const BoundingBox &_limit)
    {
        // Task2

      return checkBounds(_pos.toVec3(),_r,_limit);

    }
    /// @brief some template specialization to hand different vector types
    bool checkBounds (const ngl::Vec2 &_pos, ngl::Real _r, const BoundingBox &_limit)
    {
        // Task2

      return( _pos.m_x+_r < _limit.m_minx ||
              _pos.m_y+_r < _limit.m_miny ||
              _pos.m_x-_r > _limit.m_maxx ||
              _pos.m_y-_r > _limit.m_maxy   );



    }
    void addObjectToNode(TreeNode <T>  *_node, boost::shared_ptr<T> _p)
    {
      if(_node->m_height == 1) // this is the leaves level
      {
        _node->m_objectList.push_back(_p);
      }
      else
      {
        VEC pos = _p->getOrigState().m_pos;//getPos();
        ngl::Real   r = _p->getOrigState().m_rad;
        BoundingBox limit;
        for(int i=0;i<8;++i)
        {
          limit = _node->m_child[i]->m_limit;
          // Task 2: if the object fall in or intersect with the limit of this child, add this object to this subtree
          // Attention: all objects added to leaf node, all internal node doesn't hold any object;
          if(!checkBounds(pos,r,limit))
          {
              //continue;
              addObjectToNode(_node->m_child[i], _p);
              break;
          }
          //addObjectToNode(_node->m_child[i], _p);
        }
      }
    }

    /// @brief clear off all the Ts from the tree
    void clearTree()
    {
        clearObjectFromNode(m_root);
    }

    void clearObjectFromNode(TreeNode <T> *_node)
    {
      if(_node->m_height == 1)
      {
        _node->m_objectList.clear();
      }
      else
      {
        for(int i=0;i<8;++i)
        {
            clearObjectFromNode(_node->m_child[i]);
        }
        _node->m_objectList.clear();
      }
    }

    /// @brief check the collision for the Ts in each leaves
    void    addNeighbours()
    {
        checkAgentNeighbours(m_root);
    }

    virtual void  checkAgentNeighbours(TreeNode <T> *_node)=0;


  protected :
    TreeNode <T> *m_root;
};

#endif

