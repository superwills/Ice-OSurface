#ifndef MARCHINGCOMMON_H
#define MARCHINGCOMMON_H

extern float EPS ;

#include "VoxelGrid.h"

// Common base class for finding an isosurface.
struct IsosurfaceFinder
{
  // the value of the isosurface
  float isosurface ; 

  // how thick the isosurface is to be AROUND isosurface (unused)
  float isosurfaceThickness ;
  
  // The voxel grid I am operating on.
  VoxelGrid *voxelGrid ;
  
  // Pointers to arrays in caller program space
  vector<VertexPNC> *verts ;
  
  Vector4f baseColor ;
  
  IsosurfaceFinder( VoxelGrid *iVoxelGrid, vector<VertexPNC>* iVerts, float iIsosurface, const Vector4f& iBaseColor )
  {
    voxelGrid = iVoxelGrid ;
    verts = iVerts ;
    isosurface = iIsosurface ;
    isosurfaceThickness = 0.1f;
    baseColor = iBaseColor ;
  }

  // The primitives for determining if a point is in an isosurface or not.
  bool inSurface( float v )
  {
    return v < isosurface ; 

      // These don't work:
      // cut point where you get `isosurfaceThickness` units away from isosurface.
      //return fabsf( v-isosurface ) < isosurfaceThickness ;
      //isNear( v, isosurface, isosurfaceThickness ) ;
      //(isosurface - isosurfaceThickness) < v && v < (isosurface+isosurfaceThickness) ;
  }

  // so to avoid COMPLETE fill, you DON'T gen a tet for 
  // fully embedded tet that is ALL TOO DEEP
  bool tooDeep( float v )
  {
    return v < isosurface - isosurfaceThickness ; 
  }


} ;

#endif