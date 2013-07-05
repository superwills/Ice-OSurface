#ifndef VOXEL_H
#define VOXEL_H

#include "Vectorf.h"

// handling for VOlumetrix piXEL (where pixel was PIXture ELement)
struct Voxel
{
  float v ;
  Vector4f d ;
  Vector4f color ;
} ;

struct VoxelGrid
{
  // the actual 3d grid of voxels, stored as a linear array.
  // You're not supposed to write these directly without calling RESIZE.
  // A more conservative programmer would mark them private.
  //int cols, rows, slabs ; // I used to have 3 variables instead of the Vector3i dims:
  // cols for x, rows for y, and slabs for z.
  Vector3i dims ; // dims is used when you need cols,rows,slabs as an xyz Vector3i

  // I let you access the voxel grid directly if you want to.
  vector<Voxel> voxels ;

  // These variables are about where the voxel grid meets the world.
  // by default the voxel grid goes on positive indices in xyz,
  // but you may want voxel (0,0,0) to be at (-10.5,-10.5,-10.5) of your rendered world.
  // So, these are the world transformations to apply when rendering the voxel grid.
  float worldSize ;

  Vector3f offset ;     // the offset appliied to the voxel grid to center it in world space
  Vector3f gridSizer ;  // blow up the visualization so it isn't too small

  void defaults(){
    dims=Vector3i(10);
    worldSize=200;
  }

  VoxelGrid()
  {
    defaults() ;
    resize() ;
  }

  VoxelGrid( int size )
  {
    defaults() ;
    dims=Vector3i(size);
    resize() ;
  }

  VoxelGrid( int iCols, int iRows, int iSlabs )
  {
    defaults() ;
    dims = Vector3i( iCols, iRows, iSlabs ) ;
    resize() ;
  }

  // Call when #cols,rows,slabs has changed.
  void resize()
  {
    voxels.resize( dims.x*dims.y*dims.z ) ;

    // recalculate the offset to center the voxel grid in the world
    offset = -Vector3f(dims)/2.f ;

    // gridSizer first normalizes cols/rows/slabs, then mults each by
    // worldSize.
    gridSizer = Vector3f(worldSize) / dims ;
  }

  inline float wallValue( int wallSide ) const
  {
    int negWall = wallSide%2 ; // 0 for PX, PY (+walls),
    // 1 for negative walls

    // -2*negWall+1 gives: +1 for P walls, -1 for N walls
    // (0=>+1, 1=>-1)
    return (-2*negWall+1)*worldSize/2.f ;
  }

  void increaseResolution( int by )
  {
    dims += by ;
    resize();
  }

  void increaseWorldSize( float by )
  {
    worldSize += by ;
    gridSizer = Vector3f(worldSize) / dims ;
  }

  // converts triple indexing into a linear index to use on voxels array.
  inline int index( int i, int j, int k ) const
  {
    // These comments use the old mapping of dims.x==cols, dims.y==rows, dims.z==slabs.
    // i:             col index=> directly to linear offset.
    // j*cols:        row index=> j*cols elts / row. This value gives linear offset.
    // k*cols*rows:  slab index=> cols*rows elts covered / slab. Advance this many in linear array to skip to slab #k.
    return i + j*dims.x + k*dims.x*dims.y ;
  }
  inline int index( const Vector3i& idx ) const
  {
    return index( idx.x, idx.y, idx.z ) ;
  }

  inline Vector3i& wrappedIndex( Vector3i& idx ) const
  {
    // modulus negative number not positive in C, so 
    // you have to first take it to the +side by adding (cols,rows,slabs),
    // then modulus by those dims.
    idx += dims ;
    idx %= dims ;
    return idx;
  }

  // just directly looks up into the voxels array.
  // because the voxel grid WRAPS,
  inline Voxel& operator()( int i, int j, int k ) {
    return (*this)( Vector3i(i,j,k) ) ;
  }

  // You can use a Vector3i to index the voxel grid too
  inline Voxel& operator()( Vector3i idx ) {
    wrappedIndex( idx ) ;
    return voxels[ index(idx.x,idx.y,idx.z) ] ;
  }

  inline Voxel& getVoxel( Vector3i idx ) {
    return (*this)( idx ) ;
  }

  // Gets you the real world space point of
  // a given voxel grid index (using offset&gridSizer).
  Vector3f getP( const Vector3i& dex )
  {
    return ( offset + dex ) * gridSizer ;
  }

  // Gets you the 3-space isosurface cut point
  Vector3f getCutPoint( float isosurface, const Vector3i& A, const Vector3i& B )
  {
    // Minor optimization comment: repeated calls to getVoxel() DO happen
    // for the same exact grid point.  that's a couple of adds and multiplies
    // just to do the lookup, its best to cache these values.
    float vA = getVoxel( A ).v ; // REDUNDANT
    float vB = getVoxel( B ).v ; // REDUNDANT
  
    // Get the `t` that represents "% of the way from vA to vB"
    float tAB = unlerp( isosurface, vA, vB ) ;

    // 0 means @ vA.  1 means @ vB.  Outside of this range means
    // before vA or beyond vB (error).
    if( !isBetween( tAB, 0.f, 1.f ) )
    {
      // This error should ONLY come up if you look for a voxel grid intersection
      // between two vertices that are ON THE SAME SIDE OF THE ISOSURFACE.
      // In general if you are checking correctly a call like this should NEVER HAPPEN,
      // hence the detailed warning.
      printf( "ERROR: CUT POINT FOR ISOSURFACE %f "
        "NOT BETWEEN VOXEL GRID (%d,%d,%d)=%f AND (%d,%d,%d)=%f. "
        "CHECK YOUR ALGORITHM. t=%f\n", isosurface,
         A.x,A.y,A.z,vA,  B.x,B.y,B.z,vB, tAB ) ;
      return 0 ;
    }
    Vector3f cutAB = Vector3f::lerp( tAB, getP(A), getP(B) ) ;
    return cutAB ;
  }

  void genData()
  {
    resize() ; // ensure voxel grid is right size.

    Vector4f d1, d2 ;
    for( int i = 0 ; i < dims.x ; i++ )
    {
      for( int j = 0 ; j < dims.y ; j++ )
      {
        for( int k = 0 ; k < dims.z ; k++ )
        {
          int dex = index(i,j,k);
          float fx=(float)i/dims.x, fy=(float)j/dims.y, fz=(float)k/dims.z ;
        
          //voxels[ dex ].v = Perlin::sdnoise( fx*f1, fy*f1, fz*f1, pw, &d1.x, &d1.y, &d1.z, &d1.w ) ;
          //voxels[ dex ].v += Perlin::sdnoise( fx*f2, fy*f2, fz*f2, pw, &d2.x, &d2.y, &d2.z, &d2.w ) ;
          //voxels[ dex ].d = d1 + d2 ;

          voxels[ dex ].v = Perlin::pnoise( fx, fy, fz, pw, 1,1,1, pwPeriod ) ;

          for( int i = 2 ; i <= 4 ; i *= 2 )
            voxels[ dex ].v += Perlin::pnoise( fx*i, fy*i, fz*i, pw, i,i,i, pwPeriod ) ;

          //voxels[ dex ].v = Perlin::noise( fx*f1, fy*f1, fz*f1, pw ) -
          //                  fabsf( Perlin::noise( fx*f2, fy*f2, fz*f2, 10*pw ) ) ; //randFloat() ;
          //voxels[ dex ].v = Perlin::noise( sin(fx), cos(fy), sin(fz), w ) ; //randFloat() ;
        }
      }
    }
  }




} ;


#endif