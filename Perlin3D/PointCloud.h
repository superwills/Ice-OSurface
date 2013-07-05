#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "Vectorf.h"

struct PointCloud
{
  // Point cloud vars.
  static float ptSize, cubeSize ;
  static bool useCubes ;  // point clouds use cubes?

  // The voxel grid I am operating on.
  VoxelGrid *voxelGrid ;

  // Pointers to arrays in caller program space
  vector<VertexPNC> *verts ; // the vertex array into which to drop my results.

  // structures and code for creating the point cloud
  // with points at isosurface breakthroughs etc.
  #pragma region structs
  // This code is generally not to be used unless you want to start
  // from a point where you JUST KNOW the isosurface punchthru points,
  // but nothing about how the surface is connected.

  // It may be a nice starting point for generating a tetrahedralization based on
  // a point cloud.
  struct IsosurfacePunchthru
  {
    // the original voxel index that DETECTED the punchthru
    Vector3i voxelIndex ;

    // The actual 3-space point of the punchthru
    // this is computed based on dv and the values at
    // the actual voxels where this punchthru took place.
    //Vector3f pt ;
    int directionIndex ; // the direction of the vector that breaks the isosurface.
    // looks up into IsosurfacePunchthruSet.Directions[ directionIndex ] to get the actual direction.
  
    float t ;    // how far between voxelIndex and the voxel at directionIndex you were at isosurface intn.
    float dv ;   // the actual jump in value across dir.

    IsosurfacePunchthru():t(0.f),dv(0.f)
    {
    }

    IsosurfacePunchthru( Vector3i iVoxelIndex, int iDirectionIndex, float iT, float iDv ) :
      voxelIndex(iVoxelIndex),directionIndex(iDirectionIndex),t(iT),dv(iDv)
    {
    }
  } ;

  // Every POINT stores the directions for which the isosurface
  // punchthru succeeded.  So you store up to (Directions.size())
  // POINTERS inside an IsosurfacePunchthruSet.
  struct IsosurfacePunchthruSet
  {
    // There are 26 directions over which the isosurface can break.
    // (1 towards each of the 26 points that surround each 3x3 group).
    // If isosurface is drawn differently for going - and going + into it,
    // it will be 2 sided and have different colors.  Or you can just draw
    // the + side, or you can even give it a thickness (provided grid spacing
    // is course enough.)

    // indexing is a 3x3x3 cube LINEAR indexing 
    // in the order z,y,x.
    vector<IsosurfacePunchthru*> directedPunchthrus ; // punchthrus indexed by directions in Directions.

    IsosurfacePunchthruSet( int numDirs )
    {
      directedPunchthrus.resize( numDirs, 0 ) ;
    }
    ~IsosurfacePunchthruSet()
    {
      for( int i = 0 ; i < directedPunchthrus.size() ; i++ )
        if( directedPunchthrus[i] )
          delete directedPunchthrus[i] ;
    }
  } ;
  #pragma endregion

  // 1 isosurface punchthruset PER voxel
  vector<IsosurfacePunchthruSet> punchthru ;
  
  // The set of Directions for which isosurface punchthrus are determined
  // This is 
  vector<Vector3i> Directions ;

  // every DIRECTION has 1 or more NEIGHBOURS.
  // So, this maps a 
  vector< vector<int> > DirsNeighbours ;
  // Directions[i] has neighbours listed by DirsNeighbours[i][0]..DirsNeighbours[i][size]

  PointCloud( VoxelGrid *iVoxelGrid, vector<VertexPNC>* iVerts, float iIsosurface )
  {
    voxelGrid = iVoxelGrid ;
    verts = iVerts ;
    isosurface = iIsosurface ;
    initDirections() ;
  }

  // outputs a "point" into the global array data. used by pointcloud visualization.
  void pt( const Vector3f& p, float size, const Vector4f& color )
  {
    if( useCubes )
      Geometry::addCubeFacingOut( *verts, p, size*cubeSize, color ) ;
    else
      verts->push_back( VertexPNC( p, Vector3f(0,1,0), color ) ) ;
  }

  void genVizPunchthru()
  {
    punchthru.clear() ;
    punchthru.resize( voxelGrid->voxels.size(), IsosurfacePunchthruSet( Directions.size() ) ) ;

    for( int k = 0 ; k < voxelGrid->dims.z ; k++ )
    {
      for( int j = 0 ; j < voxelGrid->dims.y ; j++ )
      {
        for( int i = 0 ; i < voxelGrid->dims.x ; i++ )
        {
          Vector3i dex( i,j,k ) ;
          int idex = voxelGrid->index( dex ) ;
          float val = voxelGrid->voxels[ idex ].v ;
        
          // Measure the isosurface breaks in 26 directions.
          //punchthru[dex.index(cols,rows)]. ;
          for( int dirIndex = 0 ; dirIndex < Directions.size() ; dirIndex++ )
          {
            const Vector3i& dir = Directions[dirIndex] ; // GET A DIRECTION. ONE DIRECTION.
            Vector3i adjCell = dex + dir ;    // GET THE IJK INDEX OF THE ADJACENT CELL IN THIS DIRECTION
            voxelGrid->wrappedIndex( adjCell ) ;
            float adjVal = (*voxelGrid)( adjCell ).v ; // MAKE SURE IS IN BOUNDS. WRAP AT BORDERS

            float t = unlerp( isosurface, val, adjVal ) ;
            if( isBetween( t, 0.f, 1.f ) )
            {
              // BROKE THE SURFACE
              float diff = adjVal - val ; //+ if value INCREASES towards adjVal.
              // this is the amount you need to "add" to val to GET adjVal.
              //- if value GOING DOWN
              // like a type of derivative
              punchthru[idex].directedPunchthrus[dirIndex] = new IsosurfacePunchthru( dex, dirIndex, t, diff ) ;
            
              Vector3f voxelCenter = (voxelGrid->offset + dex)*voxelGrid->gridSizer ;
              Vector3f p2 = (voxelGrid->offset + dex + dir)*voxelGrid->gridSizer ;
              Vector3f p = Vector3f::lerp( t, voxelCenter, p2 ) ;
              pt( p, 0.25, Vector4f(0.8) ) ;
            }
          }
        }
      }
    }
  }

  void addDirection( const Vector3i& v )
  {
    Directions.push_back( v ) ;
    DirsNeighbours.push_back( vector<int>() ) ;
  }

  void addNeighbours( int forDirection, int* neighbours, int len )
  {
    if( forDirection >= DirsNeighbours.size() )
    {
      printf( "%d oob DirsNeighbours (%d)\n", forDirection, DirsNeighbours.size() ) ;
      return ;
    }
    for( int i = 0 ; i < len ; i++ )
      DirsNeighbours[forDirection].push_back( neighbours[i] ) ;
  }

  void initDirections()
  {
    addDirection( Vector3i( 1, 0, 0 ) ) ;  //0
    addDirection( Vector3i( 0, 1, 0 ) ) ;  //1
    addDirection( Vector3i( 0, 0, 1 ) ) ;  //2
  
    addDirection( Vector3i( -1,  0,  0 ) ) ; //3
    addDirection( Vector3i(  0, -1,  0 ) ) ; //4
    addDirection( Vector3i(  0,  0, -1 ) ) ; //5

    // UPPER NEIGHBOR, LEFT NEIGHBOUR,  LEFT NEIGHBOUR, LOWER NEIGHBOUR
    int px[4] = { 1, 2,  2, 4 } ;
    addNeighbours( 0, px, 4 ) ;
  
    // y has NO NEIGHBOURS listed.
    int pz[4] = { 1, 3,  3, 4 } ;
    addNeighbours( 2, pz, 4 ) ;

    int nx[4] = { 1, 5,  5, 4 } ;
    addNeighbours( 3, nx, 4 ) ;

    int nz[4] = { 1, 0,  0, 4 } ;
    addNeighbours( 5, nz, 4 ) ;

    for( int i = 0 ; i < Directions.size() ; i++ )
    {
      Vector3i p = Directions[i] + 1 ; // ADD ONE WHEN GETTING INDEX
      //printf( "(%d,%d,%d) index %d\n", p.x, p.y, p.z, p.index( 3, 3 ) ) ;
    }
  }

  
} ;


// Point cloud vars.
float PointCloud::ptSize=1.f, PointCloud::cubeSize=50.f ;
bool PointCloud::useCubes=1 ;
  
#endif