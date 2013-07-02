#ifndef POINTCLOUD_H
#define POINTCLOUD_H

// structures and code for creating the point cloud
// with points at isosurface breakthroughs etc.

// This code is generally not to be used unless you want to start
// from a point where you JUST KNOW the isosurface punchthru points,
// but nothing about how the surface is connected.

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
// punchthru succeeded.
struct IsosurfacePunchthruSet
{
  static vector<Vector3i> Directions ;
  static vector< vector<int> > DirsNeighbours ;
  // Directions[i] has neighbours listed by neighbours[i][0]..neighbours[i][size]

  static void addDirection( const Vector3i& v )
  {
    Directions.push_back( v ) ;
    DirsNeighbours.push_back( vector<int>() ) ;
  }

  static void addNeighbours( int forDirection, int* neighbours, int len )
  {
    if( forDirection >= DirsNeighbours.size() )
    {
      printf( "%d oob DirsNeighbours (%d)\n", forDirection, DirsNeighbours.size() ) ;
      return ;
    }
    for( int i = 0 ; i < len ; i++ )
      DirsNeighbours[forDirection].push_back( neighbours[i] ) ;
  }

  // There are 26 directions over which the isosurface can break.
  // (1 towards each of the 26 points that surround each 3x3 group).
  // If isosurface is drawn differently for going - and going + into it,
  // it will be 2 sided and have different colors.  Or you can just draw
  // the + side, or you can even give it a thickness (provided grid spacing
  // is course enough.)

  // indexing is a 3x3x3 cube LINEAR indexing 
  // in the order z,y,x.
  vector<IsosurfacePunchthru*> directedPunchthrus ; // punchthrus indexed by directions in Directions.

  IsosurfacePunchthruSet()
  {
    directedPunchthrus.resize( Directions.size(),0 ) ;
  }
  ~IsosurfacePunchthruSet()
  {
    for( int i = 0 ; i < directedPunchthrus.size() ; i++ )
      if( directedPunchthrus[i] )
        delete directedPunchthrus[i] ;
  }
} ;

vector<Vector3i> IsosurfacePunchthruSet::Directions ;
vector< vector<int> > IsosurfacePunchthruSet::DirsNeighbours ;
vector<IsosurfacePunchthruSet> punchthru ;

void genVizPunchthru()
{
  punchthru.clear() ;
  punchthru.resize( voxels.size() ) ;

  for( int k = 0 ; k < slabs ; k++ )
  {
    for( int j = 0 ; j < rows ; j++ )
    {
      for( int i = 0 ; i < cols ; i++ )
      {
        Vector3i dex( i,j,k ) ;
        int idex = dex.index(cols,rows) ;
        float val = voxels[ idex ].v ;
        
        // Measure the isosurface breaks in 26 directions.
        //punchthru[dex.index(cols,rows)]. ;
        for( int dirIndex = 0 ; dirIndex < IsosurfacePunchthruSet::Directions.size() ; dirIndex++ )
        {
          const Vector3i& dir = IsosurfacePunchthruSet::Directions[dirIndex] ; // GET A DIRECTION. ONE DIRECTION.
          Vector3i adjCell = dex + dir ;    // GET THE IJK INDEX OF THE ADJACENT CELL IN THIS DIRECTION
          adjCell.wrap( cols,rows,slabs ) ; // MAKE SURE IS IN BOUNDS. WRAP AT BORDERS
          float adjVal = voxels[ adjCell.index(cols,rows) ].v ;

          float t = unlerp( isosurface, val, adjVal ) ;
          if( isBetween( t, 0.f, 1.f ) )
          {
            // BROKE THE SURFACE
            float diff = adjVal - val ; //+ if value INCREASES towards adjVal.
            // this is the amount you need to "add" to val to GET adjVal.
            //- if value GOING DOWN
            // like a type of derivative
            punchthru[idex].directedPunchthrus[dirIndex] = new IsosurfacePunchthru( dex, dirIndex, t, diff ) ;
            
            Vector3f voxelCenter = (offset + dex)*gridSizer ;
            Vector3f p2 = (offset + dex + dir)*gridSizer ;
            Vector3f p = Vector3f::lerp( t, voxelCenter, p2 ) ;
            pt( p, 0.25, Vector4f(0.8) ) ;
          }
        }
      }
    }
  }
}

void initDirections()
{
  IsosurfacePunchthruSet::addDirection( Vector3i( 1, 0, 0 ) ) ;  //0
  IsosurfacePunchthruSet::addDirection( Vector3i( 0, 1, 0 ) ) ;  //1
  IsosurfacePunchthruSet::addDirection( Vector3i( 0, 0, 1 ) ) ;  //2
  
  IsosurfacePunchthruSet::addDirection( Vector3i( -1,  0,  0 ) ) ; //3
  IsosurfacePunchthruSet::addDirection( Vector3i(  0, -1,  0 ) ) ; //4
  IsosurfacePunchthruSet::addDirection( Vector3i(  0,  0, -1 ) ) ; //5

  // UPPER NEIGHBOR, LEFT NEIGHBOUR,  LEFT NEIGHBOUR, LOWER NEIGHBOUR
  int px[4] = { 1, 2,  2, 4 } ;
  IsosurfacePunchthruSet::addNeighbours( 0, px, 4 ) ;
  
  // y has NO NEIGHBOURS listed.
  
  int pz[4] = { 1, 3,  3, 4 } ;
  IsosurfacePunchthruSet::addNeighbours( 2, pz, 4 ) ;

  int nx[4] = { 1, 5,  5, 4 } ;
  IsosurfacePunchthruSet::addNeighbours( 3, nx, 4 ) ;

  int nz[4] = { 1, 0,  0, 4 } ;
  IsosurfacePunchthruSet::addNeighbours( 5, nz, 4 ) ;

  for( int i = 0 ; i < IsosurfacePunchthruSet::Directions.size() ; i++ )
  {
    Vector3i p = IsosurfacePunchthruSet::Directions[i] + 1 ; // ADD ONE WHEN GETTING INDEX
    printf( "(%d,%d,%d) index %d\n", p.x, p.y, p.z, p.index( 3, 3 ) ) ;
  }
}



#endif