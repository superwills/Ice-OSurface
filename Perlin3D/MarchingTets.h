#ifndef MARCHINGTETS_H
#define MARCHINGTETS_H

#include "MarchingCommon.h"

struct MarchingTets : public IsosurfaceFinder
{
  bool SOLID ;

  MarchingTets( VoxelGrid *iVoxelGrid, vector<VertexPNC>* iVerts, float iIsosurface, const Vector4f& color ) :
    IsosurfaceFinder( iVoxelGrid, iVerts, iIsosurface, color )
  {
    SOLID=0;
  }

  void cutTet1Out( const Vector3i& A, const Vector3i& B, const Vector3i& C, const Vector3i& D )
  {
    // D is OUT OF SURFACE.
    // Get the 3 cut points
    Vector3f cutAD = voxelGrid->getCutPoint( isosurface, A, D ) ;
    Vector3f cutCD = voxelGrid->getCutPoint( isosurface, C, D ) ;
    Vector3f cutBD = voxelGrid->getCutPoint( isosurface, B, D ) ;

    if( SOLID )
      Geometry::triPrism( *verts, voxelGrid->getP(A), voxelGrid->getP(B), voxelGrid->getP(C),
                          cutAD, cutCD, cutBD, baseColor ) ;
    else
      Geometry::addTriWithNormal( *verts, cutAD, cutCD, cutBD, baseColor ) ; // SHOW ONLY THE CUT FACE
  
  }

  // ABC wound CCW, D is out.
  void cutTet2Out( const Vector3i& A, const Vector3i& B, const Vector3i& C, const Vector3i& D )
  {
    // C,D is OUT OF SURFACE.

    // Get the 4 cut points
    Vector3f cutAC = voxelGrid->getCutPoint( isosurface, A, C ) ;
    Vector3f cutAD = voxelGrid->getCutPoint( isosurface, A, D ) ;
    Vector3f cutBC = voxelGrid->getCutPoint( isosurface, B, C ) ;
    Vector3f cutBD = voxelGrid->getCutPoint( isosurface, B, D ) ;

    if( SOLID )
      Geometry::triPrism( *verts, voxelGrid->getP(B), cutBD, cutBC,   voxelGrid->getP(A), cutAC, cutAD, baseColor ) ;
    else
    {
      Geometry::addTriWithNormal( *verts, cutAC, cutBC, cutBD, baseColor ) ;
      Geometry::addTriWithNormal( *verts, cutAC, cutBD, cutAD, baseColor ) ;
    }
  }

  void cutTet3Out( const Vector3i& A, const Vector3i& B, const Vector3i& C, const Vector3i& D )
  {
    // A is in. the rest are out
    Vector3f cutAB = voxelGrid->getCutPoint( isosurface, A, B ) ;
    Vector3f cutAC = voxelGrid->getCutPoint( isosurface, A, C ) ;
    Vector3f cutAD = voxelGrid->getCutPoint( isosurface, A, D ) ;

    if( SOLID )
    {
      Geometry::addTet( *verts, voxelGrid->getP(A), cutAB, cutAC, cutAD, baseColor ) ;
    }
    else
      Geometry::addTriWithNormal( *verts, cutAB, cutAD, cutAC, baseColor ) ;
  }

  void tet( const Vector3i& A, const Vector3i& B, const Vector3i& C, const Vector3i& D )
  {
    float vA = (*voxelGrid)( A ).v ;
    float vB = (*voxelGrid)( B ).v ;
    float vC = (*voxelGrid)( C ).v ;
    float vD = (*voxelGrid)( D ).v ;
  
    if( inSurface( vA ) && inSurface( vB ) && inSurface( vC ) && inSurface( vD ) )
    {
      // REMOVE to just have a shell.
      //if( !tooDeep( vA ) && tooDeep( vB ) && tooDeep( vC ) && tooDeep( vD ) )
      //Geometry::addTet( *verts, getP(A), getP(B), getP(C), getP(D), Vector4f( 0,0,1,1 ) ) ;
    }

    // 3 are in the surface
    else if( inSurface( vA ) && inSurface( vB ) && inSurface( vC ) )
      cutTet1Out( A, B, C, D ) ;
    else if( inSurface( vA ) && inSurface( vD ) && inSurface( vB ) )
      cutTet1Out( A, D, B, C ) ;
    else if( inSurface( vB ) && inSurface( vD ) && inSurface( vC ) )
      cutTet1Out( B, D, C, A ) ;
    else if( inSurface( vA ) && inSurface( vC ) && inSurface( vD ) )
      cutTet1Out( A, C, D, B ) ;

    // 2 in
    else if( inSurface( vA ) && inSurface( vB ) )
      cutTet2Out( A, B, C, D ) ;
    // ASSUMING ORDER DOESN'T MATTER.  IF BACKWARDS TURN AROUND.
    else if( inSurface( vA ) && inSurface( vC ) )
      cutTet2Out( A, C, D, B ) ;
    else if( inSurface( vA ) && inSurface( vD ) )
      cutTet2Out( A, D, B, C ) ;
    else if( inSurface( vB ) && inSurface( vC ) )
      cutTet2Out( B, C, A, D ) ;
    else if( inSurface( vB ) && inSurface( vD ) )
      cutTet2Out( B, D, C, A ) ;
    else if( inSurface( vC ) && inSurface( vD ) )
      cutTet2Out( C, D, A, B ) ;
  
    else if( inSurface( vA ) )
      cutTet3Out( A, B, C, D ) ;
    else if( inSurface( vB ) )
      cutTet3Out( B, A, D, C ) ;
    else if( inSurface( vC ) )
      cutTet3Out( C, A, B, D ) ;
    else if( inSurface( vD ) )
      cutTet3Out( D, B, A, C ) ;
  }

  void genVizMarchingTets()
  {
    for( int k = 0 ; k < voxelGrid->dims.z ; k++ )
    {
      for( int j = 0 ; j < voxelGrid->dims.y ; j++ )
      {
        for( int i = 0 ; i < voxelGrid->dims.x ; i++ )
        {
          Vector3i dex( i,j,k ) ;
          Vector3i A=dex+Vector3i(0,0,0), B=dex+Vector3i(0,0,1), C=dex+Vector3i(0,1,0), D=dex+Vector3i(0,1,1),
                   E=dex+Vector3i(1,0,0), F=dex+Vector3i(1,0,1), G=dex+Vector3i(1,1,0), H=dex+Vector3i(1,1,1);
        
          tet( A, B, D, E ) ;
          tet( A, D, C, E ) ;
          tet( D, G, C, E ) ;
          tet( D, H, G, E ) ;
          tet( B, F, D, E ) ;
          tet( F, H, D, E ) ;
        }
      }
    }
  }
} ;

#endif