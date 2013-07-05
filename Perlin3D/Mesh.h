#ifndef MESH_H
#define MESH_H

#include "Vectorf.h"
#include <vector>
using namespace std ;

struct Mesh
{
  // Final array of vertices output by program (to draw)
  vector<VertexPNC> verts ;
  vector<int> indices ;

  int renderMode ;

  static float minEdgeLength ;   // the minimum ALLOWED edge length before the edge gets removed.

  Mesh()
  {
    renderMode = GL_TRIANGLES ; //default is triangles.
  }

  void createIndexBuffer()
  {
    vector<VertexPNC> iVerts ;

    // now smooth the normals.
    iVerts.clear() ;
    indices.clear() ;

    // first I merge into an index buffer.
    for( int i = 0 ; i < verts.size() ; i++ )
    {
      int jindex = -1 ;
      for( int j = 0 ; j < iVerts.size() && jindex==-1 ; j++ )
        if( iVerts[j].pos.isNear( verts[i].pos, EPS ) )
          jindex = j ; // verts[i] already exists at jindex
    
      if( jindex==-1 )
      {
        iVerts.push_back( verts[i] ) ; // another vertex
        indices.push_back( iVerts.size() - 1 ) ;  // 
      }
      else
      {
        iVerts[jindex].normal += verts[i].normal ;
        indices.push_back( jindex ) ; // there is another index.
      }
    }
    for( int i = 0 ; i < iVerts.size() ; i++ )
      iVerts[i].normal.normalize() ;
    
    verts.swap( iVerts ) ;
  }

  // "smooths" the mesh by removing small EDGES
  // first it creates the index buffer, then it works from there
  // to eliminate short edges.
  void smoothMesh( const VoxelGrid& voxelGrid )
  {
    createIndexBuffer() ;

    // parallel array of NEIGHBOUR structure, tells you:
    // WHAT EDGE YOU'RE ON (if you are on an edge)
    // and WHO YOUR NEIGHBOUR IS
    // Neighbour is actually a bad name for this structure
    // more like Isopoint or SamePoint -- the LAST PT on the mesh
    // is exactly the same as the 1st pt, only translated +worldSize in x,y, or z

    // Count of how many times each vertex hit an edge.
    vector< vector<int> > vertexWallHits ; // the numbers are WHICH EDGE you hit.
    vector< vector<int> > wallToVertexHits ; // maps PX=>list of verts on that edge.

    vertexWallHits.resize( verts.size() ) ;
    wallToVertexHits.resize( 6 ) ;

    for( int i = 0 ; i < verts.size() ; i++ )
    {
      for( int j = PX ; j <= NZ ; j++ )
      {
        int axis=j/2;
        //int neg=j%2 ; // negative axes are the odd ones 1,3,5.
        //float worldEdge = (-2*neg + 1) * worldSize/2.f ; 
        float worldEdge = voxelGrid.wallValue( j ) ;

        if( isNear( verts[i].pos.elts[axis], worldEdge, EPS ) )
        {
          // You're on this axis.
          // "clean up" the edges.
          verts[i].pos.elts[axis] = worldEdge ;

          vertexWallHits[i].push_back( j ) ;
          wallToVertexHits[j].push_back( i ) ; // store the reverse mapping as well
        }
      }
    }
  
    bool showEdgeColors=0;
    for( int i = 0 ; i < verts.size() ; i++ )
    {
      if( vertexWallHits[i].size() )
      {
        // change the color.
        if( showEdgeColors )  verts[i].color = Black ;

        for( int axisEdge : vertexWallHits[i] )
        {
          if( showEdgeColors )  verts[i].color.xyz() += AxisEdgeColors[axisEdge].xyz() ;
        
          // get the neighbour and avg the normal
          int axis= axisEdge/2;
          int oAxis1 = OTHERAXIS1( axis ) ;
          int oAxis2 = OTHERAXIS2( axis ) ;

          int neg = axisEdge%2 ; // negative axes are the odd ones 1,3,5.
          int sign = -2*neg + 1 ; // 0=>+1, 1=>-1
          //float axisOffset = sign * worldSize ;  // to move from NX wall to PX, for example,
          // you ADD axisOffset (+worldSize) to elts[axis].
          // so (x,y,z) => (x+worldSize,y,z).
        
          // Check the elements on the OPPOSITE wall
          // EVEN: add one 0(PX)=>1(NX).  ODD: subtract one.  3(NY)=>2(PY)
          int oppositeWall = axisEdge + sign ;

          // search the elements in the opposite wall for a vertex that matches mine
          for( int owv : wallToVertexHits[oppositeWall] )
          {
            // owi: oppositeWallVertexIndex
            // MOVE that opposite wall pt TO MY WALL
            //oppositeWallPt.elts[axis] += axisOffset ;
            //OR you could just compare THE OTHER 2 AXES.
            if( isNear( verts[owv].pos.elts[oAxis1], verts[i].pos.elts[oAxis1], EPS ) &&
                isNear( verts[owv].pos.elts[oAxis2], verts[i].pos.elts[oAxis2], EPS ) )
            {
              // These are "touching" at the wrap point
              verts[i].normal += verts[owv].normal ;
              //verts[i].color = verts[owv].color = Vector4f::random() ; // ensure it is behaving correctly
            }
          }
        }

        verts[i].normal.normalize() ;
      }
    }
  
    // Now we can downsample the mesh.
    // You can only merge EDGES.
    // attempt to reduce small triangles to degeneracy (actually sharing all 3 pts)
    for( int indexNo = 0 ; indexNo < indices.size() ; indexNo+=3 )
    {
      // 3 edges per tri (group of 3 verts)
      for( int eNo=0 ; eNo < 3 ; eNo++ )
      {
        // [0,1], [1,2], [2,0]
        int i1 = indices[indexNo + eNo] ;
        int i2 = indices[indexNo + (eNo+1)%3] ;

        Vector3f a=verts[i1].pos, b=verts[i2].pos;
        Vector3f edge = a-b ;
      
        // tolerance is the max size edge to collapse
        if( edge.len() < minEdgeLength )
        {
          // get new avg value
          Vector3f newPt = ( a+b ) / 2 ;
          Vector3f newNormal = ( verts[i1].normal + verts[i2].normal ).normalize() ;

          // if a vertex from the (i1,i2) pair is ON A CORNER, then you MUST merge INTO THE CORNER
          // (ie keep the corner vertex and delete the other one).
          if( vertexWallHits[i1].size() >= 2 )
          {
            /// i1 is a CORNER
            newPt = a;//i1
          }
          else if( vertexWallHits[i2].size() >= 2 )
          {
            /// i2 is a CORNER
            newPt = b;//i2
          }
          else if( vertexWallHits[i1].size() == 1 && vertexWallHits[i2].size() == 1 )
          {
            // if both vertices are on the SAME EDGE, then merge normally.
            if( vertexWallHits[i1][0] == vertexWallHits[i2][0] ) // same edge
            {
              // The same merge will happen on the opposite side and it won't move
              // AWAY from the edge as a result of the merge at all.
              newPt = ( a+b ) / 2 ;
            }
            else
            {
              // well, you're trying to collapse an edge that straddles a corner.
              // this is not allowed
              //iVerts[i1].color=iVerts[i2].color=Red ; // so you can see this was skipped.
              skip ;
            }
          }
          else if( vertexWallHits[i1].size() == 1 ) // ONLY a is a wall hit
          {
            newPt = a ;
          }
          else if( vertexWallHits[i2].size() == 1 ) // ONLY b is a wall hit
          {
            newPt = b ;
          
            // Actually here we will use i2 instead of i1
            swap( i1, i2 ) ;
          }
        
          verts[i1].pos = newPt ;
          verts[i1].normal = newNormal ;

          // EVERYBODY that used i2 now uses i1
          for( int j = 0 ; j < indices.size() ; j++ )  if( indices[j] == i2 )  indices[j]=i1 ;
        }
      }
    }

    // You need to REBUILD the mesh now becaue the deleted vertices
    // that are never references STILL TAKE UP SPACES.  deleting them
    // shifts the array, so changes ALL the indices after the deleted elt.
    vector<VertexPNC> rebuiltiVerts ;
    vector<int> rebuiltIndices ;
  
    // if its not referenced it gets left out of the rebuild
    // if it is DEGENERATE then it gets left out of the rebuild.
    // Go in 3's, test the tri for degeneracy (by checking if 2 indices are equal).
    for( int indexNo = 0 ; indexNo < indices.size() ; indexNo+=3 )
    {
      int ixs[3] = { indices[indexNo], indices[indexNo+1], indices[indexNo+2] } ;
      // If any of them are the same, SKIP/LEAVE OUT because its a degenerate face.
      if( ixs[0] == ixs[1] || ixs[0] == ixs[2] || ixs[1] == ixs[2] )  skip ;

      // Triangle Ok.  process each index individually.
      for( int iNo=0 ; iNo < 3 ; iNo++ )
      {
        int jindex = -1 ;
      
        // Look for a vertex near iVerts[indices[i]]
        for( int j = 0 ; j < rebuiltiVerts.size() && jindex==-1 ; j++ )
          if( rebuiltiVerts[j].pos.isNear( verts[ ixs[iNo] ].pos ) )
            jindex = j ; // iVerts[ indices[i] ] already exists at jindex
    
        if( jindex==-1 )
        {
          rebuiltiVerts.push_back( verts[ ixs[iNo] ] ) ; // another vertex
          rebuiltIndices.push_back( rebuiltiVerts.size() - 1 ) ;  // 
        }
        else
        {
          //rebuiltiVerts[jindex].normal += iVerts[ indices[i] ].normal ;
          rebuiltIndices.push_back( jindex ) ; // there is another index just into this same vertex
        }
      }
    }

    verts.swap( rebuiltiVerts ) ;
    indices.swap( rebuiltIndices ) ;

    //printf( "%d vertices converted to %d vertices and %d indices\n", verts.size(), iVerts.size(), indices.size() ) ;
  
  }

  void rebuild()
  {
  }
} ;

float Mesh::minEdgeLength=0.1f ;

#endif