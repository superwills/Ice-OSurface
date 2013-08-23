#ifndef MESH_H
#define MESH_H

#include "Vectorf.h"
#include <vector>
using namespace std ;

Vector4f AxisEdgeColors[] = {
  Vector4f(1,0,0,1), Vector4f(1,0,0,1),
  Vector4f(0,1,0,1), Vector4f(0,1,0,1),
  Vector4f(0,0,1,1), Vector4f(0,0,1,1)
} ;

struct Mesh
{
  // Final array of vertices output by program (to draw)
  vector<VertexPNCT> verts ;
  vector<int> indices ;

  int renderMode ;

  Mesh()
  {
    renderMode = GL_TRIANGLES ; //default is triangles.
  }

  void createIndexBuffer()
  {
    vector<VertexPNCT> iVerts ;

    // now smooth the normals.
    iVerts.clear() ;
    
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
        indices.push_back( (int)iVerts.size() - 1 ) ;  // 
      }
      else
      {
        iVerts[jindex].normal += verts[i].normal ;
        indices.push_back( jindex ) ; // there is another index.
      }
    }
    
    // now fix the merged normals.
    for( int i = 0 ; i < iVerts.size() ; i++ )
      iVerts[i].normal.normalize() ;
    
    verts.swap( iVerts ) ;
  }

  void rebuild()
  {
    vector<VertexPNCT> rebuiltiVerts ;
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
          rebuiltIndices.push_back( (int)rebuiltiVerts.size() - 1 ) ;  // 
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

  }
  
  // Count of how many times each vertex hit an edge.
  // parallel array of NEIGHBOUR structure, tells you:
  // WHAT EDGE YOU'RE ON (if you are on an edge)
  // and WHO YOUR NEIGHBOUR IS
  // Neighbour is actually a bad name for this structure
  // more like Isopoint or SamePoint -- the LAST PT on the mesh
  // is exactly the same as the 1st pt, only translated +worldSize in x,y, or z
  vector< vector<int> > vertexWallHits ; // the numbers are WHICH EDGE you hit.
  vector< vector<int> > wallToVertexHits ; // maps PX=>list of verts on that edge.
  vector< vector<int> > vNeighbours ; // MAPS a vertex to the vertex indices that it TOUCHES
  // on the "other side".  CORNERS touch more than 1 vertex on more than 1 axis.
  
  // The voxelGrid object is needed only for getting WALL values
  void gatherEdgeData( VoxelGrid *voxelGrid )
  {
    wallToVertexHits.clear() ;
    vertexWallHits.clear() ;
    vNeighbours.clear() ;
    
    wallToVertexHits.resize( 6 ) ;
    vertexWallHits.resize( verts.size() ) ;
    vNeighbours.resize( verts.size() ) ;
    
    // Every vertex..
    for( int i = 0 ; i < verts.size() ; i++ )
    {
      // ..test proximity to ALL 6 walls (PX wall, NX wall, .. NZ wall)
      for( int j = 0 ; j < 6 ; j++ )
      {
        int axis=j/2; // PX=0,NX=1, so both map to index 0 (x-axis)
        
        //int neg=j%2 ; // negative axes (NX,NY,NZ) are the odd ones (1,3,5).
        //float worldEdge = (-2*neg + 1) * worldSize/2.f ; 
        float worldEdge = voxelGrid->wallValue( j ) ; // get the world edge for axis j
        
        // Now I'm going to test if this vertex is near the
        // PX,NX, wall (whatever wall j is)
        if( isNear( verts[i].pos.elts[axis], worldEdge, EPS ) )
        {
          // It is, so vertex i is on axis j.
          // "clean up" the edges by making it not just NEAR the world edge, but
          // exactly equal to the world edge on that axis.
          verts[i].pos.elts[axis] = worldEdge ;
          
          // Now remember that vertex i is on wall j
          vertexWallHits[i].push_back( j ) ;
          
          // store the reverse mapping as well
          wallToVertexHits[j].push_back( i ) ;  // wall j has vertex i
          
          // Can't find neighbour easily until ALL verts processed, it's faster to find
          // neighbours using the vertexWallHits collection.
        }
      }
    }
    
    
    bool showEdgeColors = 0 ;
    
    // Loudly shows you vertex neighbour finding errors in pink on black.
    // Actually shows that the errors are quite rare and don't produce very big holes!
    bool showErrors = 0 ;
    
    vector<int> errs ;
    // every vertex..
    for( int i = 0 ; i < verts.size() ; i++ )
    {
      // what walls is vertex `i` on?
      if( vertexWallHits[i].size() ) // could be 1 (edge), 2 (corner) or even 3 walls (absolute world corner points)
      {
        // change the color.
        if( showEdgeColors )  verts[i].color = Black ;

        // may be more than 1 wall:
        //for( int j = 0 ; j < vertexWallHits[i].size() ; j++ ) {
          //int axisEdge = vertexWallHits[i][j] ;
        for( int axisEdge : vertexWallHits[i] ) {
          if( showEdgeColors )  verts[i].color.xyz() += AxisEdgeColors[axisEdge].xyz() ;
        
          // get the vertex that neighbours this one on the repeated section
          int axis = axisEdge/2;
          int oAxis1 = OTHERAXIS1( axis ) ;
          int oAxis2 = OTHERAXIS2( axis ) ;

          int neg = axisEdge%2 ; // negative axes are the odd ones 1,3,5.
          int sign = -2*neg + 1 ; // 0=>+1, 1=>-1
          //float axisOffset = sign * worldSize ;  // to move from NX wall to PX, for example,
          // you ADD axisOffset (+worldSize) to elts[axis].
          // so (x,y,z) => (x+worldSize,y,z).
          
          // Check the elements on the OPPOSITE wall
          // To get the opposite walls:
          // EVEN: add one 0(PX)=>1(NX).  ODD: subtract one.  3(NY)=>2(PY)
          int oppositeWall = axisEdge + sign ;
          
          //float worldEdge = voxelGrid->wallValue( axisEdge ) ; // get the world edge for axisEdge
        
          // search the elements in the opposite wall for a vertex that matches mine
          // (this is much more efficient than searching ALL vertices!)
          for( int owv : wallToVertexHits[oppositeWall] )
          {
            // owv: oppositeWallVertexIndex
            
            // to check owi is my neighbour, move that opposite wall pt TO MY WALL,
            // and check if verts[i]==oppositeWallPt
            ////Vector3f oppositeWallPt = verts[owv].pos ;
            ////oppositeWallPt.elts[axis] = worldEdge ; // If I slam your axis to my side,
            // are you the same as me?  If yes, we are neighbours.
            ////if( verts[i].pos.isNear( oppositeWallPt, EPS ) )
            ////  vNeighbours[i].push_back( owv ) ;  // These are "touching" at the wrap point
            
            // OR you could just compare THE OTHER 2 AXES.
            if( isNear( verts[owv].pos.elts[oAxis1], verts[i].pos.elts[oAxis1], EPS ) &&
                isNear( verts[owv].pos.elts[oAxis2], verts[i].pos.elts[oAxis2], EPS ) )
              vNeighbours[i].push_back( owv ) ;  // These are "touching" at the wrap point
            
          }
        }
        
        // Sanity check.  I should have 
        if( vNeighbours[i].size() != vertexWallHits[i].size() )
        {
          // This happens if a break was introduced in the mesh
          //printf( "ERROR: vNeighbours[%d].size()(%lu) != vertexWallHits[%d].size()(%lu).\n",
          //  i, vNeighbours[i].size(), i, vertexWallHits[i].size() ) ;
          //verts[i].color = Magenta ; // errors shown below
          errs.push_back( i ) ;
          
          // Ok, if it couldn't find a neighbour, don't consider it a vertex wall hit then.
          // This keeps the error from compounding, especially if marching cubes produced
          // a mesh with one of these errors in it to begin with (which it sometimes does)
          for( int wall : vertexWallHits[ i ] )
            for( int j = 0 ; j < wallToVertexHits[wall].size() ; j++ )
              if( wallToVertexHits[wall][j] == i )
                wallToVertexHits[wall].erase( wallToVertexHits[wall].begin() + j ) ;
            
          vertexWallHits[ i ].clear() ;
          
        }
      }
    } // end every vertex
    
    if( errs.size() ) {
      printf( "%lu vertices didn't find neighbour-friends :(\n"
        //"This happens because a neighbour was decimated in a previous stage merge.\n"
        //"This just means your mesh has a (probably) small hole/discontinuity.\n"
        //"Turn on `showErrors` in the code to see where the problem is.\n",
        , errs.size() ) ;
      
      if( showErrors )
      {
        for( int i = 0 ; i < verts.size() ; i++ )
          verts[i].color = Black ;
        for( int i = 0 ; i < errs.size() ; i++ )
          verts[errs[i]].color = Magenta ;
      }
      
    }
  }
  
  /// Make the edge normals continuous and smooth
  void smoothEdgeNormals()
  {
    // This DOES visit each vertex pair twice, but to no ill effect.
    for( int i = 0 ; i < vNeighbours.size() ; i++ )
    {
      for( int neighbour : vNeighbours[i] )
      {
        verts[i].normal += verts[neighbour].normal ;
      }
      
      //if( vNeighbours[i].size() == 2 )  verts[i].color = Yellow ; // show we detect corners properly
      //else if( vNeighbours[i].size() == 3 )  verts[i].color = White ; // rare total corner
      
      verts[i].normal.normalize() ;
      //verts[i].color = verts[i].color = Vector4f::random() ; // ensure it is behaving correctly
    }
  }
  
private:
  // basically set ALL use of i2=i1.
  inline void changeUseOf( int i1, int i2 )
  {
    // EVERYBODY that used i2 now uses i1.  i2 is discarded.
    for( int j = 0 ; j < indices.size() ; j++ )
      if( indices[j] == i2 )
        indices[j]=i1 ;
    
    // Any vertex that had i2 as a neighbour no longer has i2 as a neighbour
    for( int i = 0 ; i < vNeighbours.size() ; i++ )
      for( int j = 0 ; j < vNeighbours[i].size() ; j++ )
        if( vNeighbours[i][j] == i2 )
          vNeighbours[i].erase( vNeighbours[i].begin() + j ) ;  ///these are small arrays so the performance hit is small
  }
  
  // Merges 2 indices into 1 vertex, COLLAPSING AN EDGE.
  // the vertices at i1 and i2 merge to the geometric center between them
  inline void mergeToCenter( int i1, int i2 )
  {
    // get new avg value
    Vector3f newPt = ( verts[i1].pos + verts[i2].pos ) / 2 ;
    Vector3f newNormal = ( verts[i1].normal + verts[i2].normal ).normalize() ;
    
    verts[i1].pos=newPt;
    verts[i1].normal=newNormal;
    
    changeUseOf( i1, i2 ) ;
  }
  
  // merges i2 into i1.
  inline void mergeToFirst( int i1, int i2 )
  {
    // Avg the normal, but DO NOT move i1's pos
    verts[i1].normal = ( verts[i1].normal + verts[i2].normal ).normalize() ;
    changeUseOf( i1, i2 ) ;
  }
  
  

public:
  // "smooths" the mesh by removing small EDGES
  // first it creates the index buffer, then it works from there
  // to eliminate short edges.
  void smoothMesh( VoxelGrid *voxelGrid, float minEdgeLength )
  {
    createIndexBuffer() ;
    gatherEdgeData( voxelGrid ) ;
    
    // If you want to smooth edge normals before actual mesh smoothing, it must be done here.
    smoothEdgeNormals() ;
    
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
        
        // let i1 be the "higher degree" index (one with more neighbours)
        if( vertexWallHits[i2].size() > vertexWallHits[i1].size() )
        {
          swap( i1, i2 ) ;
        }
        
        Vector3f a=verts[i1].pos, b=verts[i2].pos;
        Vector3f edge = a-b ;
      
        // tolerance is the max size edge to collapse
        if( edge.len() < minEdgeLength )
        {
          // If the number of wall hits is EQUAL (which maxes out at 3), use
          // an averaging of the vertices.
          if( vertexWallHits[i1].size() == vertexWallHits[i2].size() )
          {
            // MUST BE from the same wall.
            bool sameWall=1;
            for( int wall : vertexWallHits[i1] )
            {
              if( !contains( vertexWallHits[i2], wall ) )
              {
                // Cannot allow this type of merge.
                //printf( "The two vertices share the same # walls (%ld), BUT THE WALL "
                //  "VERTEX i1 IS ON (%d) IS NOT SHARED BY i2. Thus these vertices cannot merge.\n",
                //  vertexWallHits[i1].size(), wall ) ;
                //verts[i1].color = Magenta ;
                sameWall=0;
                break ;
              }
            }
            
            // Not the same wall.  Skip the merge.
            if( !sameWall ) skip ;
            
            // You do the same thing for #edge wall hits=0,1,2.
            mergeToCenter( i1, i2 ) ;

            // in case it IS an edge vertex,
            // We force a merge on all other sides to happen NOW
            // because it's possible that for the repeat side,
            // some internal edges will merge FIRST, before that
            // repeat side gets a chance to, creating holes.
            if( vNeighbours[i1].size() == vNeighbours[i2].size() ) // this is NOT true when there's ALREADY a hole in the mesh
            {
              for( int n = 0 ; n < vNeighbours[i1].size() ; n++ )
              {
                // do a regular edge merge for ni1, ni2.
                int ni1=vNeighbours[i1][n],ni2=vNeighbours[i2][n] ;
                //verts[ni1].color = Green ;
                
                // This prevents breaks from being introduced in the mesh
                mergeToCenter( ni1, ni2 ) ;
              }
            }
          }
          else if( vertexWallHits[i1].size() > vertexWallHits[i2].size() )
          {
            // merge towards HIGHER DEGREE edge (we made degree i1 always > degree i2 w/swap above).
            mergeToFirst( i1, i2 ) ;
            
          }
        }
      }
    }

    // Don't bother smoothing edge normals until downsampling is over
    gatherEdgeData( voxelGrid ) ;
    smoothEdgeNormals() ;
    
    // You need to REBUILD the mesh now becaue the deleted vertices
    // that are never references STILL TAKE UP SPACES.  deleting them
    // shifts the array, so changes ALL the indices after the deleted elt.
    rebuild() ;

    //printf( "%d vertices converted to %d vertices and %d indices\n", verts.size(), iVerts.size(), indices.size() ) ;
  }

  void texture()
  {
    // Here I will create a very large GL texture.
    // 3D perlin noise will be used to generate the values here.

    // (not done)
  }
  
  
  // Generates per-vertex colors using perlin noise and a cubic spline
  // also generates texcoords for procedural detail tex
  void vertexTexture( float wTexture, int wTexturePeriod, const Vector3f& worldSize, int textureRepeats )
  {
    // this makes the texture repeat (textureRepeats) times across the world
    Vector2f texScale = Vector2f(textureRepeats) / worldSize.xy() ;
    
    for( int i = 0 ; i < verts.size() ; i++ )
    {
      Vector3f sp = verts[i].pos ;
      
      //sp.normalize() ;
      sp /= worldSize ;
      
      // The color comes out of the perlin noise mapping from the 3-space position,
      // so it varies smoothly in 3 space.
      //float n = Perlin::pnoise( sinf(sp.x), cosf(2*sp.y), sp.z, tw.x, 2,2,2,2 ) ;
      //float n = Perlin::pnoise( sinf(2*M_PI*sp.x), cosf(2*M_PI*sp.y), sp.z, tw.x, 2,2,1,8 ) ;
      float n = Perlin::pnoise( sp.x, sp.y, sp.z, wTexture, 1,1,1,wTexturePeriod ) ;

      //Vector3f color(
      //  Perlin::noise( sp.x, sp.y, sp.z, tw.x ),
      //  Perlin::noise( sp.x, sp.y, sp.z, tw.y ),
      //  Perlin::noise( sp.x, sp.y, sp.z, tw.z )
      //) ;
      Vector3f color = Vector3f::cubicSpline( n, Vector3f( 0.45,0.34,0.54 ),
        Vector3f( 0.87,0.1,0 ),
        Vector3f( 0.66,0.24,0.2 ),
        Vector3f( 0.55,0.21,0.1 )
      ) ;
      
      // Negative color is undefined
      color.fabs() ;

      // No extreme colors
      //color.clampLen( 0.45f, 0.65f ) ;
      
      // Give each vertex this uniqueish smooth color
      verts[i].color.xyz() = color ;
      
      
      
      // Figure out a reasonable tiling on the surface.
      verts[i].tex = Vector2f( randInt(0, 2), randInt( 0, 2 ) ) ;
      
      

      
      //verts[i].tex = Vector2f::random() ;
      verts[i].tex = verts[i].pos.xy() * texScale ; 
      //verts[i].tex.fabs() ;
    }
  }
  
  
} ;


#endif