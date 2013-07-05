#ifndef MARCHINGCUBES_H
#define MARCHINGCUBES_H

// These are CW faces, LEFT, TOP, RIGHT.
// If you wind a face using the order here, the face will be facing INTO the cube.
static int adj[8][3] = {
  { 4,2,1 },{ 0,3,5 },{ 3,0,6 },{ 7,1,2 },
  { 5,6,0 },{ 1,7,4 },{ 2,4,7 },{ 6,5,3 }
} ; // each of the 8 verts has 4 neighbours. always.

struct MarchingCubes
{
  // The voxel grid I am operating on.
  VoxelGrid *voxelGrid ;

  // Pointers to arrays in caller program space
  vector<VertexPNC> *verts ;

  Vector4f cubeColor ;
  bool SOLID ;

  MarchingCubes(  VoxelGrid *iVoxelGrid, vector<VertexPNC>* iVerts, float iIsosurface, const Vector4f& color )
  {
    voxelGrid = iVoxelGrid ;
    verts = iVerts ;
    isosurface = iIsosurface ;
    cubeColor = color ;
  }

  /// MARCHING CUBES
  // Neighbours are in the order a facing out tri should be wound
  // I only need to know these to gen an isosurface.
  //    2----6
  //   /|   /|
  //  3-0--7 4
  //  |/   |/
  //  1----5
  void cornerTri( Vector3i* pts, int a, bool rev, Vector4f color )
  {
    // assumes cut1, cut2, cut3 specifies a CCW face.

    //!! THERE IS A PROBLEM HERE!!!   when a is a "TOP" tri, its verts are
    // specified 0(left),1(bottom),2(right) which winds a CW face facing the vertx in quesiton.
    //if(a==2||a==3||a==6||a==7)  color=Vector4f(1,1,1,1) ;
    //else  color=Yellow;
    //!! Actually the above lines show that it ISN'T a problem.
    // The reason is I reverse the winding of top tris by reversing the ORDER
    // when the UP axis is chosen.
    Vector3f cut1 = voxelGrid->getCutPoint( isosurface, pts[a], pts[ adj[a][0] ] ) ;
    Vector3f cut2 = voxelGrid->getCutPoint( isosurface, pts[a], pts[ adj[a][1] ] ) ;
    Vector3f cut3 = voxelGrid->getCutPoint( isosurface, pts[a], pts[ adj[a][2] ] ) ;

    if( !rev )
      Geometry::addTriWithNormal( *verts, cut2,cut1,cut3, color ) ; //So,
      // the default winding is 0,1,2, which is LEFT, UP, RIGHT.
      // If i'm the vertex, then the tri I draw (left,up,right) is CW
      // so its FACING AWAY from me.  But I want the default to have
      // the tri face THE PIONT IN QUESTION.  So its reversed here ;).
    else // REVERSED WINDING ORDER
      Geometry::addTriWithNormal( *verts, cut1,cut2,cut3, color ) ;

  }

  // 2 inversion tricks for making sure the face wound is ccw
  void benchReady( Vector3i* pts, int ia, int ib, vector<int>& nia, vector<int>& nib )
  {
    // This is the "trick" that makes the benches orient the right way (CCW).
    // if ia is 0, (b is at a's left), or 2 (b is at a's right) then the quad spun below will be the right way.

    // but basically WHEN a's neighbour (b) is on the UP axis,
    // you have to SWAP the default order of the 2 that are NOT
    // b.
    // modulus did this automagically, but I prefer this way.
  
    if( ia == 1 )
      swap(nia[0],nia[1]) ;

    // For the 2nd part, this checking MUST be done with the CONTEXT OF THE POINTS.

    // I will be drawing a quad on the cutpoints.
    // `a` has 2 adjacent pts in nia, and so does b in nib.
    // So I don't get a "butterfly" (twisted quad),
    // If the LAST 2 don't share an edge, make them.
    // This ensures that they are on the same face,
    if( !pts[nia[1]].twoEqual(pts[nib[1]]) )
      swap(nib[0],nib[1]);
  }

  // nia and nib produced by adjacencyOf2()
  void benchTris( Vector3i* pts, int a, int b, int &ia, int &ib, vector<int> &nia, vector<int> &nib, bool rev, const Vector4f& color ) 
  {
    benchReady( pts, ia, ib, nia, nib ) ;

    Vector3f cutA1 = voxelGrid->getCutPoint( isosurface, pts[ a ], pts[ adj[ a ][nia[0]] ] ) ;
    Vector3f cutA2 = voxelGrid->getCutPoint( isosurface, pts[ a ], pts[ adj[ a ][nia[1]] ] ) ;
    Vector3f cutB1 = voxelGrid->getCutPoint( isosurface, pts[ b ], pts[ adj[ b ][nib[0]] ] ) ;
    Vector3f cutB2 = voxelGrid->getCutPoint( isosurface, pts[ b ], pts[ adj[ b ][nib[1]] ] ) ;

    if( !rev )
      Geometry::addQuadWithNormal( *verts, cutA1,cutB1,cutB2,cutA2, color ) ;  

    else // REVERSED WINDING ORDER
      Geometry::addQuadWithNormal( *verts, cutA1,cutA2,cutB2,cutB1, color ) ;

  } ;

  // tells you if a&b are adjacent to each other.
  // returns the INDEX of the adjacency for a and b.
  // Does not need/know about the actual pts values (though that is needed actually for
  // full determination)
  int adjacencyOf2( Vector3i* pts, int a, int b, bool revs )
  {
    int ia=-1, ib=-1 ;
    vector<int> nia, nib;

    for( int i = 0 ; i < 3 ; i++ )
    {
      if( adj[a][i] == b )  ia=i; // which index in your adj list is `b`?
      else  nia.push_back( i ) ;  // `i` is an adjacent edge that is NOT `b`
    
      if( adj[b][i] == a )  ib=i;
      else  nib.push_back( i ) ;
    }

    if( ia != -1 )
    {
      benchTris( pts, a,b, ia,ib, nia,nib, revs, cubeColor ) ;
      return 1;
    }
    else
    {
      // they're not adjacent. 2 edge tris
      cornerTri( pts, a, revs, cubeColor ) ;
      cornerTri( pts, b, revs, cubeColor ) ;
      return 0 ;
    }
  }

  // This is a token pasting macro that performs the
  // 3 required swaps (a, ia, and nia).  This means
  // the naming convention cannot change.
  // People will complain, but using this macro helps avoid typos
  // and see what the code is doing
  #define SWAP( a,b ) swap(a,b), i##a.swap(i##b), ni##a.swap(ni##b)

  // forces them to SHARE the index in question.
  // This is done to fix winding order.
  // NON is the index NOT to be on
  // ONI is the index TO be on.
  // You only have to do this when both nia and nib are of SIZE 2,
  // and they share a vertex index, but you're not sure what index its on.
  void forceShare( int a, int b, vector<int>& nia, vector<int>& nib, int NON, int ONI )
  {
    if( nia.size() != 2 || nib.size() != 2 )
    {
      error( "nia=%d, nib=%d, must be 2,2", nia.size(), nib.size() ) ;
      return ;
    }
    // Equality checks the vertex index is the same here.
    if( adj[ a ][ nia[NON] ] == adj[ b ][ nib[NON] ] ) // shared vertex was index NON
      swap( nia[NON], nia[ONI] ), swap( nib[NON], nib[ONI] ) ; // make it index ONI for both.
    else if( adj[ a ][ nia[NON] ] == adj[ b ][ nib[ONI] ] ) // nia was wrong
      swap( nia[NON], nia[ONI] ) ;
    else if( adj[ a ][ nia[ONI] ] == adj[ b ][ nib[NON] ] ) // nib was wrong
      swap( nib[NON], nib[ONI] ) ;
  }

  // I will tell you the adjacency of 3 verts
  int adjacencyOf3( Vector3i *pts, int& a, int& b, int& c, bool revs )
  {
    vector<int> ia, ib, ic, nia, nib, nic ;

    // there are 3 possibilities:
    // 1) all 3 are off by themselves
    //   RET 0, NO ADJACENCY

    // 2) 2 share an edge, 1 is off by itself.
    //   RET 1, ia[0] and ib[0] have the adjacent tris.  c is the one off by itself.

    // 3) all 3 are adjacent.  this happens iff all 3 are on the same FACE.
    //   RET 2, a is the CENTER, b & c are the sides.
  
    // if `a` has `b` as its `ith` adjacency, save that info
    // So evaluate each of the items in my adjacency list.
    // For a, is the 0th adjacent vertex B or C?  If it is,
    // then save taht info.

    // IA contains adjacencies to a THAT ARE other pts (ie b or c)
    // NIA contains adjacencies that ARE NOT other pts b or c.
    // ia.size() + nia.size() always == 3
    for( int i = 0 ; i < 3 ; i++ )
    {
      if( adj[a][i] == b || adj[a][i] == c )
        ia.push_back( i ) ;   // adj[a][i] IS either b or c.
      else
        nia.push_back( i ) ;  // adj[a][i] is NOT b or c.

      if( adj[b][i] == a || adj[b][i] == c )
        ib.push_back( i ) ;
      else
        nib.push_back( i ) ;

      if( adj[c][i] == a || adj[c][i] == b )
        ic.push_back( i ) ;
      else
        nic.push_back( i ) ;
    }

    // Now evaluate the results
    if( ib.size() == 2 )
    {
      SWAP( a, b ) ;
    }
    else if( ic.size() == 2 )
    {
      SWAP( a, c ) ;
    }
  
    // WIND TRIS.
    if( ia.size() == 2 )
    {
      // they all have adj, ib and ic have 2 adj pts
      // ia is the center.
      // the other 2 must have their _2nd_ point, the shared point.
      // b's adjacent 2nd should be the same as c's adjacent 2nd.
      forceShare( b,c, nib, nic, 0, 1 ) ;
    
      // must use cross product.  too many combinations
      // This forces B to be the CCW neighbour of A (and never C).
      Vector3i AB = pts[b] - pts[a] ;
      Vector3i AC = pts[c] - pts[a] ;
      Vector3i n = AB.cross( AC ) ;
      int dPlane = -n.dot( pts[a] ) ;
      if( n.dot( pts[ adj[a][ nia[0] ] ] ) + dPlane > 0 )
      {
        SWAP( b,c ) ;
      }

      Vector3f cutA  = voxelGrid->getCutPoint( isosurface, pts[ a ], pts[ adj[a][ nia[0] ] ] ) ;
      Vector3f cutB1 = voxelGrid->getCutPoint( isosurface, pts[ b ], pts[ adj[b][ nib[0] ] ] ) ;
      Vector3f cutB2 = voxelGrid->getCutPoint( isosurface, pts[ b ], pts[ adj[b][ nib[1] ] ] ) ;
      Vector3f cutC1 = voxelGrid->getCutPoint( isosurface, pts[ c ], pts[ adj[c][ nic[0] ] ] ) ;
      Vector3f cutC2 = voxelGrid->getCutPoint( isosurface, pts[ c ], pts[ adj[c][ nic[1] ] ] ) ;
      
      if( !revs )
      {
        Geometry::addPentagonWithNormal( *verts, cutA, cutB1, cutB2, cutC2, cutC1, cubeColor ) ;
      }
      else
      {
        Geometry::addPentagonWithNormal( *verts, cutA, cutC1, cutC2, cutB2, cutB1, cubeColor ) ;
      }

      return 2 ;
    }
  
    // 1
    else if( ia.size() == 1 )
    {
      if( !ib.size() )
      {
        SWAP( b,c ) ;
      }
    }
    else if( ib.size() == 1 || ic.size() == 1 )
    {
      if( !ia.size() )
      {
        // a got stuck as the loner. make c the loner
        SWAP( a,c ) ;
      }
    }

    if( ia.size() == 1 )
    {
      // 
      benchTris( pts, a, b, ia[0], ib[0], nia, nib, revs, cubeColor ) ;
      cornerTri( pts, c, revs, cubeColor ) ;
    }
    else // ia.size() == 0
    {
      cornerTri( pts, a, revs, cubeColor ) ;
      cornerTri( pts, b, revs, cubeColor ) ;
      cornerTri( pts, c, revs, cubeColor ) ;
    }

    return ia.size() ; // NO ADJACENCY.
  }

  int adjacencyOf4( Vector3i* pts, int& a, int& b, int& c, int& d,
    vector<int> &ia, vector<int> &ib, vector<int> &ic, vector<int> &id, 
    vector<int> &nia, vector<int> &nib, vector<int> &nic, vector<int> &nid )
  {
    for( int i = 0 ; i < 3 ; i++ )
    {
      if( adj[a][i] == b || adj[a][i] == c || adj[a][i] == d )
        ia.push_back( i ) ;
      else  nia.push_back( i ) ;

      if( adj[b][i] == a || adj[b][i] == c || adj[b][i] == d )
        ib.push_back( i ) ;
      else  nib.push_back( i ) ;

      if( adj[c][i] == a || adj[c][i] == b || adj[c][i] == d )
        ic.push_back( i ) ;
      else  nic.push_back( i ) ;

      if( adj[d][i] == a || adj[d][i] == b || adj[d][i] == c )
        id.push_back( i ) ;
      else  nid.push_back( i ) ;
    }
  
    if( ia.size() == 2 && ib.size() == 2 && ic.size() == 2 && id.size() == 2 )
    {
      // then CASE 5:  all are on same FACE, each with 2 adjacent neighbours.
    
      // Force b,d to be neighbours of a
      if( adj[a][ia[0]]==c || adj[a][ia[1]]==c )
      {
        SWAP(d,c) ; // swapping b & c didn't work (resulted in all criss cross)
        // ia now out of date, but we don't use it below.
      }

      // Make B ccw neighbour 
      Vector3i AB = pts[b] - pts[a] ;
      Vector3i AD = pts[d] - pts[a] ;
      Vector3i n = AB.cross( AD ) ;
      int dPlane = -n.dot( pts[a] ) ;

      if( n.dot( pts[ adj[a][ nia[0] ] ] ) + dPlane > 0 )
      {
        SWAP(b,d);
      }

      Vector3f cutA = voxelGrid->getCutPoint( isosurface, pts[ a ], pts[ adj[a][ nia[0] ] ] ) ;
      Vector3f cutB = voxelGrid->getCutPoint( isosurface, pts[ b ], pts[ adj[b][ nib[0] ] ] ) ;
      Vector3f cutC = voxelGrid->getCutPoint( isosurface, pts[ c ], pts[ adj[c][ nic[0] ] ] ) ;
      Vector3f cutD = voxelGrid->getCutPoint( isosurface, pts[ d ], pts[ adj[d][ nid[0] ] ] ) ;
      
      Geometry::addQuadWithNormal( *verts, cutA,cutB,cutC,cutD, cubeColor ) ;
      return 5 ; // DONE
    }

    // Rearrangement:
    // `a` must be the one with 3.
    else if( ib.size() == 3 )  SWAP( a,b );
    else if( ic.size() == 3 )  SWAP( a,c );
    else if( id.size() == 3 )  SWAP( a,d ) ;

    // 
    if( ia.size() == 3 )
    {
      // CASE 4:
      // one has 3 neighbours.  that means you get a diagonal cut across the cube
      // in the shape of a hexagon.
    
      // put b on edge 0, c on edge 1, d on edge 2, then the winding order will always be correct

      // You don't have to swap.  Just use the INDEX AT EDGE 0,
      //Vector3f cutA = getCutPoint( pts[ adj[a][ ia[0] ] ], pts[ adj[ adj[a][ ia[0] ] ][ nia[0]X ] ] ) ; // not going to work
      // so well b/c I need to know whether to use NIA or NIB.
      //Vector4f color=Blue ;

      //if( adj[a][ia[0]]==b && adj[a][ia[1]]==c && adj[a][ia[2]]==d )
      //  color = Blue ;

      //ia[1] is b but b must be at ia[0].
      if( adj[a][ia[1]] == b )
      {
        // b will switch with somebody (whoever's at 0).
        // For my named symbols to work here, I have to check each by name.
        // LEFT=adj[a][ ia[0] ] gets me the pt index for WHOEVER is adj[a] on left of a,
        // BUT I'D ALSO NEED the ni* for LEFT.
        // if nia,nib,nic are abstracted into arrays of arrays, then numeric indexing is possible.

        // who dat at 0 then
        if( adj[a][ia[0]] == c )  SWAP(b,c) ;
        else if( adj[a][ia[0]] == d )  SWAP(b,d) ;
      
        //swap(ia[0],ia[1]) ; // WRONG! XXdo not let ia fall out of date, I will be using it again later.
      }
      else if( adj[a][ia[2]] == b )
      {
        // who dat at 0 then
        if( adj[a][ia[0]] == c )
        {
          SWAP(b,c) ;
          //SWAP(c,d) ;
          //color = Blue ;
        }
        else if( adj[a][ia[0]] == d )
        {
          SWAP(b,d) ;
        }
      }

      // 2 possibilities now:
      // 012
      // bcd
      // bdc
      // force c at 1.  c cannot be at 0 b/c b is there already
      if( adj[a][ia[2]] == c )
      {
        // d must be in c's spot @1.
        SWAP(c,d) ;
        //color = Purple ;
      }

      // c's shared vertex with b should be index 0 for both,
      forceShare( b,c, nib,nic, 1,0 ) ; // BC, CB both use 0

      // then let c's shared index with d be index 1 for both.
      forceShare( c,d, nic,nid, 0,1 ) ; // CD, DC both use 1

      // Now they're ordered in the correct order.  BCD is CCW triangle
      // around A, so wind accordingly
      Vector3f cutBC = voxelGrid->getCutPoint( isosurface, pts[b], pts[ adj[b][ nib[0] ] ] ) ;
      Vector3f cutBD = voxelGrid->getCutPoint( isosurface, pts[b], pts[ adj[b][ nib[1] ] ] ) ; // 1 by default (the "other" one)
      Vector3f cutCB = voxelGrid->getCutPoint( isosurface, pts[c], pts[ adj[c][ nic[0] ] ] ) ;
      Vector3f cutCD = voxelGrid->getCutPoint( isosurface, pts[c], pts[ adj[c][ nic[1] ] ] ) ; 
      Vector3f cutDC = voxelGrid->getCutPoint( isosurface, pts[d], pts[ adj[d][ nid[1] ] ] ) ;
      Vector3f cutDB = voxelGrid->getCutPoint( isosurface, pts[d], pts[ adj[d][ nid[0] ] ] ) ; // 0 by default
    
      Geometry::addHexagonWithNormal( *verts, cutBC,cutBD,cutDB,cutDC,cutCD,cutCB, cubeColor ) ;

      return 4 ;
    }

    // Now check for max size of 2.
    if( ib.size() == 2 )  SWAP( a,b ) ;
    else if( ic.size() == 2 )  SWAP( a,c ) ;
    else if( id.size() == 2 )  SWAP( a,d ) ;
  
    if( ia.size() == 2 )
    {
      // 2 situation.
      // case 3: TWISTY FORK:  2 vertices have __2__ neighbours, but there is no loner.

      // get the 2nd one with 2 neighbours to be b if there is one
      if( ic.size() == 2 )
        SWAP( b,c ) ;
      else if( id.size() == 2 )
        SWAP( b,d ) ;

      if( ib.size() == 2 )
      {
        // this is the most "twisted" case.
        // these are surprisingly common.

        //a,b already have 2 neighbours in ia,ib.
        // let a and b see each other on index 0.
        if( adj[a][ia[0]] != b ) swap(ia[0],ia[1]) ;
        if( adj[b][ib[0]] != a ) swap(ib[0],ib[1]) ;

        // let a see c on its edge 1.
        if( adj[a][ia[1]] != c ) SWAP(c,d);

        // a,d share on 0
        //forceShare( a,d, nia,nid, 1,0 ) ; // can't use this, nia[0] is only size 1.
        // make sure nid[0] is a's free vertex
        if( adj[d][nid[0]] != adj[a][nia[0]] )
          swap( nid[0],nid[1] ) ;

        // make sure c's nic[0] is b's free vertex
        if( adj[c][nic[0]] != adj[b][nib[0]] )
          swap( nic[0],nic[1] ) ;

        // check winding
        bool revs = planeSide( pts, b,d,a, pts[c] )>0 ;

        Vector3f cutAD = voxelGrid->getCutPoint( isosurface, pts[ a ], pts[ adj[a][ nia[0] ] ] ) ;
        Vector3f cutC0 = voxelGrid->getCutPoint( isosurface, pts[ c ], pts[ adj[c][ nic[1] ] ] ) ;
        Vector3f cutCB = voxelGrid->getCutPoint( isosurface, pts[ c ], pts[ adj[c][ nic[0] ] ] ) ;
        Vector3f cutBA = voxelGrid->getCutPoint( isosurface, pts[ b ], pts[ adj[b][ nib[0] ] ] ) ;
        Vector3f cutD0 = voxelGrid->getCutPoint( isosurface, pts[ d ], pts[ adj[d][ nid[1] ] ] ) ;
        Vector3f cutDA = voxelGrid->getCutPoint( isosurface, pts[ d ], pts[ adj[d][ nid[0] ] ] ) ; // could also use adj[a][ nia[0] ]

        if( !revs )
          Geometry::addHexagonWithNormal( *verts, cutAD, cutC0, cutCB, cutBA, cutD0, cutDA, cubeColor ) ;
        else
          Geometry::addHexagonWithNormal( *verts, cutAD, cutDA, cutD0, cutBA, cutCB, cutC0, cubeColor ) ;
        return 3 ;
      }

      // case 2: 1 vertex has 2 neighbours (`a`).  ONE LONER (will be `d`).
      else
      {
        if( ib.size() == 0 ) SWAP( b,d ) ;
        else if( ic.size() == 0 ) SWAP( c,d ) ;

        // render the loner
        cornerTri( pts, d, 0, cubeColor ) ;

        // 
        forceShare( b,c, nib, nic, 0, 1 ) ;
      
        // pts[adj[a][nia[0]] must be on the - side of the plane.
        if( planeSide( pts, a,b,c, pts[ adj[a][nia[0]] ] ) > 0 )
          SWAP( b,c ) ;

        Vector3f cutA  = voxelGrid->getCutPoint( isosurface, pts[ a ], pts[ adj[a][ nia[0] ] ] ) ;
        Vector3f cutB1 = voxelGrid->getCutPoint( isosurface, pts[ b ], pts[ adj[b][ nib[0] ] ] ) ;
        Vector3f cutB2 = voxelGrid->getCutPoint( isosurface, pts[ b ], pts[ adj[b][ nib[1] ] ] ) ;
        Vector3f cutC1 = voxelGrid->getCutPoint( isosurface, pts[ c ], pts[ adj[c][ nic[0] ] ] ) ;
        Vector3f cutC2 = voxelGrid->getCutPoint( isosurface, pts[ c ], pts[ adj[c][ nic[1] ] ] ) ;
      
        Geometry::addPentagonWithNormal( *verts, cutA, cutB1, cutB2, cutC2, cutC1, cubeColor ) ;
        return 2 ;
      }
    }

    if( ia.size() == 1 && ib.size() == 1 && ic.size() == 1 && id.size() == 1 )
    {
      // 2 benches.
      // RARE.
      // identify which share an edge.
      //Vector4f color=Blue ;
      if( adj[a][ia[0]] == b )
      {
        // a--b
        //
        // c--d
        benchTris( pts, a,b, ia[0],ib[0], nia,nib, 0, cubeColor ) ;
        benchTris( pts, c,d, ic[0],id[0], nic,nid, 0, cubeColor ) ;
      }
      else if( adj[a][ia[0]] == c )
      {
        // a--c
        //
        // b--d
        benchTris( pts, a,c, ia[0],ic[0], nia,nic, 0, cubeColor ) ;
        benchTris( pts, b,d, ib[0],id[0], nib,nid, 0, cubeColor ) ;
      }
      else if( adj[a][ia[0]] == d )
      {
        // a--d
        //
        // b--c
        benchTris( pts, a,d, ia[0],id[0], nia,nid, 0, cubeColor ) ;
        benchTris( pts, b,c, ib[0],ic[0], nib,nic, 0, cubeColor ) ;
      }

      return 1 ;
    }

    else
    {
      // RARE.
      // 4 LONERS.  Never revs b/c we used the 4 IN pieces.
      cornerTri( pts, a, 0, cubeColor ) ;
      cornerTri( pts, b, 0, cubeColor ) ;
      cornerTri( pts, c, 0, cubeColor ) ;
      cornerTri( pts, d, 0, cubeColor ) ;

      return 0 ;
    }
  }

  void cube( const Vector3i& dex )
  {
    //    C----G
    //   /|   /|
    //  D-A--H E
    //  |/   |/
    //  B----F
    Vector3i A=dex+Vector3i(0,0,0), B=dex+Vector3i(0,0,1), C=dex+Vector3i(0,1,0), D=dex+Vector3i(0,1,1),
             E=dex+Vector3i(1,0,0), F=dex+Vector3i(1,0,1), G=dex+Vector3i(1,1,0), H=dex+Vector3i(1,1,1);
    // index: z + 2*y + 4*x (because of binary counting)
    //                  0  1  2  3  4  5  6  7
    Vector3i pts[8] = { A, B, C, D, E, F, G, H } ;
  
    // In the code below, `a`, `b`, `c`, `d` are INDICES of pts in the pts array.
    // `ia` are INDICES into adj[a][ ia[0] ] of the adjacent pts of a
    // THAT ARE ALSO (in or out of the isosurface) along with a.
    // `nia` are INDICES into adj[a][ nia[0] ] of adjacent pts
    // NOT the same isosurface status as `a`.
  
    vector<int> in, out ;
    for( int i = 0 ; i < 8 ; i++ )
      if( inSurface( (*voxelGrid)( pts[i] ).v ) )
        in.push_back( i ) ; 
      else
        out.push_back( i ) ;

    // now we enumerate those 15 cases.
    // the cases with 2,3,4,5 or 6 in, 
    // have some additional resolution

    // case 0: NOTHING IN SURFACE
    if( !in.size() ) 
      return ;

    // 0 are out
    // case 1: FULLY in surface 
    else if( in.size() == 8 )
    {
      // FILLs
      //Geometry::addCubeFacingOut( *verts, getP(A),getP(B),getP(C),getP(D),getP(E),getP(F),getP(G),getP(H), Blue ) ;
    }

    // 1 is out, or 1 is in.
    else if( in.size() == 1 || out.size() == 1 )
    {
      // You get a tri on "halfway" between excluded vert and the excluded verts neighbours.
      // Lookup neighbours.
      // cut from OUTPOINT, to each ADJACENCY OF( outpoint ) (0,1 and 2).
      int a;

      // the isosurface faces OUT.
      // cornerTri winds 0,1,2 (left, UP, right) which means
      // it defaults a CW triangle for adj visitation ordering (0,1,2).
      // If `a` is INSIDE the surface, the surfaces faces AWAY from `a`,
      // so there we reverse the orientation.
      bool revs=0 ;
      if( in.size() == 1 )  a=in[0],revs=1; // you have 1 pt in the isosurface.
      // so revs is 0 because you want the face to face INTO the cube.
      else  a=out[0]; // only 1 vertex OUT.  the cut face
      // faces OUT of rest of the cube.

      cornerTri( pts, a, revs, cubeColor ) ;
    }

    // If SAME_FACE is false, they don't even share a face at all.
    //#define SAME_FACE(i1,i2) (pts[i1].atLeastOneEqual(pts[i2]))
    //#define SAME_EDGE(i1,i2) (pts[i1].twoEqual(pts[i2]))
    else if( in.size() == 2 || out.size() == 2 )
    {
      // these 2 pts could be adjacent, same face..
      int a,b ;
      bool revs=0;
      if( in.size() == 2 )
        a=in[0],b=in[1],revs=1;
      else // out.size()==2
        a=out[0],b=out[1];
      adjacencyOf2( pts, a, b, revs ) ;
    }

    else if( in.size() == 3 || out.size() == 3 )
    {
      int a,b,c;
      bool revs=0;
      if( in.size()==3 )
      {
        a=in[0],b=in[1],c=in[2],revs=1;
      }
      else //out.size()==3
      {
        a=out[0],b=out[1],c=out[2]; //def render tris face pts. so face out don't reverse.
      }

      adjacencyOf3( pts, a,b,c, revs ) ;
    }
  
    else if( in.size()==4 || out.size()==4 ) // same test really
    {
      // 4 are in and 4 are out.
      int a=out[0],b=out[1],c=out[2],d=out[3]; // render with surfacese
      // facing towards the out points
      vector<int> ia,ib,ic,id, nia,nib,nic,nid ;
      adjacencyOf4( pts, a,b,c,d, ia,ib,ic,id, nia,nib,nic,nid ) ;
    }

    else
    {
      error( "UNHANDLED CASE" ) ;
    }
  }

  void genVizMarchingCubes()
  {
    for( int k = 0 ; k < voxelGrid->dims.z ; k++ )
    {
      for( int j = 0 ; j < voxelGrid->dims.y ; j++ )
      {
        for( int i = 0 ; i < voxelGrid->dims.x ; i++ )
        {
          Vector3i dex( i,j,k ) ;
          cube(dex);
        }
      }
    }
  }
} ;

#endif
