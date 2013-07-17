#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "GLUtil.h"
#include "Vectorf.h"

struct Geometry
{
  // makes a spherical mesh from an icosahedron.
  static void makeSphere( vector<Vector3f>& verts, float r )
  {
    //http://en.wikipedia.org/wiki/Icosahedron
    //(0, ±1, ±φ)
    //(±1, ±φ, 0)
    //(±φ, 0, ±1)
    //where φ = (1 + √5) / 2 

    const float t = ( 1 + sqrt( 5.0 ) ) / 2.0 ;
    Vector3f v[12]; // 12 base verts
    
    for( int i = 0 ; i < 4; i++ )
      //v[ i ] = Vector( 0, -(i&2), -(i&1)*t ) ; 
      v[ i ] = Vector3f( 0, i&2?-1:1, i&1?-t:t ) * r ;

    for( int i = 4 ; i < 8; i++ )
      //v[ i ] = Vector( -(i&2), -(i&1)*t, 0 ) ; 
      v[ i ] = Vector3f( i&2?-1:1, i&1?-t:t, 0 ) * r ;

    for( int i = 8 ; i < 12; i++ )
      //v[ i ] = Vector( -(i&1)*t, 0, -(i&2) ) ; 
      v[ i ] = Vector3f( i&1?-t:t, 0, i&2?-1:1 ) * r ;
      
    // these are the faces.
    addTri( verts, v[0], v[2], v[8] ) ;
    addTri( verts, v[0], v[8], v[4] ) ;
    addTri( verts, v[0], v[4], v[6] ) ;
    addTri( verts, v[0], v[6], v[9] ) ;
    addTri( verts, v[0], v[9], v[2] ) ;

    addTri( verts, v[2], v[7], v[5] ) ;
    addTri( verts, v[2], v[5], v[8] ) ;
    addTri( verts, v[2], v[9], v[7] ) ;
      
    addTri( verts, v[8], v[5], v[10] ) ;
    addTri( verts, v[8], v[10], v[4] ) ;
    
    addTri( verts, v[10], v[5], v[3] ) ;
    addTri( verts, v[10], v[3], v[1] ) ;
    addTri( verts, v[10], v[1], v[4] ) ;
    
    addTri( verts, v[1], v[6], v[4] ) ;
    addTri( verts, v[1], v[3], v[11] ) ;
    addTri( verts, v[1], v[11], v[6] ) ;

    addTri( verts, v[6], v[11], v[9] ) ;

    addTri( verts, v[11], v[3], v[7] ) ;
    addTri( verts, v[11], v[7], v[9] ) ;

    addTri( verts, v[3], v[5], v[7] ) ;
    
    
  }
  
  template <typename T> static void makeSphere( vector<T>& verts, const Vector3f& center, float r, const Vector4f& color )
  {
    const float t = ( 1 + sqrt( 5.0 ) ) / 2.0 ;
    T v[12]; // 12 base verts
    
    for( int i = 0 ; i < 4; i++ )
      v[ i ].normal = v[ i ].pos = Vector3f( 0, i&2?-1:1, i&1?-t:t ) ;

    for( int i = 4 ; i < 8; i++ )
      v[ i ].normal = v[ i ].pos = Vector3f( i&2?-1:1, i&1?-t:t, 0 ) ;

    for( int i = 8 ; i < 12; i++ )
      v[ i ].normal = v[ i ].pos = Vector3f( i&1?-t:t, 0, i&2?-1:1 ) ;
      
    for( int i = 0 ; i < 12 ; i++ )
    {
      v[ i ].color = color ;
      v[ i ].pos *= r ;
      v[ i ].pos += center ; // offset after scaling
    }
    
    // these are the faces.
    addTri( verts, v[0], v[2], v[8] ) ;
    addTri( verts, v[0], v[8], v[4] ) ;
    addTri( verts, v[0], v[4], v[6] ) ;
    addTri( verts, v[0], v[6], v[9] ) ;
    addTri( verts, v[0], v[9], v[2] ) ;

    addTri( verts, v[2], v[7], v[5] ) ;
    addTri( verts, v[2], v[5], v[8] ) ;
    addTri( verts, v[2], v[9], v[7] ) ;
      
    addTri( verts, v[8], v[5], v[10] ) ;
    addTri( verts, v[8], v[10], v[4] ) ;
    
    addTri( verts, v[10], v[5], v[3] ) ;
    addTri( verts, v[10], v[3], v[1] ) ;
    addTri( verts, v[10], v[1], v[4] ) ;
    
    addTri( verts, v[1], v[6], v[4] ) ;
    addTri( verts, v[1], v[3], v[11] ) ;
    addTri( verts, v[1], v[11], v[6] ) ;

    addTri( verts, v[6], v[11], v[9] ) ;

    addTri( verts, v[11], v[3], v[7] ) ;
    addTri( verts, v[11], v[7], v[9] ) ;

    addTri( verts, v[3], v[5], v[7] ) ;
  }

  template <typename T>
  static void makeIcosahedronVerts( T* v, float r )
  {
    //http://en.wikipedia.org/wiki/Icosahedron
    //(0, ±1, ±φ)
    //(±1, ±φ, 0)
    //(±φ, 0, ±1)
    //where φ = (1 + √5) / 2
    const static float t = ( 1 + sqrtf( 5.0f ) ) / 2.0 ;
    const static float L = sqrtf( 2.f / (5.f+sqrtf(5.f)) ) ;
    const static float S = t*L;
    
    for( int i = 0 ; i < 4; i++ )
      v[ i ].pos = Vector3f( 0, i&2?-L:L, i&1?-S:S ) * r ;

    for( int i = 4 ; i < 8; i++ )
      v[ i ].pos = Vector3f( i&2?-L:L, i&1?-S:S, 0 ) * r ;

    for( int i = 8 ; i < 12; i++ )
      v[ i ].pos = Vector3f( i&1?-S:S, 0, i&2?-L:L ) * r ;
  }
  
  static void makeIcosahedronVerts( Vector3f* v, float r )
  {
    //http://en.wikipedia.org/wiki/Icosahedron
    //(0, ±1, ±φ)
    //(±1, ±φ, 0)
    //(±φ, 0, ±1)
    //where φ = (1 + √5) / 2
    const static float t = ( 1 + sqrtf( 5.0f ) ) / 2.0 ;
    const static float L = sqrtf( 2.f / (5.f+sqrtf(5.f)) ) ;
    const static float S = t*L;
    
    for( int i = 0 ; i < 4; i++ )
      v[ i ] = Vector3f( 0, i&2?-L:L, i&1?-S:S ) * r ;

    for( int i = 4 ; i < 8; i++ )
      v[ i ] = Vector3f( i&2?-L:L, i&1?-S:S, 0 ) * r ;

    for( int i = 8 ; i < 12; i++ )
      v[ i ] = Vector3f( i&1?-S:S, 0, i&2?-L:L ) * r ;
  }

  static void makeWireframeSphere( vector<Vector3f>& verts, const Vector3f& center, float r )
  {
    Vector3f v[12]; // 12 base verts
    
    makeIcosahedronVerts( v, r ) ;
    
    for( int i = 0 ; i < 12 ; i++ )
      v[ i ] += center ; // offset after scaling
      
    // draw 30 lines!
    addEdge( verts, v[0], v[2] ) ;  addEdge( verts, v[2], v[8] ) ;
    addEdge( verts, v[0], v[8] ) ;  addEdge( verts, v[4], v[8] ) ;
    addEdge( verts, v[0], v[4] ) ;  addEdge( verts, v[4], v[6] ) ;
    addEdge( verts, v[0], v[6] ) ;  addEdge( verts, v[6], v[9] ) ;
    addEdge( verts, v[0], v[9] ) ;  addEdge( verts, v[2], v[9] ) ;
    
    addEdge( verts, v[2], v[7] ) ;  addEdge( verts, v[7], v[9] ) ;
    addEdge( verts, v[2], v[5] ) ;  addEdge( verts, v[5], v[7] ) ;
    addEdge( verts, v[5], v[8] ) ;  addEdge( verts, v[3], v[7] ) ;
    addEdge( verts, v[3], v[5] ) ;  addEdge( verts, v[3], v[10] ) ;
    addEdge( verts, v[5], v[10] ) ; addEdge( verts, v[8], v[10] ) ;
    
    addEdge( verts, v[4], v[10] ) ; addEdge( verts, v[1], v[10] ) ;
    addEdge( verts, v[1], v[3] ) ;  addEdge( verts, v[1], v[11] ) ;
    addEdge( verts, v[3], v[11] ) ; addEdge( verts, v[7], v[11] ) ;
    addEdge( verts, v[1], v[4] ) ;  addEdge( verts, v[1], v[6] ) ;
    addEdge( verts, v[6], v[11] ) ; addEdge( verts, v[9], v[11] ) ;
  }
  
  template <typename T>
  static void makeWireframeSphere( vector<T>& verts, const Vector3f& center, float r, const Vector4f& color )
  {
    T v[12]; // 12 base verts
    makeIcosahedronVerts( &v[0], r ) ;
    for( int i = 0 ; i < 12 ; i++ )
    {
      v[ i ].color = color ;
      v[ i ].pos += center ;
    }
    
    // draw 30 lines!
    addEdge( verts, v[0], v[2] ) ;  addEdge( verts, v[2], v[8] ) ;
    addEdge( verts, v[0], v[8] ) ;  addEdge( verts, v[4], v[8] ) ;
    addEdge( verts, v[0], v[4] ) ;  addEdge( verts, v[4], v[6] ) ;
    addEdge( verts, v[0], v[6] ) ;  addEdge( verts, v[6], v[9] ) ;
    addEdge( verts, v[0], v[9] ) ;  addEdge( verts, v[2], v[9] ) ;
    
    addEdge( verts, v[2], v[7] ) ;  addEdge( verts, v[7], v[9] ) ;
    addEdge( verts, v[2], v[5] ) ;  addEdge( verts, v[5], v[7] ) ;
    addEdge( verts, v[5], v[8] ) ;  addEdge( verts, v[3], v[7] ) ;
    addEdge( verts, v[3], v[5] ) ;  addEdge( verts, v[3], v[10] ) ;
    addEdge( verts, v[5], v[10] ) ;  addEdge( verts, v[8], v[10] ) ;
    
    addEdge( verts, v[4], v[10] ) ;  addEdge( verts, v[1], v[10] ) ;
    addEdge( verts, v[1], v[3] ) ;  addEdge( verts, v[1], v[11] ) ;
    addEdge( verts, v[3], v[11] ) ;  addEdge( verts, v[7], v[11] ) ;
    addEdge( verts, v[1], v[4] ) ;  addEdge( verts, v[1], v[6] ) ;
    addEdge( verts, v[6], v[11] ) ;  addEdge( verts, v[9], v[11] ) ;
    
    // Cost is 60 VERTS = 60*(3 float pos + 4 float color) = 420 floats
    
    // If you use index buffers, then cost is only
    // 12 verts = 84 floats, + 60 indices (shorts)= eq 84 + 30 float size = 114.
    

  }


  // For the vertex types that do not support normals
  template <typename T>
  static void makeSphereNoNormal( vector<T>& verts, float r, const Vector4f& color )
  {
    T v[12];
    makeIcosahedronVerts( v, r ) ;
      
    for( int i = 0 ; i < 12 ; i++ )
    {
      v[ i ].color = color ;
      v[ i ].pos *= r ;
    }
      
    // these are the faces.
    addTri( verts, v[0], v[2], v[8] ) ;
    addTri( verts, v[0], v[8], v[4] ) ;
    addTri( verts, v[0], v[4], v[6] ) ;
    addTri( verts, v[0], v[6], v[9] ) ;
    addTri( verts, v[0], v[9], v[2] ) ;

    addTri( verts, v[2], v[7], v[5] ) ;
    addTri( verts, v[2], v[5], v[8] ) ;
    addTri( verts, v[2], v[9], v[7] ) ;
      
    addTri( verts, v[8], v[5], v[10] ) ;
    addTri( verts, v[8], v[10], v[4] ) ;
    
    addTri( verts, v[10], v[5], v[3] ) ;
    addTri( verts, v[10], v[3], v[1] ) ;
    addTri( verts, v[10], v[1], v[4] ) ;
    
    addTri( verts, v[1], v[6], v[4] ) ;
    addTri( verts, v[1], v[3], v[11] ) ;
    addTri( verts, v[1], v[11], v[6] ) ;

    addTri( verts, v[6], v[11], v[9] ) ;

    addTri( verts, v[11], v[3], v[7] ) ;
    addTri( verts, v[11], v[7], v[9] ) ;

    addTri( verts, v[3], v[5], v[7] ) ;
    
  }

  template <typename T>
  static void makeOctahedron( vector<T>& verts, const T& baseVertex, const Matrix4f& mat, const Vector3f& offset )
  {

    float bwx=0.055, bwy=0.055, heado=0, tailo=8;
    float midp = 2.0*tailo/3.0 ;
    T PX,NX,PY,NY,HEAD,TAIL;
    //set up the base properties
    PX=NX=PY=NY=HEAD=TAIL=baseVertex ;
    
    PX.pos = Vector3f( bwx, 0, midp ), NX.pos = Vector3f( -bwx, 0, midp ), 
    PY.pos = Vector3f( 0, bwy, midp ), NY.pos = Vector3f( 0, -bwy, midp ),
    HEAD.pos = Vector3f(0,0,-heado), TAIL.pos = Vector3f( 0,0,tailo ) ;
    
    // The offset from base center, like "shoot from right cannon"
    // means you must offset the octahedron +x a bit, and -y a bit first.
    PX.pos += offset, PY.pos += offset, NX.pos += offset, NY.pos += offset,
    HEAD.pos += offset, TAIL.pos += offset ;
    
    // Orienting the octahedron in space
    PX.pos = mat*PX.pos ;
    PY.pos = mat*PY.pos ;
    NX.pos = mat*NX.pos ;
    NY.pos = mat*NY.pos ;
    HEAD.pos = mat*HEAD.pos ;
    TAIL.pos = mat*TAIL.pos ;

    addTri( verts, TAIL, PX, PY ) ;
    addTri( verts, TAIL, PY, NX ) ;
    addTri( verts, TAIL, NX, NY ) ;
    addTri( verts, TAIL, NY, PX ) ;

    addTri( verts, HEAD, NX, PY ) ;
    addTri( verts, HEAD, PY, PX ) ;
    addTri( verts, HEAD, PX, NY ) ;
    addTri( verts, HEAD, NY, NX ) ;

  }
  
  template <typename T> static void addEdge( vector<T>& verts, const T& a, const T& b ) {
    verts.push_back( a ) ;  verts.push_back( b ) ;
  }
  
  template <typename T> static void addTri( vector<T>& verts, const T& A, const T& B, const T& C ) {
    verts.push_back( A ) ;  verts.push_back( B ) ;  verts.push_back( C ) ;
  }
  
  static void addTri( vector<VertexPC>& verts, const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector4f& color ) {
    verts.push_back( VertexPC(A,color) ) ;
    verts.push_back( VertexPC(B,color) ) ;
    verts.push_back( VertexPC(C,color) ) ;
  }
  
  static void addTriWithNormal( vector<VertexPNC>& verts, const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector4f& color ) {
    Vector3f nABC = Triangle::triNormal( A, B, C ) ;
    addTri( verts, VertexPNC( A, nABC, color ), VertexPNC( B, nABC, color ), VertexPNC( C, nABC, color ) ) ;
  }

  static void addQuadWithNormal( vector<VertexPNC>& verts, const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector3f& D, const Vector4f& color ) {
    addTriWithNormal( verts, A, B, C, color ) ;
    addTriWithNormal( verts, A, C, D, color ) ;
  }

  static void addPentagonWithNormal( vector<VertexPNC>& verts, 
    const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector3f& D, const Vector3f& E, const Vector4f& color ) {
    addTriWithNormal( verts, A, B, C, color ) ;
    addTriWithNormal( verts, A, C, D, color ) ;
    addTriWithNormal( verts, A, D, E, color ) ;
  }
  
  static void addHexagonWithNormal( vector<VertexPNC>& verts, 
    const Vector3f& A, const Vector3f& B, const Vector3f& C,
    const Vector3f& D, const Vector3f& E, const Vector3f& F, const Vector4f& color ) {
    addTriWithNormal( verts, A, B, C, color ) ;
    addTriWithNormal( verts, C, D, A, color ) ;
    addTriWithNormal( verts, D, F, A, color ) ;
    addTriWithNormal( verts, E, F, D, color ) ;
  }

  // (0,1)
  // D----C (1,1)
  // | __/|
  // |/   |
  // A----B (1,0)
  // (0,0)
  template <typename T> static void addQuad( vector<T>& verts, const T& A, const T& B, const T& C, const T& D )
  {
    addTri( verts, A, B, C ) ;
    addTri( verts, A, C, D ) ;
  }
  
  static void addQuad( vector<VertexPC>& verts, const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector3f& D, const Vector4f& color )
  {
    addTri( verts, A, B, C, color ) ;
    addTri( verts, A, C, D, color ) ;
  }
  
  // wind the 2 faces FACING OUT ok?
  static void triPrism( vector<VertexPNC>& verts,
    const Vector3f& A, const Vector3f& B, const Vector3f& C,
    const Vector3f& D, const Vector3f& E, const Vector3f& F,
    const Vector4f& color )
  {
    addTriWithNormal( verts, A, B, C, color ) ;
    addTriWithNormal( verts, D, E, F, color ) ;
    
    addTriWithNormal( verts, A, D, F, color ) ;
    addTriWithNormal( verts, A, F, B, color ) ;
    
    addTriWithNormal( verts, B, F, E, color ) ;
    addTriWithNormal( verts, B, E, C, color ) ;
    
    addTriWithNormal( verts, C, E, D, color ) ;
    addTriWithNormal( verts, C, D, A, color ) ;
  }
  
  static void addTet( vector<VertexPNC>& verts,
    const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector3f& D,
    const Vector4f& color )
  {
    addTriWithNormal( verts, A, B, C, color ) ;
    addTriWithNormal( verts, A, D, B, color ) ;
    addTriWithNormal( verts, A, C, D, color ) ;
    addTriWithNormal( verts, B, D, C, color ) ;
  }

  // http://www.ics.uci.edu/~eppstein/projects/tetra/
  // just to see what those 5 tets stuck together in a cube look like
  // YOU CANNOT USE 5 TET PACKING FOR MARCHING TETS.  THE REASON IS
  // THE NEIGHBOURING TETRAHEDRA HAVE DIAGONALS GOING IN __OPPOSITE DIRECTIONS__ WHEN STACKED.
  // THIS IS __NOT OK__ for achieving a space filling packing because then the isosurface
  // punchthrus for adjacent cubes'o'tets will NOT be the same.
  template <typename T> static void gen5Tets( vector<T>& verts, float s, const Vector3f& center )
  {
    s/=2;
    /*
      C----G
     /|   /|
    D-A--H E
    |/   |/
    B----F
    */
    Vector3f A( -s, -s, -s ),  B( -s, -s,  s ),  C( -s,  s, -s ),  D( -s,  s,  s ),
             E(  s, -s, -s ),  F(  s, -s,  s ),  G(  s,  s, -s ),  H(  s,  s,  s ) ;
  
    A+=center,  B+=center,  C+=center,  D+=center,
    E+=center,  F+=center,  G+=center,  H+=center ;

    Geometry::addTet( verts, A, D, C, G, Vector4f( 0,0,1,0.5 ) ) ;
    Geometry::addTet( verts, E, G, F, A, Vector4f( 0,1,0,0.5 ) ) ;
    Geometry::addTet( verts, H, D, F, G, Vector4f( 0.76,0.05,0.18,0.5 ) ) ;
    Geometry::addTet( verts, F, D, A, G, Vector4f( 1,1,0,0.5 ) ) ; // MIDDLE TET
    Geometry::addTet( verts, B, D, A, F, Vector4f( 1,0,0,0.5 ) ) ;
  }

  // http://graphics.cs.ucdavis.edu/~joy/ecs177/other-notes/SixTetrahedra.html
  // This paper "Mysteries in Packing Regular Tetrahedra"
  // http://www.ams.org/notices/201211/rtx121101540p.pdf  (free atm)
  // has a diagram of the packing used here (figure 3)
  // s is the size.
  template <typename T> static void gen6Tets( vector<T>& verts, float s, const Vector3f& center )
  {
    // Notice how ALL the tets use vertex E.
    s/=2;
    /*
      C----G
     /|   /|
    D-A--H E
    |/   |/
    B----F
    */
    // the cube you lay the tets in start in the [-s/2,s/2] cube (s was divided by 2 above)
    // centered AT THE ORIGIN then you translate the entire cube.
    Vector3f A( -s, -s, -s ),  B( -s, -s,  s ),  C( -s,  s, -s ),  D( -s,  s,  s ),
             E(  s, -s, -s ),  F(  s, -s,  s ),  G(  s,  s, -s ),  H(  s,  s,  s ) ;
  
    A+=center,  B+=center,  C+=center,  D+=center,
    E+=center,  F+=center,  G+=center,  H+=center ;

    // LEFT /NX EDGE
    Geometry::addTet( verts, A, B, D, E, Vector4f(   0,   1,   1, 0.5 ) ) ; //Cyan
    Geometry::addTet( verts, A, D, C, E, Vector4f(   0,   0,   1, 0.5 ) ) ; //Blue

    // TOP
    Geometry::addTet( verts, D, G, C, E, Vector4f(   1,   0,   1, 0.5 ) ) ; //Magenta
    Geometry::addTet( verts, D, H, G, E, Vector4f(   0,   1,   0, 0.5 ) ) ; // Green

    // FRONT
    Geometry::addTet( verts, B, F, D, E, Vector4f(   1,   0,   0, 0.5 ) ) ; // red
    Geometry::addTet( verts, F, H, D, E, Vector4f(   1,   1,   0, 0.5 ) ) ; // yellow
  }


  
  template <typename T> static void addQuadGenUVNormal( vector<T>& verts,
    const T& A, const T& B, const T& C, const T& D,
    const Vector2f& minTex, const Vector2f& maxTex )
  {
    // (0,1)
    // D----C (1,1)
    // | __/|
    // |/   |
    // A----B (1,0)
    // (0,0)
    
    A.tex = minTex ;
    B.tex = Vector2f( maxTex.x, minTex.y );
    C.tex = maxTex ;
    D.tex = Vector2f( minTex.x, maxTex.y );
    
    // Sets the normal 
    setFaceNormalQuad( A, B, C, D ) ;
    
    addQuad( verts, A, B, C, D ) ;
  }
  
  template <typename T> static void setFaceNormalQuad( T& A, T& B, T& C, T& D )
  {
    // Find the normal
    Vector3f triNorm = Triangle::triNormal( A.pos, B.pos, C.pos ) ;
    A.normal = B.normal = C.normal = D.normal = triNorm ;
  }
  
  
  template <typename T> static void makeSquare( vector<T>& verts, const Vector3f& min, const Vector3f& max, const Vector4f& color,
    const Vector2f& minTex, const Vector2f& maxTex )
  {
    T A,B,C,D ;

    // D----C
    // | __/|
    // |/   |
    // A----B 

    A.pos = min ;
    B.pos = Vector3f( max.x, min.y, max.z ) ; // use max's z all the time
    C.pos = max ;
    D.pos = Vector3f( min.x, max.y, max.z ) ;

    A.color=B.color=C.color=D.color= color ;
    
    spinQuadGenUVNormal( verts, A, B, C, D, minTex, maxTex ) ;
  }
  
  template <typename T> static void makeCube( vector<T>& verts, const Vector3f& min, const Vector3f& max, const Vector4f& color,
    const Vector2f& minTex, const Vector2f& maxTex,
    bool facingOut )
  {
    T A,B,C,D,E,F,G,H;
    
    //Vector3f A( min ),B( min.x, min.y, max.z ),C( min.x, max.y, min.z ),D( min.x, max.y, max.z ),
    //  E( max.x, min.y, min.z ),F( max.x, min.y, max.z ),G( max.x, max.y, min.z ),H( max );
    A.pos = min ;
    B.pos = Vector3f( min.x, min.y, max.z ) ;
    C.pos = Vector3f( min.x, max.y, min.z ) ;
    D.pos = Vector3f( min.x, max.y, max.z ) ;
    
    E.pos = Vector3f( max.x, min.y, min.z ) ;
    F.pos = Vector3f( max.x, min.y, max.z ) ;
    G.pos = Vector3f( max.x, max.y, min.z ) ;
    H.pos = max ;
    
    A.color=B.color=C.color=D.color=E.color=F.color=G.color=H.color= color ;
    
    //       y
    //     ^
    //     |
    //    C----G
    //   /|   /|
    //  D-A--H E  -> x
    //  |/   |/ 
    //  B----F  
    //  /
    // z
    
    // 6 faces
    if( facingOut )
    {
      // CCW out
      spinQuadGenUVNormal( verts, F, E, G, H, minTex, maxTex ) ; //PX
      spinQuadGenUVNormal( verts, A, B, D, C, minTex, maxTex ) ; //NX
      spinQuadGenUVNormal( verts, G, C, D, H, minTex, maxTex ) ; //PY
      spinQuadGenUVNormal( verts, F, B, A, E, minTex, maxTex ) ; //NY
      spinQuadGenUVNormal( verts, B, F, H, D, minTex, maxTex ) ; //PZ
      spinQuadGenUVNormal( verts, E, A, C, G, minTex, maxTex ) ; //NZ
    }
    else
    {
      // normals face IN, CCW in.
      spinQuadGenUVNormal( verts, E, F, H, G, minTex, maxTex ) ; //PX
      spinQuadGenUVNormal( verts, B, A, C, D, minTex, maxTex ) ; //NX
      spinQuadGenUVNormal( verts, C, G, H, D, minTex, maxTex ) ; //PY
      spinQuadGenUVNormal( verts, B, F, E, A, minTex, maxTex ) ; //NY
      spinQuadGenUVNormal( verts, F, B, D, H, minTex, maxTex ) ; //PZ
      spinQuadGenUVNormal( verts, A, E, G, C, minTex, maxTex ) ; //NZ
    }
  }
  
  
  // related to makeCube, this function CHANGES the texcoords in verts
  // to being minTex/maxTex as specified.
  // This is needed because as you choose an itembox to create,
  // the skin has to be selected from the main texture
  template <typename T> static void cubeChangeTexcoords( vector<T>& verts,
    const Vector2f& minTex, const Vector2f& maxTex )
  {
    // now use the same visitation order (ABC, ACD)
    for( int i = 0 ; i < verts.size() ; i+=6 )
    {
      // (0,1)
      // D----C (1,1)
      // | __/|
      // |/   |
      // A----B (1,0)
      // (0,0)
      verts[i].tex = minTex ; //A
      verts[i+1].tex = Vector2f(maxTex.x,minTex.y) ;
      verts[i+2].tex = maxTex ;
      
      verts[i+3].tex = minTex ;
      verts[i+4].tex = maxTex ;
      verts[i+5].tex = Vector2f(minTex.x,maxTex.y) ;
    }
  }
  
  static void addCubeFacingIn( vector<VertexPC>& cmVerts, const Vector3f& center, float s, const Vector4f& color )
  {
    s /= 2.f;
    /*

      C----G
     /|   /|
    D-A--H E
    |/   |/
    B----F

       D--H
       |  |
    D--C--G--H--D
    |  |  |  |  |
    B--A--E--F--B
       |  |
       B--F

    */
    Vector3f A( -s, -s, -s ),  B( -s, -s,  s ),  C( -s,  s, -s ),  D( -s,  s,  s ),
      E(  s, -s, -s ),  F(  s, -s,  s ),  G(  s,  s, -s ),  H(  s,  s,  s ) ;
    A+=center ;  B+=center ;  C+=center ;  D+=center ;
    E+=center ;  F+=center ;  G+=center ;  H+=center ;
    // right face PX
    Geometry::addQuad( cmVerts, VertexPC( E,color ), VertexPC( F,color ), VertexPC( H,color ), VertexPC( G,color ) ) ; // IN
    
    // left NX
    Geometry::addQuad( cmVerts, VertexPC( B,color ), VertexPC( A,color ), VertexPC( C,color ), VertexPC( D,color ) ) ; // IN

    // top face PY
    Geometry::addQuad( cmVerts, VertexPC( C,color ), VertexPC( G,color ), VertexPC( H,color ), VertexPC( D,color ) ) ; // IN
    
    // bottom NY
    Geometry::addQuad( cmVerts, VertexPC( B,color ), VertexPC( F,color ), VertexPC( E,color ), VertexPC( A,color ) ) ; // IN
    
    // back face PZ
    Geometry::addQuad( cmVerts, VertexPC( A,color ), VertexPC( E,color ), VertexPC( G,color ), VertexPC( C,color ) ) ; // IN
    
    // front face NZ
    Geometry::addQuad( cmVerts, VertexPC( F,color ), VertexPC( B,color ), VertexPC( D,color ), VertexPC( H,color ) ) ;
  }
  
  static void addCubeFacingIn( vector<VertexPNC>& cmVerts, const Vector3f& center, float s, const Vector4f& color )
  {
    s /= 2.f;
    /*
      C----G
     /|   /|
    D-A--H E
    |/   |/
    B----F

       D--H
       |  |
    D--C--G--H--D
    |  |  |  |  |
    B--A--E--F--B
       |  |
       B--F
    */
    Vector3f A( -s, -s, -s ),  B( -s, -s,  s ),  C( -s,  s, -s ),  D( -s,  s,  s ),
             E(  s, -s, -s ),  F(  s, -s,  s ),  G(  s,  s, -s ),  H(  s,  s,  s ) ;
    A+=center ;  B+=center ;  C+=center ;  D+=center ;
    E+=center ;  F+=center ;  G+=center ;  H+=center ;
    // right face PX
    Vector3f norm( -1,0,0 ) ;
    Geometry::addQuad( cmVerts, VertexPNC( E,norm,color ), VertexPNC( F,norm,color ), VertexPNC( H,norm,color ), VertexPNC( G,norm,color ) ) ; // IN
    
    // left NX
    norm.x= 1;
    Geometry::addQuad( cmVerts, VertexPNC( B,norm,color ), VertexPNC( A,norm,color ), VertexPNC( C,norm,color ), VertexPNC( D,norm,color ) ) ; // IN

    // top face PY
    norm.x=0,norm.y=-1;
    Geometry::addQuad( cmVerts, VertexPNC( C,norm,color ), VertexPNC( G,norm,color ), VertexPNC( H,norm,color ), VertexPNC( D,norm,color ) ) ; // IN
    
    // bottom NY
    norm.y= 1;
    Geometry::addQuad( cmVerts, VertexPNC( B,norm,color ), VertexPNC( F,norm,color ), VertexPNC( E,norm,color ), VertexPNC( A,norm,color ) ) ; // IN
    
    // back face PZ
    norm.y=0,norm.z=-1;
    Geometry::addQuad( cmVerts, VertexPNC( A,norm,color ), VertexPNC( E,norm,color ), VertexPNC( G,norm,color ), VertexPNC( C,norm,color ) ) ; // IN
    
    // front face NZ
    norm.z= 1;
    Geometry::addQuad( cmVerts, VertexPNC( F,norm,color ), VertexPNC( B,norm,color ), VertexPNC( D,norm,color ), VertexPNC( H,norm,color ) ) ;
  }

  static void addCubeFacingOut( vector<VertexPNC>& cmVerts, const Vector3f& center, float s, const Vector4f& color )
  {
    s /= 2.f;
    /*
      C----G
     /|   /|
    D-A--H E
    |/   |/
    B----F
    */
    Vector3f A( -s, -s, -s ),  B( -s, -s,  s ),  C( -s,  s, -s ),  D( -s,  s,  s ),
             E(  s, -s, -s ),  F(  s, -s,  s ),  G(  s,  s, -s ),  H(  s,  s,  s ) ;
    A+=center ;  B+=center ;  C+=center ;  D+=center ;  E+=center ;  F+=center ;  G+=center ;  H+=center ;
    // right face PX
    Vector3f norm( 1,0,0 ) ;
    Geometry::addQuad( cmVerts, VertexPNC( E,norm,color ), VertexPNC( G,norm,color ), VertexPNC( H,norm,color ), VertexPNC( F,norm,color ) ) ; // IN
    
    // left NX
    norm.x=-1;
    Geometry::addQuad( cmVerts, VertexPNC( B,norm,color ), VertexPNC( D,norm,color ), VertexPNC( C,norm,color ), VertexPNC( A,norm,color ) ) ; // IN

    // top face PY
    norm.x=0,norm.y=1;
    Geometry::addQuad( cmVerts, VertexPNC( C,norm,color ), VertexPNC( D,norm,color ), VertexPNC( H,norm,color ), VertexPNC( G,norm,color ) ) ; // IN
    
    // bottom NY
    norm.y=-1;
    Geometry::addQuad( cmVerts, VertexPNC( B,norm,color ), VertexPNC( A,norm,color ), VertexPNC( E,norm,color ), VertexPNC( F,norm,color ) ) ; // IN
    
    // back face PZ
    norm.y=0,norm.z=1;
    Geometry::addQuad( cmVerts, VertexPNC( A,norm,color ), VertexPNC( C,norm,color ), VertexPNC( G,norm,color ), VertexPNC( E,norm,color ) ) ; // IN
    
    // front face NZ
    norm.z=-1;
    Geometry::addQuad( cmVerts, VertexPNC( F,norm,color ), VertexPNC( H,norm,color ), VertexPNC( D,norm,color ), VertexPNC( B,norm,color ) ) ;
  }

  static void addCubeFacingOut( vector<VertexPNC>& cmVerts,
    const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector3f& D,
    const Vector3f& E, const Vector3f& F, const Vector3f& G, const Vector3f& H, const Vector4f& color )
  {
    /*
      C----G
     /|   /|
    D-A--H E
    |/   |/
    B----F
    */
    // right face PX
    Vector3f norm( 1,0,0 ) ;
    Geometry::addQuad( cmVerts, VertexPNC( E,norm,color ), VertexPNC( G,norm,color ), VertexPNC( H,norm,color ), VertexPNC( F,norm,color ) ) ; // IN
    
    // left NX
    norm.x=-1;
    Geometry::addQuad( cmVerts, VertexPNC( B,norm,color ), VertexPNC( D,norm,color ), VertexPNC( C,norm,color ), VertexPNC( A,norm,color ) ) ; // IN

    // top face PY
    norm.x=0,norm.y=1;
    Geometry::addQuad( cmVerts, VertexPNC( C,norm,color ), VertexPNC( D,norm,color ), VertexPNC( H,norm,color ), VertexPNC( G,norm,color ) ) ; // IN
    
    // bottom NY
    norm.y=-1;
    Geometry::addQuad( cmVerts, VertexPNC( B,norm,color ), VertexPNC( A,norm,color ), VertexPNC( E,norm,color ), VertexPNC( F,norm,color ) ) ; // IN
    
    // back face PZ
    norm.y=0,norm.z=1;
    Geometry::addQuad( cmVerts, VertexPNC( A,norm,color ), VertexPNC( C,norm,color ), VertexPNC( G,norm,color ), VertexPNC( E,norm,color ) ) ; // IN
    
    // front face NZ
    norm.z=-1;
    Geometry::addQuad( cmVerts, VertexPNC( F,norm,color ), VertexPNC( H,norm,color ), VertexPNC( D,norm,color ), VertexPNC( B,norm,color ) ) ;
  }
  
} ;


#endif