/*

  https://github.com/superwills/Ice-OSurface
  version 1.0 July 2 2013 7:30p

  Copyright (C) 2013 William Sherif

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  William Sherif
  will.sherif@gmail.com

  This source code includes Stefan Gustavson's Perlin noise code (in Perlin.h)

*/


#ifdef _WIN32
#include <stdlib.h> // MUST BE BEFORE GLUT ON WINDOWS
#include <gl/glut.h>



#else
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <Carbon/Carbon.h>  // key input
#endif
#include "perlin.h"
#include "GLUtil.h"
#include "StdWilUtil.h"
#include "Vectorf.h"
#include "Geometry.h"
#include "MarchingCommon.h"
#include <vector>
#include <set>
#include <functional>
using namespace std;

#include "VoxelGrid.h"
#include "Mesh.h"
#include "PointCloud.h"
#include "MarchingTets.h"
#include "MarchingCubes.h"





// window width and height
float w=768.f, h=768.f ;
float mouseX, mouseY,
  wTerrain=2.59f, // the w to use for the terrain generation
  wTexture=0.1f,   // texture w
  isosurface=0.38f // the isosurface variable
;
// the repeat period for w.
static int wTerrainPeriod=8, wTexturePeriod=8;

// How many times you want the procedural texture to repeat around the world
// too few repeats (with a low res texture) will look pixellated
// too many repeats will make the periodicity really apparent
int textureRepeats = 2 ;

enum VizGenMode { VizGenCubes, VizGenTets, VizGenPts } ;
const char* VizGenModeName[] = { "VizGenCubes", "VizGenTets", "VizGenPts" } ;
int vizGenMode = VizGenCubes ;

// RENDERING OPTIONS:
float lineWidth=1.f;
bool lightingOn=1, axisLinesOn=0, displayTextOn=0;
bool showGradients=0 ; // not used since gradients not generated here

bool repeats = 0 ;  // Show the world repeated (key 'r')
Axis axis ;         // for moving around in space
float speed=0.02f ; // (key 'g'): movement speed
float minEdgeLength=0.1f ; // the minimum ALLOWED edge length before the edge gets removed.


// My global voxel grid.
VoxelGrid voxelGrid ;
Mesh mesh ;

vector<VertexPC> gradients ; // for showing isosurface gradients as given by the 
// Perlin noise class version that HAS gradients for each point (not used actually in final code)
vector<VertexPC> debugLines ;  // for showing surface normals etc.
void addDebugLine( const Vector3f& v1, const Vector4f& c1, const Vector3f& v2, const Vector4f& c2 )
{
  debugLines.push_back( VertexPC( v1, c1 ) ) ;
  debugLines.push_back( VertexPC( v2, c2 ) ) ;
}











void genVizFromVoxelData()
{
  // Generate the visualization
  mesh.verts.clear() ;
  mesh.indices.clear() ;
  gradients.clear() ;
  debugLines.clear() ;

  mesh.renderMode = GL_TRIANGLES ;
  // ISOSURFACE GENERATION!
  if( vizGenMode == VizGenPts )
  {
    // GENERATE THE VISUALIZATION AS POINTS
    PointCloud pc( &voxelGrid, &mesh.verts, isosurface, White ) ; 
    if( !pc.useCubes ) mesh.renderMode = GL_POINTS ;
    pc.genVizPunchthru() ;
    mesh.vertexTexture( wTexture, wTexturePeriod, voxelGrid.worldSize, textureRepeats ) ;
  }
  else if( vizGenMode == VizGenTets )
  {
    MarchingTets mt( &voxelGrid, &mesh.verts, isosurface, White ) ;
    mt.genVizMarchingTets() ;
    mesh.vertexTexture( wTexture, wTexturePeriod, voxelGrid.worldSize, textureRepeats ) ;
    mesh.smoothMesh( &voxelGrid, minEdgeLength ) ;
  }
  else
  {
    MarchingCubes mc( &voxelGrid, &mesh.verts, isosurface, White ) ;
    mc.genVizMarchingCubes() ;
    mesh.vertexTexture( wTexture, wTexturePeriod, voxelGrid.worldSize, textureRepeats ) ;
    mesh.smoothMesh( &voxelGrid, minEdgeLength ) ;
  }
  
  
}

void regen()
{
  voxelGrid.genData( wTerrain, wTerrainPeriod ) ;    
  genVizFromVoxelData() ;
}

// Voronoi texture
struct Site
{
  int row,col;
  Vector2f pos ;
  Site() : row(0),col(0),pos( 0, 0 ) {}
  Site( int icol, int irow ) : col(icol),row(irow),pos( icol, irow ) {}
  
  inline int getIndex( int rows, int cols ) const { return row*cols + col ; }
  
  // I center 
  Vector2f getWrappedPos( const Vector2f& worldSize, const Vector2f& v ) const
  {
    Vector2f offsetToCenter = worldSize/2.f - v ; // takes relativeToV to worldCenter, I may go OOB
    Vector2f imagePos = pos + offsetToCenter ; // I may go OOB as I am offset by the same amt that makes `this` @ worldCenter
    return imagePos.wrap( worldSize ) - offsetToCenter ; // wrap to fit in a world of worldSize (+ space world)
  }
  
  float getEuclideanDistance( const Vector2f& worldSize, const Vector2f& v ) const
  {
    return distance1( getWrappedPos( worldSize, v ), v ) ;
  }
  float getEuclideanDistance2( const Vector2f& worldSize, const Vector2f& v ) const
  {
    return distance2( getWrappedPos( worldSize, v ), v ) ;
  }
  
  float getManhattanDistance( const Vector2f& worldSize, const Vector2f& v ) const
  {
    return (getWrappedPos( worldSize, v ) - v).fabs().sum() ; //manhattan distance
  }
  
  float getChebyshevChessDistance( const Vector2f& worldSize, const Vector2f& v ) const
  {
    return (getWrappedPos( worldSize, v ) - v).max() ;
  }
} ;

/// Procedural textures
struct Texture
{
  GLuint texId ;
  int w,h ;
  vector<float> vals ; // floating point noise values.
  
  // You could work with each color channel separately.
  // You could map rgb COMPLETELY DIFFERENTLY __at each stage__,
  // which means rgb would go in totally distinct directions.
  //vector<Vector4f> colorVals ; // colorized noise values.
  
  function<Vector4f ( float val )> colorizationFunc ;
  
  Texture( int iw, int ih ) : w(iw), h(ih)
  {
    vals.resize( w*h, 0.f ) ;
    //colorVals.resize( w*h, Vector4f(0,0,0,1) ) ;
    
    // The default colorization func is basically grayscale, full alpha
    colorizationFunc = []( float val ) -> Vector4f {
      return Vector4f( val,val,val,1.f ) ;
    } ;
    
    clear() ;
  }
  
  ~Texture() {}
  
  inline void bind() const {
    glBindTexture( GL_TEXTURE_2D, texId ) ;  CHECK_GL ;
  }
  
  // clear black, full alpha
  Texture& clear() {
    clear( 0 ) ;
    return *this ;
  }
  
  Texture& clear( float toVal )
  {
    for( int i = 0 ; i < w*h ; i++ )
      vals[ i ] = toVal ;
    return *this ;
  }
  
  Texture& randomNoise()
  {
    for( int i = 0 ; i < h ; i++ ) {
      for( int j = 0 ; j < w ; j++ ) {
        int dex = i*w + j ;
        vals[ dex ] = randFloat() ;
      }
    }
    return *this ;
  }
  
  Texture& checkerboard( int dimX, int dimY, float darkVal, float lightVal )
  {
    for( int i = 0 ; i < h ; i++ ) {
      for( int j = 0 ; j < w ; j++ ) {
        int dex = i*w + j ;
        int iCell = i / dimY ;
        int jCell = j / dimX ;
        
        if( iCell % 2 == jCell % 2 )
          vals[ dex ] = darkVal ;
        else
          vals[ dex ] = lightVal ;
      }
    }
    return *this ;
  }
  
  Texture& perlin( int octaves, float octaveScaleFactor, int freqMult )
  {
    for( int i = 0 ; i < h ; i++ ) {
    for( int j = 0 ; j < w ; j++ ) {
      int dex = i*w + j ;
      
      // if the baseFreq is TOO LOW, start at a higher one
      float x = (float)i/h ;
      float y = (float)j/w ;
      
      float scale = 1.f ;
      int period = 1 ;
      
      // add a few octaves of typical fractal noise
      for( int i=0 ; i < octaves ; i++ )
      {
        vals[dex] += Perlin::pnoise( x, y, period, period ) * scale ;

        // "speed up" x and y
        x *= freqMult ;
        y *= freqMult ;
        scale *= octaveScaleFactor ;
        
        // The period of the noise has grown
        period *= freqMult ;

      }
    }}
    return *this ;
  }
  
  // worley's voronoi noise
  Texture& worley( const vector<Site> &sites )
  {
    Vector2f worldSize( h, w ) ;
    
    vector<float> distanceBuffer( w*h, 0.f ) ;
      
    float largestMinDist = 0.f ;
    for( int i = 0 ; i < h ; i++ ) {
      for( int j = 0 ; j < w ; j++ ) {
        int dex = i*w + j ;
        
        // get the min dist to all 12 sites
        float minDist = HUGE ;
        Vector2f mePos(j,i);
        for( int si = 0 ; si < sites.size() ; si++ )
        {
          float dist = sites[si].getEuclideanDistance2( worldSize, mePos ) ; //euclidean distance
          //float dist = sites[si].getManhattanDistance( worldSize, mePos ) ;
          //float dist = sites[si].getChebyshevChessDistance( worldSize, mePos ) ; //chebyshev (chessboard)
          
          if( dist < minDist )  minDist = dist ;
        }
        distanceBuffer[ dex ] = minDist ;
        
        // After deciding on the minDist for that pixel (to all sites),
        // see if that was the greatest minDist so far (normalizer)
        if( minDist > largestMinDist )  largestMinDist = minDist ;
      }
    }
    
    // COLOR & NORMALIZE
    for( int i = 0 ; i < h ; i++ ) {
      for( int j = 0 ; j < w ; j++ ) {
        int dex = i*w + j ;
        float s = distanceBuffer[dex] / largestMinDist ;
        vals[dex] = s ;
      }
    }
    
    return *this ;
  }
  
  // I DEFINE an 'octave' for worley is "farther distances" (still have yet to verify my terminology with the literature),
  // the "first octave" are the distances to the CLOSEST sites
  // "2nd octave" distances to 2nd closest. (I think they call these F1, F2 etc.)
  // initialScale is the multiplier for the 1st octave.
  // if octaveScaleFactor is > 1, then the high octaves weigh MORE AND MORE
  Texture& worley( int numSites, float initialScale, float octaveScaleFactor, int octaves )
  {
    vector<Site> sites ; // indices of sites.
    for( int i = 0 ; i < numSites ; i++ )
      sites.push_back( Site( randInt( 0, h ), randInt( 0, w ) ) ) ;
      
    Vector2f worldSize( h, w ) ;
    
    // these store distances in order.
    vector< vector<float> > distanceBuffer( w*h ) ;
    
    for( int i = 0 ; i < h ; i++ ) {
      for( int j = 0 ; j < w ; j++ ) {
        int dex = i*w + j ;
        Vector2f mePos(j,i);
        
        // Get all the distances to all the sites.
        for( int si = 0 ; si < sites.size() ; si++ )
        {
          float dist = sites[si].getEuclideanDistance( worldSize, mePos ) ; //euclidean distance
          //float dist = sites[si].getManhattanDistance( worldSize, mePos ) ;
          //float dist = sites[si].getChebyshevChessDistance( worldSize, mePos ) ; //chebyshev (chessboard)
          // chebyshev makes a thatch pattern
          
          // find the spot
          vector<float>::iterator iter = distanceBuffer[dex].begin() ;

          //                                                                                          0.6599          
          //                                                                                             ^
          // advance iter until the one it points to exceeds `dist`: they go in ascending order ( 0.4532, 1.1235, 2.23525 )
          while( iter != distanceBuffer[dex].end() && dist > *iter )  ++iter ;
          
          // goes BEFORE iter. (and when iter points to "1 past the end", it goes before that.)
          distanceBuffer[dex].insert( iter, dist ) ;
        }
      }
    }
    
    // NORMALIZE
    // find the max for each octave
    vector<float> maxes( octaves, 0.f ) ;
    for( int i = 0 ; i < w*h ; i++ )
    {
      for( int oc = 0 ; oc < octaves ; oc++ )
      {
        // "octave 0" is just the closest distance.  here
        // maxes[0] looks in ALL distanceBuffer[dex] for
        // the LARGEST "CLOSEST" distance
        if( maxes[oc] < distanceBuffer[i][oc] )
          maxes[oc] = distanceBuffer[i][oc] ;
      }
    }
    
    for( int i = 0 ; i < w*h ; i++ )
      for( int oc = 0 ; oc < octaves ; oc++ )
        distanceBuffer[i][oc] /= maxes[oc] ; // normalize EACH OCTAVE
    
    float scale = initialScale ;
    
    /*
    // Sum the octaves.
    for( int oc = 0 ; oc < octaves ; oc++ )
    {
      for( int i = 0 ; i < h ; i++ ) {
      for( int j = 0 ; j < w ; j++ ) {
        int dex = i*w + j ;
        
        // Apply the normalization to each octave here
        float s = scale * distanceBuffer[dex][oc] / maxes[oc] ;
        vals[dex] = s ;
      }}
      
      scale *= octaveScaleFactor ;
    }
    */
    
    // Sum the octaves.
    for( int i = 0 ; i < h ; i++ ) {
    for( int j = 0 ; j < w ; j++ ) {
      int dex = i*w + j ;
      
      // Apply the normalization to each octave here
      float s = distanceBuffer[dex][1] - distanceBuffer[dex][0] ;
      vals[dex] = s ;
    }}
    
    scale *= octaveScaleFactor ;
    return *this ;
  }
  
  Texture& operator*=( const Texture& o ) {
    for( int i = 0 ; i < w*h ; i++ )
      vals[i] *= o.vals[i] ;
    return *this ;
  }
  
  Texture& operator/=( const Texture& o ) {
    for( int i = 0 ; i < w*h ; i++ )
      vals[i] /= o.vals[i] ;
    return *this ;
  }
  
  Texture& operator+=( const Texture& o ) {
    for( int i = 0 ; i < w*h ; i++ )
      vals[i] += o.vals[i] ;
    return *this ;
  }
  
  // bias the whole texture by some float val.
  // useful if you want the MINIMUM value to be some range
  Texture& operator+=( float val ) {
    for( int i = 0 ; i < w*h ; i++ )
      vals[i] += val ;
    return *this ;
  }
  
  Texture& operator-=( const Texture& o ) {
    for( int i = 0 ; i < w*h ; i++ )
      vals[i] -= o.vals[i] ;
    return *this ;
  }
  
  // FORCES opaque (alpha=1)
//  Texture& opaque( const Texture& o ) {
//    for( int i = 0 ; i < w*h ; i++ )
//      vals[i].a = 1.f ;
//    return *this ;
//  }
  
  // Finds max component and makes range 0->1 all values
  Texture& renormalize()
  {
    float maxVal=0.f;
    float minVal=HUGE ;
    
    for( int i = 0 ; i < w*h ; i++ ) {
      if( vals[i] > maxVal )  maxVal = vals[i] ;
      if( vals[i] < minVal )  minVal = vals[i] ;
    }
    
    // BIAS if -ve
    // -0.2, 0.0, 0.2 => 0, 0.2, 0.4
    if( minVal < 0.f ){
      for( int i = 0 ; i < w*h ; i++ )
        vals[i] -= minVal ;
        
      maxVal -= minVal ;
    }

    if( maxVal > 0.f )
      for( int i = 0 ; i < w*h ; i++ )
        vals[i] /= maxVal ;
        
    return *this ;
  }
  
  void createGL()
  {
    renormalize() ;
    
    vector<unsigned int> texels ;
    texels.resize( w * h, 0 ) ;
    
    // The default is to get the rGBA int from each vector4f
    for( int i = 0 ; i < w*h ; i++ )
      texels[i] = colorizationFunc( vals[ i ] ).RGBAInt() ;
    
    glGenTextures( 1, &texId ) ;  CHECK_GL ;
    glBindTexture( GL_TEXTURE_2D, texId ) ;  CHECK_GL ;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);  CHECK_GL ;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);  CHECK_GL ;
    
    // we do not want to wrap, this will cause incorrect shadows to be rendered
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT ) ;  CHECK_GL ; //GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT ) ;  CHECK_GL ;
    
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, &texels[0] ) ;  CHECK_GL ;
    glActiveTexture( GL_TEXTURE0 ) ;  CHECK_GL ;
  }
} ;











// Generates the procedural `detail texture`
void genTex( int w, int h )
{
  Texture t1( w, h ) ;
  Texture t2( w, h ) ;
  
  // perlin( int octaves, float initialScale, float octaveScaleFactor, int freqMult )
  t1.worley( 25, 0.34, 1.3, 2 ) ;
  t1 += 0.5 ;
  t1.renormalize() ;
  
  t2.perlin( 8, 0.5, 2.0 ) ;
  t2.renormalize() ;
  t1 *= t2 ;
  
  t1.createGL() ;
}











void init() // Called before main loop to set up the program
{
  //initDirections() ;
  regen() ;
  genTex( 1024, 1024 ) ;
  
  axis.pos = Vector3f( 0, 0, 350 ) ;
  glClearColor( 0.1, 0.1, 0.1, 0.1 ) ;
  glEnable( GL_COLOR_MATERIAL ) ;

  //glPointSize(16.f);
  //for( int i = 0 ; i < 3*1000 ; i++ )
  //  mesh.verts.push_back( VertexPC( Vector3f::random(-1,1), Vector4f::random() ) ) ;
  
  glEnable( GL_BLEND ) ;
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA ) ;
}

void exportOBJ( const char* filename )
{
  FILE* f = fopen( filename, "w" ) ;

  fprintf( f, "# ICE-OSURFACE .obj file output\n" ) ;
  fprintf( f, "# v//n\n" ) ;

  // Usually it goes v, v, v, v, n, n, n, n
  for( int i = 0 ; i < mesh.verts.size() ; i++ )
  {
    fprintf( f, "v %f %f %f\n", mesh.verts[i].pos.x, mesh.verts[i].pos.y, mesh.verts[i].pos.z ) ;
    //fprintf( f, "vn %f %f %f\n", mesh.verts[i].normal.x, mesh.verts[i].normal.y, mesh.verts[i].normal.z ) ;

    // PER-VERTEX color.  Not recognized by any other program, I made this up.
    // obj uses MATERIALS which I avoid here.
    ///fprintf( f, "c %f %f %f %f\n", mesh.verts[i].color.x, mesh.verts[i].color.y, mesh.verts[i].color.z, mesh.verts[i].color.w ) ;
  }
  
  for( int i = 0 ; i < mesh.verts.size() ; i++ )
    fprintf( f, "vn %f %f %f\n", mesh.verts[i].normal.x, mesh.verts[i].normal.y, mesh.verts[i].normal.z ) ;

  for( int i = 0 ; i < mesh.indices.size() ; i+=3 )
  {
    // color is not specified here
    // INDEXING IS 1-BASED, NOT 0-BASED
    fprintf( f, "f %d//%d %d//%d %d//%d\n",
      mesh.indices[i  ]+1, mesh.indices[i  ]+1, 
      mesh.indices[i+1]+1, mesh.indices[i+1]+1,
      mesh.indices[i+2]+1, mesh.indices[i+2]+1 ) ;
  }

  fclose( f ) ;

  #ifdef _WIN32
  system( "start ." ) ; // open the folder in windows explorer
  #endif
}

#ifdef __APPLE__
KeyMap keyStates ;

bool IS_KEYDOWN( uint16_t vKey )
{
  // http://stackoverflow.com/questions/11466294/getting-keyboard-state-using-getkeys-function
  uint8_t index = vKey / 32 ;
  uint8_t shift = vKey % 32 ;
  return keyStates[index].bigEndianValue & (1 << shift) ;
}
#endif

void keys()
{
  #ifdef _WIN32
  
  // Windows makes this easy and nice..
  #define IS_KEYDOWN( c ) (GetAsyncKeyState( c ) & 0x8000)
  if( IS_KEYDOWN( 'W' ) )
    axis.pos += axis.forward * speed ;
  if( IS_KEYDOWN( 'S' ) )
    axis.pos -= axis.forward * speed ;
  if( IS_KEYDOWN( 'A' ) )
    axis.pos -= axis.right * speed ;
  if( IS_KEYDOWN( 'D' ) )
    axis.pos += axis.right * speed ;
  if( IS_KEYDOWN( 'Q' ) )
    axis.roll( -speed/15 ) ;
  if( IS_KEYDOWN( 'E' ) )
    axis.roll( speed/15 ) ;

  #elif defined __APPLE__
  
  GetKeys(keyStates) ;
  if( IS_KEYDOWN( kVK_ANSI_W ) )
    axis.pos += axis.forward * speed ;
  if( IS_KEYDOWN( kVK_ANSI_S ) )
    axis.pos -= axis.forward * speed ;
  if( IS_KEYDOWN( kVK_ANSI_A ) )
    axis.pos -= axis.right * speed ;
  if( IS_KEYDOWN( kVK_ANSI_D ) )
    axis.pos += axis.right * speed ;
  if( IS_KEYDOWN( kVK_ANSI_Q ) )
    axis.roll( -speed/15 ) ;
  if( IS_KEYDOWN( kVK_ANSI_E ) )
    axis.roll( speed/15 ) ;

  
  #endif
  }


void drawElements( int renderMode, vector<int> indices )
{
  if( repeats )
  {
    for( int i = -1 ; i <= 1 ; i++ )
    {
      for( int j = -1 ; j <= 1 ; j++ )
      {
        int k = 0 ; //for( int k = -1 ; k <= 1 ; k++ )
        {
          glPushMatrix();
          glTranslatef( i*voxelGrid.worldSize, j*voxelGrid.worldSize, k*voxelGrid.worldSize ) ;
          glDrawElements( renderMode, (int)indices.size(), GL_UNSIGNED_INT, &indices[0] ) ;
          glPopMatrix();
        }
      }
    }
  }
  else
  {
    // draw it once
    glDrawElements( renderMode, (int)indices.size(), GL_UNSIGNED_INT, &indices[0] ) ;
  }
}

void drawBoundArray( int renderMode, int size )
{
  if( repeats )
  {
    for( int i = -1 ; i <= 1 ; i++ )
    {
      for( int j = -1 ; j <= 1 ; j++ )
      {
        int k = 0 ; //for( int k = -1 ; k <= 1 ; k++ )
        {
          glPushMatrix();
          glTranslatef( i*voxelGrid.worldSize, j*voxelGrid.worldSize, k*voxelGrid.worldSize ) ;
          glDrawArrays( renderMode, 0, size ) ;
          glPopMatrix();
        }
      }
    }
  }
  else
  {
    // draw it once
    glDrawArrays( renderMode, 0, size ) ;
  }
}

void glutPuts( const char* str, Vector2f pos, const Vector4f& color )
{
  // convert x,y to canonical
  pos.y = h-pos.y ; // invert y
  float canX = 2*pos.x/w-1, canY=2*pos.y/h-1;

  glRasterPos2f( canX, canY ) ;
  glColor4fv( &color.x ) ;
  do glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, *str ); while( *(++str) ) ;
}

int glGetPolygonMode()
{
  int pMode[2];
  glGetIntegerv( GL_POLYGON_MODE, pMode ) ;
  return pMode[0]; /// the 2nd number is front&back or w/e
}

void draw()
{
  keys() ;

  //pw += 0.01f ;
  //genVoxels() ;
  glEnable( GL_DEPTH_TEST ) ;
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ) ;

  //glEnable( GL_CULL_FACE ) ;
  //glCullFace( GL_BACK ) ;

  glViewport( 0, 0, w, h ) ;
  glMatrixMode( GL_PROJECTION ) ;
  glLoadIdentity();
  //glOrtho( -5, 5, -5, 5, 5, -5 ) ;
  gluPerspective( 45.0, 1.0, 0.5, 1000.0 ) ;
  
  glMatrixMode(GL_MODELVIEW);
  
  glLoadIdentity();
  Matrix4f mat = axis.getViewingMatrix4f() ;
  glMultMatrixf( &mat.m00 ) ;

  //gluLookAt( 0, 0, sbd,   0, 0, 0,   0, 1, 0 ) ;
  //glRotatef( my, 1, 0, 0 ) ;
  //glRotatef( mx, 0, 1, 0 ) ;
  
  if( axisLinesOn )
  {
    drawAxisLines() ;
  }
  
  // Lights.
  if( lightingOn )
  {
    vector<Vector4f> lightPoses ;
    float ld = voxelGrid.worldSize ;
    lightPoses.push_back( Vector4f(  ld,  ld/2, ld, 1 ) ) ;
    lightPoses.push_back( Vector4f(  ld,  ld,   ld, 1 ) ) ;
    lightPoses.push_back( Vector4f(   0,  ld,    0, 1 ) ) ;
    lightPoses.push_back( Vector4f( -ld,   0,    0, 1 ) ) ;

    static float r = 0.f;
    r += 0.0001f;
  
    glDisable( GL_LIGHTING ) ;
  
    float white[4] = {1,1,1,1};
    for( int i = 0 ; i < lightPoses.size() ; i++ )
    {
      glEnable( GL_LIGHT0 + i ) ;
      lightPoses[i].xyz() = Matrix3f::rotationY( r ) * lightPoses[i].xyz() ;
      glLightfv( GL_LIGHT0 + i, GL_POSITION, &lightPoses[i].x ) ;
      glLightfv( GL_LIGHT0 + i, GL_DIFFUSE, white ) ;
      glLightfv( GL_LIGHT0 + i, GL_SPECULAR, white ) ;

      // visualize the light
      glPushMatrix();
      glTranslatef( lightPoses[i].x, lightPoses[i].y, lightPoses[i].z ) ;
      glutSolidSphere( voxelGrid.worldSize/10, 16, 16 ) ;
      glPopMatrix() ;
    }
  
    Vector4f spec( 1,1,1,50 ) ;
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &spec.x ) ;
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, spec.w ) ;
  
    glEnable( GL_LIGHTING ) ;
  }
  
  glEnableClientState( GL_VERTEX_ARRAY ) ;  CHECK_GL ;
  glEnableClientState( GL_NORMAL_ARRAY ) ;  CHECK_GL ;
  glEnableClientState( GL_COLOR_ARRAY ) ;  CHECK_GL ;
  glEnableClientState( GL_TEXTURE_COORD_ARRAY ) ;  CHECK_GL ;
  
  // Bind a texture to use
  glEnable( GL_TEXTURE_2D ) ;
  
  // DRAW THE MESH
  if( mesh.verts.size() )
  {
    //glDepthMask( 0 ) ;
    if( !mesh.indices.size() ) // NO INDEX BUFFER
    {
      // vertex arrays with no index buffer
      glVertexPointer( 3, GL_FLOAT, sizeof( VertexPNCT ), &mesh.verts[0].pos ) ;
      glNormalPointer( GL_FLOAT, sizeof( VertexPNCT ), &mesh.verts[0].normal ) ;
      glColorPointer( 4, GL_FLOAT, sizeof( VertexPNCT ), &mesh.verts[0].color ) ;
      glTexCoordPointer( 2, GL_FLOAT, sizeof( VertexPNCT ), &mesh.verts[0].tex ) ;
      
      if( repeats )
      {
        for( int i = -1 ; i <= 1 ; i++ )
        {
          for( int j = -1 ; j <= 1 ; j++ )
          {
            int k = 0 ; //for( int k = -1 ; k <= 1 ; k++ )
            { 
              glPushMatrix();
              glTranslatef( i*voxelGrid.worldSize, j*voxelGrid.worldSize, k*voxelGrid.worldSize ) ;
              glDrawArrays( mesh.renderMode, 0, (int)mesh.verts.size() ) ;
              glPopMatrix();
            }
          }
        }
      }
      else
        glDrawArrays( mesh.renderMode, 0, (int)mesh.verts.size() ) ;
    }
    else
    {
      // Use the index buffer if it exists
      glVertexPointer( 3, GL_FLOAT, sizeof( VertexPNCT ), &mesh.verts[0].pos ) ;
      glNormalPointer( GL_FLOAT, sizeof( VertexPNCT ), &mesh.verts[0].normal ) ;
      glColorPointer( 4, GL_FLOAT, sizeof( VertexPNCT ), &mesh.verts[0].color ) ;
      glTexCoordPointer( 2, GL_FLOAT, sizeof( VertexPNCT ), &mesh.verts[0].tex ) ;
      
      drawElements( mesh.renderMode, mesh.indices ) ;
    }
    //glDepthMask( 1 ) ;
  }
  
  glDisableClientState( GL_NORMAL_ARRAY ) ;  CHECK_GL ;
  glDisableClientState( GL_TEXTURE_COORD_ARRAY ) ;  CHECK_GL ;
  glDisable( GL_TEXTURE_2D ) ;
  glDisable( GL_LIGHTING ) ;
  
  // Draw some debug lines etc.
  if( showGradients )
  {
    glVertexPointer( 3, GL_FLOAT, sizeof( VertexPC ), &gradients[0].pos ) ;
    glColorPointer( 4, GL_FLOAT, sizeof( VertexPC ), &gradients[0].color ) ;

    drawBoundArray( GL_LINES, (int)gradients.size() ) ;
  }
  if( debugLines.size() )
  {
    glVertexPointer( 3, GL_FLOAT, sizeof( VertexPC ), &debugLines[0].pos ) ;
    glColorPointer( 4, GL_FLOAT, sizeof( VertexPC ), &debugLines[0].color ) ;
    
    drawBoundArray( GL_LINES, (int)debugLines.size() ) ;
  }
  glDisableClientState( GL_VERTEX_ARRAY ) ;
  glDisableClientState( GL_COLOR_ARRAY ) ;
  

  
  //TEXT
  glMatrixMode( GL_MODELVIEW ) ;
  glLoadIdentity() ;
  glMatrixMode( GL_PROJECTION ) ;
  glLoadIdentity();
  glDisable( GL_DEPTH_TEST ) ;
  
  // subwindow overlay
  glEnable( GL_TEXTURE_2D ) ;
  glEnableClientState( GL_VERTEX_ARRAY ) ;  CHECK_GL ;
  glEnableClientState( GL_COLOR_ARRAY ) ;  CHECK_GL ;
  glEnableClientState( GL_TEXTURE_COORD_ARRAY ) ;  CHECK_GL ;
  
  int swSize=180, swMargin=10;
  glViewport( w - swSize-swMargin, swMargin, swSize, swSize ) ;
  float m=2.f;
  static VertexPCT subWindowQuad[6] = {
    VertexPCT( Vector3f(-1,-1,0), 1, Vector2f(0,0) ),
    VertexPCT( Vector3f(1,-1,0), 1, Vector2f(m,0) ),
    VertexPCT( Vector3f(1,1,0), 1, Vector2f(m,m) ),
    
    VertexPCT( Vector3f(-1,-1,0), 1, Vector2f(0,0) ),
    VertexPCT( Vector3f(1, 1, 0), 1, Vector2f(m,m) ),
    VertexPCT( Vector3f(-1, 1, 0), 1, Vector2f(0,m) )
  } ;
  
  glVertexPointer( 3, GL_FLOAT, sizeof( VertexPCT ), &subWindowQuad[0].pos ) ;
  glColorPointer( 4, GL_FLOAT, sizeof( VertexPCT ), &subWindowQuad[0].color ) ;
  glTexCoordPointer( 2, GL_FLOAT, sizeof( VertexPCT ), &subWindowQuad[0].tex ) ;
  
  glDrawArrays( GL_TRIANGLES, 0, 6 ) ;
  
  glDisableClientState( GL_VERTEX_ARRAY ) ;  CHECK_GL ;
  glDisableClientState( GL_COLOR_ARRAY ) ;  CHECK_GL ;
  glDisableClientState( GL_TEXTURE_COORD_ARRAY ) ;  CHECK_GL ;
  glDisable( GL_TEXTURE_2D ) ;
  
  if( displayTextOn )
  {
    // back to full for text
    glViewport( 0, 0, w, h ) ;
    int polyMode ;
    glGetIntegerv( GL_POLYGON_MODE, &polyMode ) ;
    char buf[1024];
    int pos = sprintf( buf, "(!)export " ) ;
    pos += sprintf( buf+pos, " (2)%s", glGetPolygonMode()==GL_FILL?"wireframe":"solid" ) ;
    
    // if you are in pts mode, special set of options available to you
    if( vizGenMode==VizGenPts )
    {
      if( PointCloud::useCubes )
        pos += sprintf( buf+pos, " (3)render points (p/P)cubesize" ) ;
      else
        pos += sprintf( buf+pos, " (3)render cubes (p/P)ointsize" ) ;
    }
    pos += sprintf( buf+pos, repeats?" un(r)epeat":" (r)epeat" ) ;
    
    float yPos = 0.f, yi = 30.f ;
    glutPuts( buf, Vector2f( 20, yPos+=yi ), White ) ;

    int numPts = (int)mesh.verts.size() ;
    const char* ptsOrTris = "pts" ;
    if( mesh.renderMode==GL_TRIANGLES )
    {
      if( mesh.indices.size() )
        numPts = (int)mesh.indices.size()/3 ;
      else
        numPts /= 3 ;
      ptsOrTris = "tris" ;
    }
    else
    {
      
    }
    
    sprintf( buf, "(t) %s (k)gridSize=%d / %s=%d (+/-)w=%.2f (i/I)isosurface=%.2f",
      VizGenModeName[ vizGenMode ],
      voxelGrid.dims.x, ptsOrTris, numPts,
      wTerrain, isosurface ) ;
    glutPuts( buf, Vector2f(20, yPos+=yi), White ) ;
    sprintf( buf, "(n/N)minEdgeLength=%.3f (g/G)speed=%.2f (j/J)worldSize=%.2f",
      minEdgeLength, speed, voxelGrid.worldSize ) ;
    
    glutPuts( buf, Vector2f(20, h-yi), White ) ;
  }
  
  glutSwapBuffers();
}

// Called every time a window is resized to resize the projection matrix
void resizeWindow( int newWidth, int newHeight )
{
  w = newWidth ;
  h = newHeight ;
}

static int lastX=0, lastY=0 ;
static int mmode=0;
void mouseMotion( int x, int y )
{
  int dx = x-lastX, dy=y-lastY ;
  
  // LMB
  if( mmode == GLUT_LEFT_BUTTON )
  {
    mouseX += dx, mouseY += dy ;
    axis.yaw( 0.01f*dx ) ;
    axis.pitch( 0.01f*dy ) ;
  }
  else if( mmode == GLUT_RIGHT_BUTTON )
  {
  }
  
  lastX=x,lastY=y;
}

void mouse( int button, int state, int x, int y )
{
  lastX = x ;
  lastY = y ;
  
  mmode=button; // 0 for LMB, 2 for RMB
  //printf( "%d %d %d %d\n", button,state,x,y ) ;
}

void keyboard( unsigned char key, int x, int y )
{
  float diff = 0.01f ;
  
  switch( key )
  {
  case '!':
    exportOBJ( "exported.obj" ) ;
    break ;

  case '2':
    {
    int polyMode = glGetPolygonMode() ;
    if( polyMode == GL_FILL )  glPolygonMode( GL_FRONT_AND_BACK, GL_LINE ) ;
    else  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL ) ;
    }
    break ;

  case '3':
    if( vizGenMode == VizGenPts )
    {
      PointCloud::useCubes = !PointCloud::useCubes ;
      genVizFromVoxelData() ;
    }
    break ;
    
  case '4':
    EPS += 1e-2f ;
    regen() ;
    break;
  case '$':
    EPS -= 1e-2f ;
    regen() ;
    break ;
    
  case '5':
    displayTextOn = !displayTextOn ;
    break ;

  case '7':
    axisLinesOn = !axisLinesOn ;
    break ;
    
  case '=':
    wTerrain+=diff;
    regen();
    break;
  
  case '+':
    wTerrain+=10*diff;
    regen() ;
    break;
  
  case '-':
    wTerrain-=diff;
    regen();
    break; 
  
  case '_':
    wTerrain-=10*diff;
    regen();
    break;

  case 'i':
    isosurface += 0.01f ;
    regen();
    break ;

  case 'I':
    isosurface -= 0.01f ;
    regen();
    break ;

  case 'c':
    debugLines.clear() ;
    break ;

  case 'g':
    speed += 0.01f;
    break ;
  case 'G':
    speed -= 0.01f;
    break; 
  case 'h':
    showGradients = !showGradients ;
    break ;

  case 'j':
    voxelGrid.increaseWorldSize( 10 ) ;
    regen() ;
    break;

  case 'J':
    voxelGrid.increaseWorldSize( -10 ) ;
    regen() ;
    break; 

  // refine the grid
  case 'k':
    voxelGrid.increaseResolution( 1 ) ;
    regen() ;
    break ;

  case 'K':
    voxelGrid.increaseResolution( -1 ) ;
    regen() ;
    break ;

  case 'l':
    lineWidth++;
    clamp( lineWidth, 1.f, 16.f ) ;
    glLineWidth( lineWidth ) ;
    break; 
  case 'L':
    lineWidth--;
    clamp( lineWidth, 1.f, 16.f ) ;
    glLineWidth( lineWidth ) ;
    break;
    
  case 'n':
    minEdgeLength += 0.1 ;
    regen() ;
    break ;
  case 'N':
    minEdgeLength -= 0.1 ;
    regen() ;
    break ;
  case 'm':
    for( int i = 0 ;  i < mesh.verts.size() ; i++ )
      addDebugLine( mesh.verts[i].pos, Black, mesh.verts[i].pos+mesh.verts[i].normal*1, mesh.verts[i].color ) ;
    break; 

  case 'p':
    if( PointCloud::useCubes )
    {
      PointCloud::cubeSize += 1 ;
      genVizFromVoxelData() ;
    }
    else
    {
      PointCloud::ptSize++;
      ::clamp( PointCloud::ptSize, 1, 16 ) ;
      glPointSize( PointCloud::ptSize ) ;
    } 
    break ;
  case 'P':
    if( PointCloud::useCubes )
    {
      PointCloud::cubeSize -= 1 ;
      genVizFromVoxelData() ;
    }
    else
    {
      PointCloud::ptSize--;
      ::clamp( PointCloud::ptSize, 1, 16 ) ;
      glPointSize( PointCloud::ptSize ) ;
    }
    break ;
  case 'r': repeats = !repeats ; break ;
  case 't':
    cycleFlag( vizGenMode, VizGenMode::VizGenCubes, VizGenMode::VizGenPts ) ;
    regen() ;
    break; 
  case 'T':
    decycleFlag( vizGenMode, VizGenMode::VizGenCubes, VizGenMode::VizGenPts ) ;
    regen() ;
    break; 

  case 'y':
    wTexture += 0.01f ;
    mesh.vertexTexture( wTexture, wTexturePeriod, voxelGrid.worldSize, textureRepeats ) ;
    break ;

  case 'Y':
    wTexture -= 0.01f ;
    mesh.vertexTexture( wTexture, wTexturePeriod, voxelGrid.worldSize, textureRepeats ) ;
    break ;

  case 'z':
    lightingOn=!lightingOn ;
    break ;
  case 27:
    exit(0);
    break;
    
  default:
    break;
  }
}

int main( int argc, char **argv )
{
  glutInit( &argc, argv ) ; // Initializes glut

  glutInitDisplayMode( GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGBA ) ;
  glutInitWindowSize( w, h ) ;
  glutInitWindowPosition( 0, 0 ) ;
  glutCreateWindow( "Isosurface contour from 3D Perlin noise" ) ;
  glutReshapeFunc( resizeWindow ) ;
  glutDisplayFunc( draw ) ;
  glutIdleFunc( draw ) ;
  
  glutMotionFunc( mouseMotion ) ;
  glutMouseFunc( mouse ) ;
  
  glutKeyboardFunc( keyboard ) ;

  init();

  glutMainLoop();
  return 0;
}












