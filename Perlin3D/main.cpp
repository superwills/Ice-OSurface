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
#endif
#include "perlin.h"
#include "GLUtil.h"
#include "StdWilUtil.h"
#include "Vectorf.h"
#include "Geometry.h"
#include <vector>
#include <functional>
using namespace std;



// For naming the axis edge a vertex is on
//              0   1   2   3   4   5
enum AxisEdge{ PX, NX, PY, NY, PZ, NZ } ;
Vector4f AxisEdgeColors[] = {
  Vector4f(1,0,0,1), Vector4f(1,0,0,1),
  Vector4f(0,1,0,1), Vector4f(0,1,0,1),
  Vector4f(0,0,1,1), Vector4f(0,0,1,1)
} ;

#define OTHERAXIS1(axis) ((axis+1)%3)
#define OTHERAXIS2(axis) ((axis+2)%3)

// EVEN AXES have (AxisEdge%2==0).
// YOUR AXIS INDEX is AxisEdge/2 (PX,NX=>0, PY,NY=>1, PZ,NZ=>2)

// You need a fat epsilon for the edge-collapsing vertex merge. 1e-6f is TOO NARROW
float EPS = 1e-3f;
float w=768.f, h=768.f ;
static float mouseX, mouseY, sbd=150.f,
  pw=2.59,//0.58 
  isosurface=0.38, // the value of the isosurface
  isosurfaceThickness=0.1 // how thick the isosurface is to be AROUND isosurface
;
static int pwPeriod=8;

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

#include "VoxelGrid.h"
#include "Mesh.h"
#include "PointCloud.h"
#include "MarchingTets.h"
#include "MarchingCubes.h"

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





// globals specific to main.cpp
enum VizGenMode { VizGenCubes, VizGenTets, VizGenPts } ;
const char* VizGenModeName[] = { "VizGenCubes", "VizGenTets", "VizGenPts" } ;
int vizGenMode = VizGenCubes ;

// RENDERING OPTIONS:
float lineWidth=1.f ;

bool showGradients=0 ; // not used since gradients not generated here

bool repeats = 0 ;  // Show the world repeated (key 'r')
Axis axis ;         // for moving around in space
float speed=0.02f ; // (key 'g'): movement speed






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
    PointCloud pc( &voxelGrid, &mesh.verts, isosurface ) ; 
    if( !pc.useCubes ) mesh.renderMode = GL_POINTS ;
    pc.genVizPunchthru() ;
  }
  else if( vizGenMode == VizGenTets )
  {
    MarchingTets mt( &voxelGrid, &mesh.verts, isosurface, Blue ) ;
    mt.genVizMarchingTets() ;
    mesh.smoothMesh( voxelGrid ) ;
  }
  else
  {
    MarchingCubes mc( &voxelGrid, &mesh.verts, isosurface, Blue ) ;
    mc.genVizMarchingCubes() ;
    mesh.smoothMesh( voxelGrid ) ;
  }
  
  
}

void regen()
{
  voxelGrid.genData() ;    
  genVizFromVoxelData() ;
}

void init() // Called before main loop to set up the program
{
  //initDirections() ;
  regen() ;
  
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

void keys()
{
  #define IS_KEYDOWN( c ) (GetAsyncKeyState( c ) & 0x8000)
  if( IS_KEYDOWN( 'W' ) )
  {
    axis.pos += axis.forward * speed ;
  }
  
  if( IS_KEYDOWN( 'S' ) )
  {
    axis.pos -= axis.forward * speed ;
  }

  if( IS_KEYDOWN( 'A' ) )
  {
    axis.pos -= axis.right * speed ;
  }

  if( IS_KEYDOWN( 'D' ) )
  {
    axis.pos += axis.right * speed ;
  }

  if( IS_KEYDOWN( 'Q' ) )
  {
    axis.roll( -speed/15 ) ;
  }

  if( IS_KEYDOWN( 'E' ) )
  {
    axis.roll( speed/15 ) ;
  }

}


void drawElements( int renderMode, vector<int> indices )
{
  if( repeats )
  {
    for( int i = -1 ; i <= 1 ; i++ )
    {
      for( int j = -1 ; j <= 1 ; j++ )
      {
        glPushMatrix();
        glTranslatef( i*voxelGrid.worldSize, j*voxelGrid.worldSize,0 ) ;
        glDrawElements( renderMode, indices.size(), GL_UNSIGNED_INT, &indices[0] ) ;
        glPopMatrix();
      }
    }
  }
  else
  {
    // draw it once
    glDrawElements( renderMode, indices.size(), GL_UNSIGNED_INT, &indices[0] ) ;
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
        glPushMatrix();
        glTranslatef( i*voxelGrid.worldSize, j*voxelGrid.worldSize,0 ) ;
        glDrawArrays( renderMode, 0, size ) ;
        glPopMatrix();
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
  
  glLineWidth( lineWidth ) ;
  drawAxisLines() ;
  
  vector<Vector4f> lightPoses ;

  static float ld = voxelGrid.worldSize ;
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

  glEnableClientState( GL_VERTEX_ARRAY ) ;  CHECK_GL ;
  glEnableClientState( GL_NORMAL_ARRAY ) ;  CHECK_GL ;
  glEnableClientState( GL_COLOR_ARRAY ) ;  CHECK_GL ;
  

  // DRAW THE MESH
  if( mesh.verts.size() )
  {
    //glDepthMask( 0 ) ;
    if( !mesh.indices.size() ) // NO INDEX BUFFER
    {
      // vertex arrays with no index buffer
      glVertexPointer( 3, GL_FLOAT, sizeof( VertexPNC ), &mesh.verts[0].pos ) ;
      glNormalPointer( GL_FLOAT, sizeof( VertexPNC ), &mesh.verts[0].normal ) ;
      glColorPointer( 4, GL_FLOAT, sizeof( VertexPNC ), &mesh.verts[0].color ) ;

      if( repeats )
      {
        for( int i = -1 ; i <= 1 ; i++ )
        {
          for( int j = -1 ; j <= 1 ; j++ )
          {
            glPushMatrix();
            glTranslatef( i*voxelGrid.worldSize, j*voxelGrid.worldSize,0 ) ;
            glDrawArrays( mesh.renderMode, 0, (int)mesh.verts.size() ) ;
            glPopMatrix();
          }
        }
      }
      else
        glDrawArrays( mesh.renderMode, 0, (int)mesh.verts.size() ) ;
    }
    else
    {
      // Use the index buffer if it exists
      glVertexPointer( 3, GL_FLOAT, sizeof( VertexPNC ), &mesh.verts[0].pos ) ;
      glNormalPointer( GL_FLOAT, sizeof( VertexPNC ), &mesh.verts[0].normal ) ;
      glColorPointer( 4, GL_FLOAT, sizeof( VertexPNC ), &mesh.verts[0].color ) ;

      drawElements( mesh.renderMode, mesh.indices ) ;
    }
    //glDepthMask( 1 ) ;
  }
  
  glDisableClientState( GL_NORMAL_ARRAY ) ;
  glDisable( GL_LIGHTING ) ;

  if( showGradients )
  {
    glVertexPointer( 3, GL_FLOAT, sizeof( VertexPC ), &gradients[0].pos ) ;
    glColorPointer( 4, GL_FLOAT, sizeof( VertexPC ), &gradients[0].color ) ;

    drawBoundArray( GL_LINES, gradients.size() ) ;
  }
  if( debugLines.size() )
  {
    glVertexPointer( 3, GL_FLOAT, sizeof( VertexPC ), &debugLines[0].pos ) ;
    glColorPointer( 4, GL_FLOAT, sizeof( VertexPC ), &debugLines[0].color ) ;
    
    drawBoundArray( GL_LINES, debugLines.size() ) ;
  }
  glDisableClientState( GL_VERTEX_ARRAY ) ;
  glDisableClientState( GL_COLOR_ARRAY ) ;
  

  
  //TEXT
  glMatrixMode( GL_MODELVIEW ) ;
  glLoadIdentity() ;
  glMatrixMode( GL_PROJECTION ) ;
  glLoadIdentity();
  glDisable( GL_DEPTH_TEST ) ;
  
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

  int numPts = mesh.verts.size() ;
  const char* ptsOrTris = "pts" ;
  if( mesh.renderMode==GL_TRIANGLES )
  {
    numPts /= 3 ;
    ptsOrTris = "tris" ;
  }
  else
  {
    
  }
  
  sprintf( buf, "(t) %s (k)gridSize=%d / %s=%d (+/-)w=%.2f (i/I)isosurface=%.2f",
    VizGenModeName[ vizGenMode ],
    voxelGrid.dims.x, ptsOrTris, numPts,
    pw, isosurface ) ;
  glutPuts( buf, Vector2f(20, yPos+=yi), White ) ;
  sprintf( buf, "(n/N)minEdgeLength=%.3f (g/G)speed=%.2f (j/J)worldSize=%.2f",
    Mesh::minEdgeLength, speed, voxelGrid.worldSize ) ;
  const char * p = buf ;
  glutPuts( buf, Vector2f(20, h-yi), White ) ;

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
    // dolly
    sbd +=0.1*(-dx+dy) ;
    clamp( sbd, 10, 1000 ) ;
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
  case '=':
    pw+=diff;
    regen();
    break;
  
  case '+':
    pw+=10*diff;
    regen() ;
    break;
  
  case '-':
    pw-=diff;
    regen();
    break; 
  
  case '_':
    pw-=10*diff;
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
    break; 
  case 'L':
    lineWidth--;
    clamp( lineWidth, 1.f, 16.f ) ;
    break;
    
  case 'n':
    Mesh::minEdgeLength += 0.1 ;
    regen() ;
    break ;
  case 'N':
    Mesh::minEdgeLength -= 0.1 ;
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












