#include "GLUtil.h"

// GL ERR
map<int,const char*> createErrMap()
{
  map<int,const char*> errmap ;
  errmap.insert( make_pair( 0x0000, "GL_NO_ERROR" ) ) ;
  errmap.insert( make_pair( 0x0500, "GL_INVALID_ENUM" ) ) ;
  errmap.insert( make_pair( 0x0501, "GL_INVALID_VALUE" ) ) ;
  errmap.insert( make_pair( 0x0502, "GL_INVALID_OPERATION" ) ) ;
  errmap.insert( make_pair( 0x0503, "GL_STACKOVERFLOW" ) ) ;
  errmap.insert( make_pair( 0x0504, "GL_STACK_UNDERFLOW" ) ) ;
  errmap.insert( make_pair( 0x0505, "GL_OUTOFMEMORY" ) ) ;
  
  errmap.insert( make_pair( 0x8CD5, "GL_FRAMEBUFFER_COMPLETE" ) ) ;
  errmap.insert( make_pair( 0x8CD6, "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT" ) ) ;
  errmap.insert( make_pair( 0x8CD7, "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT" ) ) ;
  errmap.insert( make_pair( 0x8CD9, "GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS" ) ) ;
  errmap.insert( make_pair( 0x8CDD, "GL_FRAMEBUFFER_UNSUPPORTED" ) ) ;                    
  return errmap ;
} 

map<int,const char*> glErrName = createErrMap() ;


void drawAxisLines()
{
	static vector<VertexPC> axes, axesEnds ;
  
  // init once
  if( !axes.size() )
  {
    const int gll = 160 ;  // length of purple grid lines
    // numberGridLines and gridLineLength should usually have the same value
    
    const int gridSkip = gll/10 ;// world units spacing between the grid lines
    //const int numberGridLines = worldMax-worldMin;	// number of purple grid lines
    
    bool fullAxes = 1 ; // full or half axes
    bool showWalls = 0, showFloor=1;
    const int axisLength = gll ;      // length of the axes (R, G, B for X, Y, Z)
    const float axisIntensity = 0.8f;	// color intensity of each axis

    const Vector4f gridColor( 0.25, 0.0, 0.45, 0.45 ),  // dark purple, color for gridlines
    gridColorXZ( 0.25, 0, 0.45, 0.45 ),
    gridColorXY( 0.75, 0.6, 0.25, 0.45 ),
    gridColorYZ( 0.0, 0.45, 0.45, 0.45 ) ;
    
    info( "Init axis lines.." ) ;
    if( fullAxes )
    {
      for(int t = -gll; t < 0; t+=gridSkip)		// for loop done this way to not draw axes
      {
        // first line.. parallel to z-axis, crossing x-axis
        axes.push_back( VertexPC( Vector3f( t, 0, -gll ), gridColor ) );
        axes.push_back( VertexPC( Vector3f( t, 0,  gll ), gridColor ) );

        // .. and on the other side of the z-axis
        axes.push_back( VertexPC( Vector3f( -t, 0, -gll ), gridColor ) ) ;
        axes.push_back( VertexPC( Vector3f( -t, 0,  gll ), gridColor ) ) ;
        
        // 2nd line.. parallel to x-axis, crossing z-axis
        axes.push_back( VertexPC( Vector3f( -gll, 0, t ), gridColor ) ) ;
        axes.push_back( VertexPC( Vector3f(  gll, 0, t ), gridColor ) ) ;
        
        // .. and on the other side of the x-axis
        axes.push_back( VertexPC( Vector3f( -gll, 0, -t ), gridColor ) ) ;
        axes.push_back( VertexPC( Vector3f(  gll, 0, -t ), gridColor ) ) ;
      }
		}
    else
    {
      for(int t = gridSkip; t < gll; t+=gridSkip)
      {
        // first line.. parallel to z-axis, crossing x-axis
        // LINES FROM X axis IN Z, XZ plane
        
        //floor
        axes.push_back( VertexPC( Vector3f( t, 0, 0 ), gridColorXZ ) );
        axes.push_back( VertexPC( Vector3f( t, 0, gll ), gridColorXZ ) );

        //ceiling
        axes.push_back( VertexPC( Vector3f( t, gll, 0 ), gridColorXZ ) );
        axes.push_back( VertexPC( Vector3f( t, gll, gll ), gridColorXZ ) );
        
        // Lines from Z axis in X, XZ plane
        axes.push_back( VertexPC( Vector3f( 0,   0, t ), gridColorXZ ) );
        axes.push_back( VertexPC( Vector3f( gll, 0, t ), gridColorXZ ) );

        axes.push_back( VertexPC( Vector3f( 0,   gll, t ), gridColorXZ ) );
        axes.push_back( VertexPC( Vector3f( gll, gll, t ), gridColorXZ ) );

        
        // LINES FROM X axis IN Y, XY plane
        axes.push_back( VertexPC( Vector3f( t,   0, 0 ), gridColorXY ) );
        axes.push_back( VertexPC( Vector3f( t, gll, 0 ), gridColorXY ) );
        
        axes.push_back( VertexPC( Vector3f( t,   0, gll ), gridColorXY ) );
        axes.push_back( VertexPC( Vector3f( t, gll, gll ), gridColorXY ) );
        
        
        // Lines from Y axis in X, XY plane
        axes.push_back( VertexPC( Vector3f( 0,   t, 0 ), gridColorXY ) ) ;
        axes.push_back( VertexPC( Vector3f( gll, t, 0 ), gridColorXY ) ) ;
        
        axes.push_back( VertexPC( Vector3f( 0,   t, gll ), gridColorXY ) ) ;
        axes.push_back( VertexPC( Vector3f( gll, t, gll ), gridColorXY ) ) ;
        
        
        // Lines from Y axis in Z, YZ plane
        axes.push_back( VertexPC( Vector3f( 0, t,   0 ), gridColorYZ ) );
        axes.push_back( VertexPC( Vector3f( 0, t, gll ), gridColorYZ ) );
        
        axes.push_back( VertexPC( Vector3f( gll, t,   0 ), gridColorYZ ) );
        axes.push_back( VertexPC( Vector3f( gll, t, gll ), gridColorYZ ) );
        
        
        
        // Lines from Z axis in Y, YZ plane
        axes.push_back( VertexPC( Vector3f( 0,   0, t ), gridColorYZ ) );
        axes.push_back( VertexPC( Vector3f( 0, gll, t ), gridColorYZ ) );
        
        axes.push_back( VertexPC( Vector3f( gll,   0, t ), gridColorYZ ) );
        axes.push_back( VertexPC( Vector3f( gll, gll, t ), gridColorYZ ) );
        
      }
    }
		
    // Now draw the axes themselves:
    int fullAxis=0 ;
		// a red line for the x-axis
    axes.push_back( VertexPC( Vector3f( fullAxis*-axisLength, 0, 0 ), Vector4f( axisIntensity, 0.0f, 0.0f ) ) );
    axes.push_back( VertexPC( Vector3f(  axisLength, 0, 0 ), Vector4f( axisIntensity, 0.0f, 0.0f ) ) );

		// a green line for the y-axis
		axes.push_back( VertexPC( Vector3f(  0, fullAxis*-axisLength, 0 ), Vector4f( 0.0f, axisIntensity, 0.0f ) ) ) ;
    axes.push_back( VertexPC( Vector3f(  0,  axisLength, 0 ), Vector4f( 0.0f, axisIntensity, 0.0f ) ) ) ;
		
		// a blue line for the z-axis
		axes.push_back( VertexPC( Vector3f( 0, 0, fullAxis*-axisLength ), Vector4f(	0.0f, 0.0f, axisIntensity ) ) ) ;
 		axes.push_back( VertexPC( Vector3f( 0, 0,  axisLength ), Vector4f(	0.0f, 0.0f, axisIntensity ) ) ) ;
    
    
    // add a large point to the end of each axis
   
    // a red dot for the x-axis
    axesEnds.push_back( VertexPC( Vector3f( axisLength, 0, 0 ), Vector4f( axisIntensity, 0.0f, 0.0f ) ) ) ;
    // a green dot for the y-axis
    axesEnds.push_back( VertexPC( Vector3f( 0, axisLength, 0 ), Vector4f( 0.0f, axisIntensity, 0.0f ) ) ) ;
    // a blue dot for the z-axis
    axesEnds.push_back( VertexPC( Vector3f( 0, 0, axisLength ), Vector4f(	0.0f, 0.0f, axisIntensity ) ) ) ;
    
	}
  

  glEnableClientState( GL_VERTEX_ARRAY ) ;  CHECK_GL ;
  glEnableClientState( GL_COLOR_ARRAY ) ;  CHECK_GL ;
  
  glDisable( GL_LIGHTING ) ;
  
  //glLineWidth( 4.0f );	// thicken lines
  glVertexPointer( 3, GL_FLOAT, sizeof( VertexPC ), &axes[0].pos ) ;
  glColorPointer( 4, GL_FLOAT, sizeof( VertexPC ), &axes[0].color ) ;
  glDrawArrays( GL_LINES, 0, axes.size() ) ;
  
  glVertexPointer( 3, GL_FLOAT, sizeof( VertexPC ), &axesEnds[0].pos ) ;
  glColorPointer( 4, GL_FLOAT, sizeof( VertexPC ), &axesEnds[0].color ) ;
  glDrawArrays( GL_POINTS, 0, axesEnds.size() ) ;
  
  glDisableClientState( GL_VERTEX_ARRAY ) ;  CHECK_GL ;
  glDisableClientState( GL_COLOR_ARRAY ) ;  CHECK_GL ;
  
  //glLineWidth( 1.0f );
}
