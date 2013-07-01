#ifndef GLUTIL_H
#define GLUTIL_H

#ifdef _WIN32
#include <stdlib.h> // MUST BE BEFORE GLUT ON WINDOWS
#include <gl/glut.h>
#else
#include <OpenGL/gl.h>
#endif
#include "StdWilUtil.h"
#include "Vectorf.h"

extern map<int,const char*> glErrName ;

inline bool GL_OK()
{
  GLenum err = glGetError() ;
  if( err != GL_NO_ERROR )
    error( "GLERROR %d %s", err, glErrName[ err ] ) ;
  return err == GL_NO_ERROR ;
}

inline bool GL_OK( int line, const char* file )
{
  GLenum err = glGetError() ;
  if( err != GL_NO_ERROR )
    error( "GLERROR %d %s, line=%d of file=%s", err, glErrName[ err ], line, file ) ;
  return err == GL_NO_ERROR ;
}

inline bool CHECK( bool cond, const char* errMsg )
{
  if( !cond )  error( errMsg ) ;
  return cond ;
}

// WITH LINE NUMBERS
#define CHECK_GL GL_OK( __LINE__, __FILE__ ) 

void drawAxisLines() ;



#endif