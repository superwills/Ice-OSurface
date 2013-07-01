#ifndef PERLIN_H
#define PERLIN_H

// Perlin
// Copyright © 2003-2011, Stefan Gustavson
//
// Contact: stegu@itn.liu.se
// Stefan Gustavson (stefan.gustavson@gmail.com)

#define FASTFLOOR(x) ( ((x)>0) ? ((int)x) : (((int)x)-1) )

// This is the new and improved, C(2) continuous interpolant
#define FADE(t) ( t * t * t * ( t * ( t * 6.f - 15.f ) + 10.f ) )
#define LERP(t, a, b) ((a) + (t)*((b)-(a)))


namespace Perlin
{
  static float grad( int hash, float x ) ;
  static float grad( int hash, float x, float y ) ;
  static float grad( int hash, float x, float y , float z ) ;
  static float grad( int hash, float x, float y, float z, float t ) ;

  static void grad1( int hash, float *gx ) ;
  static void grad2( int hash, float *gx, float *gy ) ;
  static void grad3( int hash, float *gx, float *gy, float *gz ) ;
  static void grad4( int hash, float *gx, float *gy, float *gz, float *gw) ;

  //1D, 2D, 3D and 4D float Perlin noise
  float noise( float x ) ;
  float noise( float x, float y ) ;
  float noise( float x, float y, float z ) ;
  float noise( float x, float y, float z, float w ) ;

  // PERIODIC perlin noise
  float pnoise( float x, int px ) ;
  float pnoise( float x, float y, int px, int py ) ;
  float pnoise( float x, float y, float z, int px, int py, int pz ) ;
  float pnoise( float x, float y, float z, float w, int px, int py, int pz, int pw ) ;

  // 1D simplex noise with derivative.
  // If the last argument is not null, the analytic derivative
  // is also calculated.
  float sdnoise( float x, float *dnoise_dx);

  // 2D simplex noise with derivatives.
  // If the last two arguments are not null, the analytic derivative
  // (the 2D gradient of the scalar noise field) is also calculated.
  float sdnoise( float x, float y, float *dnoise_dx, float *dnoise_dy );

  // 3D simplex noise with derivatives.
  // If the last tthree arguments are not null, the analytic derivative
  // (the 3D gradient of the scalar noise field) is also calculated.
  float sdnoise( float x, float y, float z,
                 float *dnoise_dx, float *dnoise_dy, float *dnoise_dz );

  // 4D simplex noise with derivatives.
  // If the last four arguments are not null, the analytic derivative
  // (the 4D gradient of the scalar noise field) is also calculated.
  float sdnoise( float x, float y, float z, float w,
                 float *dnoise_dx, float *dnoise_dy,
                 float *dnoise_dz, float *dnoise_dw);

};


#endif