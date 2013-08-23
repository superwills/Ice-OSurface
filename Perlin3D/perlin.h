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
  
  // Harmonic summing functions.
  // persistence, lacunarity, octaves
  // You want to add multiple octaves of noise.
  // so you add: noise(f) + 1/2*noise(2*f) + 1/4*noise(4*f)..
  // octaveScaleFactor (aka `persistence`) is how much to MULTIPLY each successive octave by (should be <1)
  // freqMult (aka `lacunarity`) is the spacing between "octaves" (if it is not 2, then it technically isn't an "octave")
  // numFreqs (aka `octaves`) is the NUMBER of harmonics to add.
  //
  // See here for pictures & definitions of these terms http://libnoise.sourceforge.net/glossary/
  float hnoise1( float x, int px, float octaveScaleFactor, float freqMult, int numFreqs ) ;

  // To wrap on a sphere:  use theta and phi for x and y
  // Brushed metal: vary only ONE of x,y
  //   (eg PerlinNoise2D( uv.x, _1_, 2, 2, 8 ) // (the 1 stays constant across the 2d face)
  //   (OR you could just generate a line of 1d noise and copy it)
  float hnoise2( float x, float y, int px, int py, float octaveScaleFactor, float freqMult, int numFreqs ) ;
  
  float hnoise3(float x, float y, float z, int px, int py, int pz, float octaveScaleFactor, float freqMult, int numFreqs ) ;

};


#endif