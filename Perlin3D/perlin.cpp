#include "perlin.h"


//---------------------------------------------------------------------
// Static data

/*
* Permutation table. This is just a random jumble of all numbers 0-255,
* repeated twice to avoid wrapping the index at 255 for each lookup.
* This needs to be exactly the same for all instances on all platforms,
* so it's easiest to just keep it as static explicit data.
* This also removes the need for any initialisation of this class.
*
* Note that making this an int[] instead of a char[] might make the
* code run faster on platforms with a high penalty for unaligned single
* byte addressing. Intel x86 is generally single-byte-friendly, but
* some other CPUs are faster with 4-aligned reads.
* However, a char[] is smaller, which avoids cache trashing, and that
* is probably the most important aspect on most architectures.
* This array is accessed a *lot* by the noise functions.
* A vector-valued noise over 3D accesses it 96 times, and a
* float-valued 4D noise 64 times. We want this to fit in the cache!
*/
static unsigned char perm[512] = {151,160,137,91,90,15,
  131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
  190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
  88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
  77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
  102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
  135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
  5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
  223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
  129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
  251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
  49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
  138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180,
  151,160,137,91,90,15,
  131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
  190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
  88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
  77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
  102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
  135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
  5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
  223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
  129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
  251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
  49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
  138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180 
};
//---------------------------------------------------------------------

/*
* Helper functions to compute gradients-dot-residualvectors (1D to 4D)
* Note that these generate gradients of more than unit length. To make
* a close match with the value range of classic Perlin noise, the final
* noise values need to be rescaled to fit nicely within [-1,1].
* (The simplex noise functions as such also have different scaling.)
* Note also that these noise functions are the most practical and useful
* signed version of Perlin noise. To return values according to the
* RenderMan specification from the SL noise() and pnoise() functions,
* the noise values need to be scaled and offset to [0,1], like this:
* float SLnoise = (Perlin::noise(x,y,z) + 1.0) * 0.5;
*/

// Skewing factors for 2D simplex grid:
// F2 = 0.5*(sqrt(3.0)-1.0)
// G2 = (3.0-Math.sqrt(3.0))/6.0
const static float F2 = 0.366025403f ; // F2 = 0.5*(sqrt(3.0)-1.0)
const static float G2 = 0.211324865f ; // G2 = (3.0-Math.sqrt(3.0))/6.0

// Skewing factors for 3D simplex grid:
// F3 = 1/3
// G3 = 1/6
const static float F3 = 0.333333333f ;
const static float G3 = 0.166666667f ;

// The skewing and unskewing factors are hairy again for the 4D case
const static float F4 = 0.309016994f ; // F4 = (Math.sqrt(5.0)-1.0)/4.0
const static float G4 = 0.138196601f ; // G4 = (5.0-Math.sqrt(5.0))/20.0



/*
* Gradient tables. These could be programmed the Ken Perlin way with
* some clever bit-twiddling, but this is more clear, and not really slower.
*/
static float grad2lut[8][2] = {
  { -1.0f, -1.0f }, { 1.0f, 0.0f } , { -1.0f, 0.0f } , { 1.0f, 1.0f } ,
  { -1.0f, 1.0f } , { 0.0f, -1.0f } , { 0.0f, 1.0f } , { 1.0f, -1.0f }
};

/*
* Gradient directions for 3D.
* These vectors are based on the midpoints of the 12 edges of a cube.
* A larger array of random unit length vectors would also do the job,
* but these 12 (including 4 repeats to make the array length a power
* of two) work better. They are not random, they are carefully chosen
* to represent a small, isotropic set of directions.
*/

static float grad3lut[16][3] = {
  { 1.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f }, // 12 cube edges
  { -1.0f, 0.0f, 1.0f }, { 0.0f, -1.0f, 1.0f },
  { 1.0f, 0.0f, -1.0f }, { 0.0f, 1.0f, -1.0f },
  { -1.0f, 0.0f, -1.0f }, { 0.0f, -1.0f, -1.0f },
  { 1.0f, -1.0f, 0.0f }, { 1.0f, 1.0f, 0.0f },
  { -1.0f, 1.0f, 0.0f }, { -1.0f, -1.0f, 0.0f },
  { 1.0f, 0.0f, 1.0f }, { -1.0f, 0.0f, 1.0f }, // 4 repeats to make 16
  { 0.0f, 1.0f, -1.0f }, { 0.0f, -1.0f, -1.0f }
};

static float grad4lut[32][4] = {
  { 0.0f, 1.0f, 1.0f, 1.0f }, { 0.0f, 1.0f, 1.0f, -1.0f }, { 0.0f, 1.0f, -1.0f, 1.0f }, { 0.0f, 1.0f, -1.0f, -1.0f }, // 32 tesseract edges
  { 0.0f, -1.0f, 1.0f, 1.0f }, { 0.0f, -1.0f, 1.0f, -1.0f }, { 0.0f, -1.0f, -1.0f, 1.0f }, { 0.0f, -1.0f, -1.0f, -1.0f },
  { 1.0f, 0.0f, 1.0f, 1.0f }, { 1.0f, 0.0f, 1.0f, -1.0f }, { 1.0f, 0.0f, -1.0f, 1.0f }, { 1.0f, 0.0f, -1.0f, -1.0f },
  { -1.0f, 0.0f, 1.0f, 1.0f }, { -1.0f, 0.0f, 1.0f, -1.0f }, { -1.0f, 0.0f, -1.0f, 1.0f }, { -1.0f, 0.0f, -1.0f, -1.0f },
  { 1.0f, 1.0f, 0.0f, 1.0f }, { 1.0f, 1.0f, 0.0f, -1.0f }, { 1.0f, -1.0f, 0.0f, 1.0f }, { 1.0f, -1.0f, 0.0f, -1.0f },
  { -1.0f, 1.0f, 0.0f, 1.0f }, { -1.0f, 1.0f, 0.0f, -1.0f }, { -1.0f, -1.0f, 0.0f, 1.0f }, { -1.0f, -1.0f, 0.0f, -1.0f },
  { 1.0f, 1.0f, 1.0f, 0.0f }, { 1.0f, 1.0f, -1.0f, 0.0f }, { 1.0f, -1.0f, 1.0f, 0.0f }, { 1.0f, -1.0f, -1.0f, 0.0f },
  { -1.0f, 1.0f, 1.0f, 0.0f }, { -1.0f, 1.0f, -1.0f, 0.0f }, { -1.0f, -1.0f, 1.0f, 0.0f }, { -1.0f, -1.0f, -1.0f, 0.0f }
};

/*
* Helper functions to compute gradients in 1D to 4D
* and gradients-dot-residualvectors in 2D to 4D.
*/
float Perlin::grad( int hash, float x ) 
{
  int h = hash & 15;
  float grad = 1.0f + (h & 7);   // Gradient value 1.0, 2.0, ..., 8.0
  if (h&8) grad = -grad;         // Set a random sign for the gradient
  return ( grad * x );           // Multiply the gradient with the distance
}

void Perlin::grad1( int hash, float *gx )
{
  int h = hash & 15;
  *gx = 1.0f + (h & 7);   // Gradient value is one of 1.0, 2.0, ..., 8.0
  if (h&8) *gx = - *gx;   // Make half of the gradients negative
}

// 2d
float Perlin::grad( int hash, float x, float y ) 
{
  int h = hash & 7;      // Convert low 3 bits of hash code
  float u = h<4 ? x : y;  // into 8 simple gradient directions,
  float v = h<4 ? y : x;  // and compute the dot product with (x,y).
  return ((h&1)? -u : u) + ((h&2)? -2.0f*v : 2.0f*v);
}

void Perlin::grad2( int hash, float *gx, float *gy )
{
  int h = hash & 7;
  *gx = grad2lut[h][0];
  *gy = grad2lut[h][1];
  return;
}

// 3d
float Perlin::grad( int hash, float x, float y , float z ) 
{
  int h = hash & 15;     // Convert low 4 bits of hash code into 12 simple
  float u = h<8 ? x : y; // gradient directions, and compute dot product.
  float v = h<4 ? y : h==12||h==14 ? x : z; // Fix repeats at h = 12 to 15
  return ((h&1)? -u : u) + ((h&2)? -v : v);
}

void Perlin::grad3( int hash, float *gx, float *gy, float *gz )
{
  int h = hash & 15;
  *gx = grad3lut[h][0];
  *gy = grad3lut[h][1];
  *gz = grad3lut[h][2];
  return;
}

// 4d
float Perlin::grad( int hash, float x, float y, float z, float t )
{
  int h = hash & 31;      // Convert low 5 bits of hash code into 32 simple
  float u = h<24 ? x : y; // gradient directions, and compute dot product.
  float v = h<16 ? y : z;
  float w = h<8 ? z : t;
  return ((h&1)? -u : u) + ((h&2)? -v : v) + ((h&4)? -w : w);
}

void Perlin::grad4( int hash, float *gx, float *gy, float *gz, float *gw)
{
  int h = hash & 31;
  *gx = grad4lut[h][0];
  *gy = grad4lut[h][1];
  *gz = grad4lut[h][2];
  *gw = grad4lut[h][3];
  return;
}

// A lookup table to traverse the simplex around a given point in 4D.
// Details can be found where this table is used, in the 4D noise method.
/* TODO: This should not be required, backport it from Bill's GLSL code! */
static unsigned char simplex[64][4] = {
  {0,1,2,3},{0,1,3,2},{0,0,0,0},{0,2,3,1},{0,0,0,0},{0,0,0,0},{0,0,0,0},{1,2,3,0},
  {0,2,1,3},{0,0,0,0},{0,3,1,2},{0,3,2,1},{0,0,0,0},{0,0,0,0},{0,0,0,0},{1,3,2,0},
  {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
  {1,2,0,3},{0,0,0,0},{1,3,0,2},{0,0,0,0},{0,0,0,0},{0,0,0,0},{2,3,0,1},{2,3,1,0},
  {1,0,2,3},{1,0,3,2},{0,0,0,0},{0,0,0,0},{0,0,0,0},{2,0,3,1},{0,0,0,0},{2,1,3,0},
  {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
  {2,0,1,3},{0,0,0,0},{0,0,0,0},{0,0,0,0},{3,0,1,2},{3,0,2,1},{0,0,0,0},{3,1,2,0},
  {2,1,0,3},{0,0,0,0},{0,0,0,0},{0,0,0,0},{3,1,0,2},{0,0,0,0},{3,2,0,1},{3,2,1,0}};




// 1D simplex noise
float Perlin::noise(float x) 
{
  int i0 = FASTFLOOR(x);
  int i1 = i0 + 1;
  float x0 = x - i0;
  float x1 = x0 - 1.0f;

  float n0, n1;

  float t0 = 1.0f - x0*x0;
  // if(t0 < 0.0f) t0 = 0.0f;
  t0 *= t0;
  n0 = t0 * t0 * grad(perm[i0 & 0xff], x0);

  float t1 = 1.0f - x1*x1;
  // if(t1 < 0.0f) t1 = 0.0f;
  t1 *= t1;
  n1 = t1 * t1 * grad(perm[i1 & 0xff], x1);
  // The maximum value of this noise is 8*(3/4)^4 = 2.53125
  // A factor of 0.395 would scale to fit exactly within [-1,1], but
  // we want to match PRMan's 1D noise, so we scale it down some more.
  return 0.25f * (n0 + n1);
}

// 2D simplex noise
float Perlin::noise(float x, float y) 
{
  float n0, n1, n2; // Noise contributions from the three corners

  // Skew the input space to determine which simplex cell we're in
  float s = (x+y)*F2; // Hairy factor for 2D
  float xs = x + s;
  float ys = y + s;
  int i = FASTFLOOR(xs);
  int j = FASTFLOOR(ys);

  float t = (float)(i+j)*G2;
  float X0 = i-t; // Unskew the cell origin back to (x,y) space
  float Y0 = j-t;
  float x0 = x-X0; // The x,y distances from the cell origin
  float y0 = y-Y0;

  // For the 2D case, the simplex shape is an equilateral triangle.
  // Determine which simplex we are in.
  int i1, j1; // Offsets for second (middle) corner of simplex in (i,j) coords
  if(x0>y0) {i1=1; j1=0;} // lower triangle, XY order: (0,0)->(1,0)->(1,1)
  else {i1=0; j1=1;}      // upper triangle, YX order: (0,0)->(0,1)->(1,1)

  // A step of (1,0) in (i,j) means a step of (1-c,-c) in (x,y), and
  // a step of (0,1) in (i,j) means a step of (-c,1-c) in (x,y), where
  // c = (3-sqrt(3))/6

  float x1 = x0 - i1 + G2; // Offsets for middle corner in (x,y) unskewed coords
  float y1 = y0 - j1 + G2;
  float x2 = x0 - 1.0f + 2.0f * G2; // Offsets for last corner in (x,y) unskewed coords
  float y2 = y0 - 1.0f + 2.0f * G2;

  // Wrap the integer indices at 256, to avoid indexing perm[] out of bounds
  int ii = i % 256;
  int jj = j % 256;

  // Calculate the contribution from the three corners
  float t0 = 0.5f - x0*x0-y0*y0;
  if(t0 < 0.0f) n0 = 0.0f;
  else {
    t0 *= t0;
    n0 = t0 * t0 * grad(perm[ii+perm[jj]], x0, y0); 
  }

  float t1 = 0.5f - x1*x1-y1*y1;
  if(t1 < 0.0f) n1 = 0.0f;
  else {
    t1 *= t1;
    n1 = t1 * t1 * grad(perm[ii+i1+perm[jj+j1]], x1, y1);
  }

  float t2 = 0.5f - x2*x2-y2*y2;
  if(t2 < 0.0f) n2 = 0.0f;
  else {
    t2 *= t2;
    n2 = t2 * t2 * grad(perm[ii+1+perm[jj+1]], x2, y2);
  }

  // Add contributions from each corner to get the final noise value.
  // The result is scaled to return values in the interval [-1,1].
  return 40.0f * (n0 + n1 + n2); // TODO: The scale factor is preliminary!
}

// 3D simplex noise
float Perlin::noise(float x, float y, float z) 
{
  float n0, n1, n2, n3; // Noise contributions from the four corners

  // Skew the input space to determine which simplex cell we're in
  float s = (x+y+z)*F3; // Very nice and simple skew factor for 3D
  float xs = x+s;
  float ys = y+s;
  float zs = z+s;
  int i = FASTFLOOR(xs);
  int j = FASTFLOOR(ys);
  int k = FASTFLOOR(zs);

  float t = (float)(i+j+k)*G3; 
  float X0 = i-t; // Unskew the cell origin back to (x,y,z) space
  float Y0 = j-t;
  float Z0 = k-t;
  float x0 = x-X0; // The x,y,z distances from the cell origin
  float y0 = y-Y0;
  float z0 = z-Z0;

  // For the 3D case, the simplex shape is a slightly irregular tetrahedron.
  // Determine which simplex we are in.
  int i1, j1, k1; // Offsets for second corner of simplex in (i,j,k) coords
  int i2, j2, k2; // Offsets for third corner of simplex in (i,j,k) coords

  /* This code would benefit from a backport from the GLSL version! */
  if(x0>=y0) {
    if(y0>=z0)
    { i1=1; j1=0; k1=0; i2=1; j2=1; k2=0; } // X Y Z order
    else if(x0>=z0) { i1=1; j1=0; k1=0; i2=1; j2=0; k2=1; } // X Z Y order
    else { i1=0; j1=0; k1=1; i2=1; j2=0; k2=1; } // Z X Y order
  }
  else { // x0<y0
    if(y0<z0) { i1=0; j1=0; k1=1; i2=0; j2=1; k2=1; } // Z Y X order
    else if(x0<z0) { i1=0; j1=1; k1=0; i2=0; j2=1; k2=1; } // Y Z X order
    else { i1=0; j1=1; k1=0; i2=1; j2=1; k2=0; } // Y X Z order
  }

  // A step of (1,0,0) in (i,j,k) means a step of (1-c,-c,-c) in (x,y,z),
  // a step of (0,1,0) in (i,j,k) means a step of (-c,1-c,-c) in (x,y,z), and
  // a step of (0,0,1) in (i,j,k) means a step of (-c,-c,1-c) in (x,y,z), where
  // c = 1/6.

  float x1 = x0 - i1 + G3; // Offsets for second corner in (x,y,z) coords
  float y1 = y0 - j1 + G3;
  float z1 = z0 - k1 + G3;
  float x2 = x0 - i2 + 2.0f*G3; // Offsets for third corner in (x,y,z) coords
  float y2 = y0 - j2 + 2.0f*G3;
  float z2 = z0 - k2 + 2.0f*G3;
  float x3 = x0 - 1.0f + 3.0f*G3; // Offsets for last corner in (x,y,z) coords
  float y3 = y0 - 1.0f + 3.0f*G3;
  float z3 = z0 - 1.0f + 3.0f*G3;

  // Wrap the integer indices at 256, to avoid indexing perm[] out of bounds
  int ii = i % 256;
  int jj = j % 256;
  int kk = k % 256;

  // Calculate the contribution from the four corners
  float t0 = 0.6f - x0*x0 - y0*y0 - z0*z0;
  if(t0 < 0.0f) n0 = 0.0f;
  else {
    t0 *= t0;
    n0 = t0 * t0 * grad(perm[ii+perm[jj+perm[kk]]], x0, y0, z0);
  }

  float t1 = 0.6f - x1*x1 - y1*y1 - z1*z1;
  if(t1 < 0.0f) n1 = 0.0f;
  else {
    t1 *= t1;
    n1 = t1 * t1 * grad(perm[ii+i1+perm[jj+j1+perm[kk+k1]]], x1, y1, z1);
  }

  float t2 = 0.6f - x2*x2 - y2*y2 - z2*z2;
  if(t2 < 0.0f) n2 = 0.0f;
  else {
    t2 *= t2;
    n2 = t2 * t2 * grad(perm[ii+i2+perm[jj+j2+perm[kk+k2]]], x2, y2, z2);
  }

  float t3 = 0.6f - x3*x3 - y3*y3 - z3*z3;
  if(t3<0.0f) n3 = 0.0f;
  else {
    t3 *= t3;
    n3 = t3 * t3 * grad(perm[ii+1+perm[jj+1+perm[kk+1]]], x3, y3, z3);
  }

  // Add contributions from each corner to get the final noise value.
  // The result is scaled to stay just inside [-1,1]
  return 32.0f * (n0 + n1 + n2 + n3); // TODO: The scale factor is preliminary!
}

// 4D simplex noise
float Perlin::noise(float x, float y, float z, float w)
{
  float n0, n1, n2, n3, n4; // Noise contributions from the five corners

  // Skew the (x,y,z,w) space to determine which cell of 24 simplices we're in
  float s = (x + y + z + w) * F4; // Factor for 4D skewing
  float xs = x + s;
  float ys = y + s;
  float zs = z + s;
  float ws = w + s;
  int i = FASTFLOOR(xs);
  int j = FASTFLOOR(ys);
  int k = FASTFLOOR(zs);
  int l = FASTFLOOR(ws);

  float t = (i + j + k + l) * G4; // Factor for 4D unskewing
  float X0 = i - t; // Unskew the cell origin back to (x,y,z,w) space
  float Y0 = j - t;
  float Z0 = k - t;
  float W0 = l - t;

  float x0 = x - X0;  // The x,y,z,w distances from the cell origin
  float y0 = y - Y0;
  float z0 = z - Z0;
  float w0 = w - W0;

  // For the 4D case, the simplex is a 4D shape I won't even try to describe.
  // To find out which of the 24 possible simplices we're in, we need to
  // determine the magnitude ordering of x0, y0, z0 and w0.
  // The method below is a good way of finding the ordering of x,y,z,w and
  // then find the correct traversal order for the simplex weíre in.
  // First, six pair-wise comparisons are performed between each possible pair
  // of the four coordinates, and the results are used to add up binary bits
  // for an integer index.
  int c1 = (x0 > y0) ? 32 : 0;
  int c2 = (x0 > z0) ? 16 : 0;
  int c3 = (y0 > z0) ? 8 : 0;
  int c4 = (x0 > w0) ? 4 : 0;
  int c5 = (y0 > w0) ? 2 : 0;
  int c6 = (z0 > w0) ? 1 : 0;
  int c = c1 + c2 + c3 + c4 + c5 + c6;

  int i1, j1, k1, l1; // The integer offsets for the second simplex corner
  int i2, j2, k2, l2; // The integer offsets for the third simplex corner
  int i3, j3, k3, l3; // The integer offsets for the fourth simplex corner

  // simplex[c] is a 4-vector with the numbers 0, 1, 2 and 3 in some order.
  // Many values of c will never occur, since e.g. x>y>z>w makes x<z, y<w and x<w
  // impossible. Only the 24 indices which have non-zero entries make any sense.
  // We use a thresholding to set the coordinates in turn from the largest magnitude.
  // The number 3 in the "simplex" array is at the position of the largest coordinate.
  i1 = simplex[c][0]>=3 ? 1 : 0;
  j1 = simplex[c][1]>=3 ? 1 : 0;
  k1 = simplex[c][2]>=3 ? 1 : 0;
  l1 = simplex[c][3]>=3 ? 1 : 0;
  // The number 2 in the "simplex" array is at the second largest coordinate.
  i2 = simplex[c][0]>=2 ? 1 : 0;
  j2 = simplex[c][1]>=2 ? 1 : 0;
  k2 = simplex[c][2]>=2 ? 1 : 0;
  l2 = simplex[c][3]>=2 ? 1 : 0;
  // The number 1 in the "simplex" array is at the second smallest coordinate.
  i3 = simplex[c][0]>=1 ? 1 : 0;
  j3 = simplex[c][1]>=1 ? 1 : 0;
  k3 = simplex[c][2]>=1 ? 1 : 0;
  l3 = simplex[c][3]>=1 ? 1 : 0;
  // The fifth corner has all coordinate offsets = 1, so no need to look that up.

  float x1 = x0 - i1 + G4; // Offsets for second corner in (x,y,z,w) coords
  float y1 = y0 - j1 + G4;
  float z1 = z0 - k1 + G4;
  float w1 = w0 - l1 + G4;
  float x2 = x0 - i2 + 2.0f*G4; // Offsets for third corner in (x,y,z,w) coords
  float y2 = y0 - j2 + 2.0f*G4;
  float z2 = z0 - k2 + 2.0f*G4;
  float w2 = w0 - l2 + 2.0f*G4;
  float x3 = x0 - i3 + 3.0f*G4; // Offsets for fourth corner in (x,y,z,w) coords
  float y3 = y0 - j3 + 3.0f*G4;
  float z3 = z0 - k3 + 3.0f*G4;
  float w3 = w0 - l3 + 3.0f*G4;
  float x4 = x0 - 1.0f + 4.0f*G4; // Offsets for last corner in (x,y,z,w) coords
  float y4 = y0 - 1.0f + 4.0f*G4;
  float z4 = z0 - 1.0f + 4.0f*G4;
  float w4 = w0 - 1.0f + 4.0f*G4;

  // Wrap the integer indices at 256, to avoid indexing perm[] out of bounds
  int ii = i % 256;
  int jj = j % 256;
  int kk = k % 256;
  int ll = l % 256;

  // Calculate the contribution from the five corners
  float t0 = 0.6f - x0*x0 - y0*y0 - z0*z0 - w0*w0;
  if(t0 < 0.0f) n0 = 0.0f;
  else {
    t0 *= t0;
    n0 = t0 * t0 * grad(perm[ii+perm[jj+perm[kk+perm[ll]]]], x0, y0, z0, w0);
  }

  float t1 = 0.6f - x1*x1 - y1*y1 - z1*z1 - w1*w1;
  if(t1 < 0.0f) n1 = 0.0f;
  else {
    t1 *= t1;
    n1 = t1 * t1 * grad(perm[ii+i1+perm[jj+j1+perm[kk+k1+perm[ll+l1]]]], x1, y1, z1, w1);
  }

  float t2 = 0.6f - x2*x2 - y2*y2 - z2*z2 - w2*w2;
  if(t2 < 0.0f) n2 = 0.0f;
  else {
    t2 *= t2;
    n2 = t2 * t2 * grad(perm[ii+i2+perm[jj+j2+perm[kk+k2+perm[ll+l2]]]], x2, y2, z2, w2);
  }

  float t3 = 0.6f - x3*x3 - y3*y3 - z3*z3 - w3*w3;
  if(t3 < 0.0f) n3 = 0.0f;
  else {
    t3 *= t3;
    n3 = t3 * t3 * grad(perm[ii+i3+perm[jj+j3+perm[kk+k3+perm[ll+l3]]]], x3, y3, z3, w3);
  }

  float t4 = 0.6f - x4*x4 - y4*y4 - z4*z4 - w4*w4;
  if(t4 < 0.0f) n4 = 0.0f;
  else {
    t4 *= t4;
    n4 = t4 * t4 * grad(perm[ii+1+perm[jj+1+perm[kk+1+perm[ll+1]]]], x4, y4, z4, w4);
  }

  // Sum up and scale the result to cover the range [-1,1]
  return 27.0f * (n0 + n1 + n2 + n3 + n4); // TODO: The scale factor is preliminary!
}
//---------------------------------------------------------------------



float Perlin::pnoise( float x, int px )
{
  int ix0, ix1;
  float fx0, fx1;
  float s, n0, n1;

  ix0 = FASTFLOOR( x ); // Integer part of x
  fx0 = x - ix0;       // Fractional part of x
  fx1 = fx0 - 1.0f;
  ix1 = (( ix0 + 1 ) % px) & 0xff; // Wrap to 0..px-1 *and* wrap to 0..255
  ix0 = ( ix0 % px ) & 0xff;      // (because px might be greater than 256)

  s = FADE( fx0 );

  n0 = grad( perm[ ix0 ], fx0 );
  n1 = grad( perm[ ix1 ], fx1 );
  return 0.188f * ( LERP( s, n0, n1 ) );
}

// 2D float Perlin periodic noise.
float Perlin::pnoise( float x, float y, int px, int py )
{
  int ix0, iy0, ix1, iy1;
  float fx0, fy0, fx1, fy1;
  float s, t, nx0, nx1, n0, n1;

  ix0 = FASTFLOOR( x ); // Integer part of x
  iy0 = FASTFLOOR( y ); // Integer part of y
  fx0 = x - ix0;        // Fractional part of x
  fy0 = y - iy0;        // Fractional part of y
  fx1 = fx0 - 1.0f;
  fy1 = fy0 - 1.0f;
  ix1 = (( ix0 + 1 ) % px) & 0xff;  // Wrap to 0..px-1 and wrap to 0..255
  iy1 = (( iy0 + 1 ) % py) & 0xff;  // Wrap to 0..py-1 and wrap to 0..255
  ix0 = ( ix0 % px ) & 0xff;
  iy0 = ( iy0 % py ) & 0xff;

  t = FADE( fy0 );
  s = FADE( fx0 );

  nx0 = grad(perm[ix0 + perm[iy0]], fx0, fy0);
  nx1 = grad(perm[ix0 + perm[iy1]], fx0, fy1);
  n0 = LERP( t, nx0, nx1 );

  nx0 = grad(perm[ix1 + perm[iy0]], fx1, fy0);
  nx1 = grad(perm[ix1 + perm[iy1]], fx1, fy1);
  n1 = LERP(t, nx0, nx1);

  return 0.507f * ( LERP( s, n0, n1 ) );
}

// 3D float Perlin periodic noise.
float Perlin::pnoise( float x, float y, float z, int px, int py, int pz )
{
  int ix0, iy0, ix1, iy1, iz0, iz1;
  float fx0, fy0, fz0, fx1, fy1, fz1;
  float s, t, r;
  float nxy0, nxy1, nx0, nx1, n0, n1;

  ix0 = FASTFLOOR( x ); // Integer part of x
  iy0 = FASTFLOOR( y ); // Integer part of y
  iz0 = FASTFLOOR( z ); // Integer part of z
  fx0 = x - ix0;        // Fractional part of x
  fy0 = y - iy0;        // Fractional part of y
  fz0 = z - iz0;        // Fractional part of z
  fx1 = fx0 - 1.0f;
  fy1 = fy0 - 1.0f;
  fz1 = fz0 - 1.0f;
  ix1 = (( ix0 + 1 ) % px ) & 0xff; // Wrap to 0..px-1 and wrap to 0..255
  iy1 = (( iy0 + 1 ) % py ) & 0xff; // Wrap to 0..py-1 and wrap to 0..255
  iz1 = (( iz0 + 1 ) % pz ) & 0xff; // Wrap to 0..pz-1 and wrap to 0..255
  ix0 = ( ix0 % px ) & 0xff;
  iy0 = ( iy0 % py ) & 0xff;
  iz0 = ( iz0 % pz ) & 0xff;

  r = FADE( fz0 );
  t = FADE( fy0 );
  s = FADE( fx0 );

  nxy0 = grad(perm[ix0 + perm[iy0 + perm[iz0]]], fx0, fy0, fz0);
  nxy1 = grad(perm[ix0 + perm[iy0 + perm[iz1]]], fx0, fy0, fz1);
  nx0 = LERP( r, nxy0, nxy1 );

  nxy0 = grad(perm[ix0 + perm[iy1 + perm[iz0]]], fx0, fy1, fz0);
  nxy1 = grad(perm[ix0 + perm[iy1 + perm[iz1]]], fx0, fy1, fz1);
  nx1 = LERP( r, nxy0, nxy1 );

  n0 = LERP( t, nx0, nx1 );

  nxy0 = grad(perm[ix1 + perm[iy0 + perm[iz0]]], fx1, fy0, fz0);
  nxy1 = grad(perm[ix1 + perm[iy0 + perm[iz1]]], fx1, fy0, fz1);
  nx0 = LERP( r, nxy0, nxy1 );

  nxy0 = grad(perm[ix1 + perm[iy1 + perm[iz0]]], fx1, fy1, fz0);
  nxy1 = grad(perm[ix1 + perm[iy1 + perm[iz1]]], fx1, fy1, fz1);
  nx1 = LERP( r, nxy0, nxy1 );

  n1 = LERP( t, nx0, nx1 );

  return 0.936f * ( LERP( s, n0, n1 ) );
}

// 4D float Perlin periodic noise.
float Perlin::pnoise( float x, float y, float z, float w, int px, int py, int pz, int pw )
{
  int ix0, iy0, iz0, iw0, ix1, iy1, iz1, iw1;
  float fx0, fy0, fz0, fw0, fx1, fy1, fz1, fw1;
  float s, t, r, q;
  float nxyz0, nxyz1, nxy0, nxy1, nx0, nx1, n0, n1;

  ix0 = FASTFLOOR( x ); // Integer part of x
  iy0 = FASTFLOOR( y ); // Integer part of y
  iz0 = FASTFLOOR( z ); // Integer part of y
  iw0 = FASTFLOOR( w ); // Integer part of w
  fx0 = x - ix0;        // Fractional part of x
  fy0 = y - iy0;        // Fractional part of y
  fz0 = z - iz0;        // Fractional part of z
  fw0 = w - iw0;        // Fractional part of w
  fx1 = fx0 - 1.0f;
  fy1 = fy0 - 1.0f;
  fz1 = fz0 - 1.0f;
  fw1 = fw0 - 1.0f;
  ix1 = (( ix0 + 1 ) % px ) & 0xff;  // Wrap to 0..px-1 and wrap to 0..255
  iy1 = (( iy0 + 1 ) % py ) & 0xff;  // Wrap to 0..py-1 and wrap to 0..255
  iz1 = (( iz0 + 1 ) % pz ) & 0xff;  // Wrap to 0..pz-1 and wrap to 0..255
  iw1 = (( iw0 + 1 ) % pw ) & 0xff;  // Wrap to 0..pw-1 and wrap to 0..255
  ix0 = ( ix0 % px ) & 0xff;
  iy0 = ( iy0 % py ) & 0xff;
  iz0 = ( iz0 % pz ) & 0xff;
  iw0 = ( iw0 % pw ) & 0xff;

  q = FADE( fw0 );
  r = FADE( fz0 );
  t = FADE( fy0 );
  s = FADE( fx0 );

  nxyz0 = grad(perm[ix0 + perm[iy0 + perm[iz0 + perm[iw0]]]], fx0, fy0, fz0, fw0);
  nxyz1 = grad(perm[ix0 + perm[iy0 + perm[iz0 + perm[iw1]]]], fx0, fy0, fz0, fw1);
  nxy0 = LERP( q, nxyz0, nxyz1 );

  nxyz0 = grad(perm[ix0 + perm[iy0 + perm[iz1 + perm[iw0]]]], fx0, fy0, fz1, fw0);
  nxyz1 = grad(perm[ix0 + perm[iy0 + perm[iz1 + perm[iw1]]]], fx0, fy0, fz1, fw1);
  nxy1 = LERP( q, nxyz0, nxyz1 );

  nx0 = LERP ( r, nxy0, nxy1 );

  nxyz0 = grad(perm[ix0 + perm[iy1 + perm[iz0 + perm[iw0]]]], fx0, fy1, fz0, fw0);
  nxyz1 = grad(perm[ix0 + perm[iy1 + perm[iz0 + perm[iw1]]]], fx0, fy1, fz0, fw1);
  nxy0 = LERP( q, nxyz0, nxyz1 );

  nxyz0 = grad(perm[ix0 + perm[iy1 + perm[iz1 + perm[iw0]]]], fx0, fy1, fz1, fw0);
  nxyz1 = grad(perm[ix0 + perm[iy1 + perm[iz1 + perm[iw1]]]], fx0, fy1, fz1, fw1);
  nxy1 = LERP( q, nxyz0, nxyz1 );

  nx1 = LERP ( r, nxy0, nxy1 );

  n0 = LERP( t, nx0, nx1 );

  nxyz0 = grad(perm[ix1 + perm[iy0 + perm[iz0 + perm[iw0]]]], fx1, fy0, fz0, fw0);
  nxyz1 = grad(perm[ix1 + perm[iy0 + perm[iz0 + perm[iw1]]]], fx1, fy0, fz0, fw1);
  nxy0 = LERP( q, nxyz0, nxyz1 );

  nxyz0 = grad(perm[ix1 + perm[iy0 + perm[iz1 + perm[iw0]]]], fx1, fy0, fz1, fw0);
  nxyz1 = grad(perm[ix1 + perm[iy0 + perm[iz1 + perm[iw1]]]], fx1, fy0, fz1, fw1);
  nxy1 = LERP( q, nxyz0, nxyz1 );

  nx0 = LERP ( r, nxy0, nxy1 );

  nxyz0 = grad(perm[ix1 + perm[iy1 + perm[iz0 + perm[iw0]]]], fx1, fy1, fz0, fw0);
  nxyz1 = grad(perm[ix1 + perm[iy1 + perm[iz0 + perm[iw1]]]], fx1, fy1, fz0, fw1);
  nxy0 = LERP( q, nxyz0, nxyz1 );

  nxyz0 = grad(perm[ix1 + perm[iy1 + perm[iz1 + perm[iw0]]]], fx1, fy1, fz1, fw0);
  nxyz1 = grad(perm[ix1 + perm[iy1 + perm[iz1 + perm[iw1]]]], fx1, fy1, fz1, fw1);
  nxy1 = LERP( q, nxyz0, nxyz1 );

  nx1 = LERP ( r, nxy0, nxy1 );

  n1 = LERP( t, nx0, nx1 );

  return 0.87f * ( LERP( s, n0, n1 ) );
}



/** 1D simplex noise with derivative.
* If the last argument is not null, the analytic derivative
* is also calculated.
*/
float Perlin::sdnoise( float x, float *dnoise_dx)
{
  int i0 = FASTFLOOR(x);
  int i1 = i0 + 1;
  float x0 = x - i0;
  float x1 = x0 - 1.0f;

  float gx0, gx1;
  float n0, n1;
  float t1, t20, t40, t21, t41, x21;

  float x20 = x0*x0;
  float t0 = 1.0f - x20;
  //  if(t0 < 0.0f) t0 = 0.0f; // Never happens for 1D: x0<=1 always
  t20 = t0 * t0;
  t40 = t20 * t20;
  grad1(perm[i0 & 0xff], &gx0);
  n0 = t40 * gx0 * x0;

  x21 = x1*x1;
  t1 = 1.0f - x21;
  //  if(t1 < 0.0f) t1 = 0.0f; // Never happens for 1D: |x1|<=1 always
  t21 = t1 * t1;
  t41 = t21 * t21;
  grad1(perm[i1 & 0xff], &gx1);
  n1 = t41 * gx1 * x1;

  /* Compute derivative, if requested by supplying non-null pointer
  * for the last argument
  * Compute derivative according to:
  *  *dnoise_dx = -8.0f * t20 * t0 * x0 * (gx0 * x0) + t40 * gx0;
  *  *dnoise_dx += -8.0f * t21 * t1 * x1 * (gx1 * x1) + t41 * gx1;
  */

  if( (0 != dnoise_dx ))
  {
    *dnoise_dx = t20 * t0 * gx0 * x20;
    *dnoise_dx += t21 * t1 * gx1 * x21;
    *dnoise_dx *= -8.0f;
    *dnoise_dx += t40 * gx0 + t41 * gx1;
    *dnoise_dx *= 0.25f; /* Scale derivative to match the noise scaling */
  }
  // The maximum value of this noise is 8*(3/4)^4 = 2.53125
  // A factor of 0.395 would scale to fit exactly within [-1,1], but
  // to better match classic Perlin noise, we scale it down some more.
  return 0.25f * (n0 + n1);
}

/** 2D simplex noise with derivatives.
* If the last two arguments are not null, the analytic derivative
* (the 2D gradient of the scalar noise field) is also calculated.
*/
float Perlin::sdnoise( float x, float y, float *dnoise_dx, float *dnoise_dy )
{
  float n0, n1, n2; /* Noise contributions from the three simplex corners */
  float gx0, gy0, gx1, gy1, gx2, gy2; /* Gradients at simplex corners */
  float t0, t1, t2, x1, x2, y1, y2;
  float t20, t40, t21, t41, t22, t42;
  float temp0, temp1, temp2, noise;

  /* Skew the input space to determine which simplex cell we're in */
  float s = ( x + y ) * F2; /* Hairy factor for 2D */
  float xs = x + s;
  float ys = y + s;
  int ii, i = FASTFLOOR( xs );
  int jj, j = FASTFLOOR( ys );

  float t = ( float ) ( i + j ) * G2;
  float X0 = i - t; /* Unskew the cell origin back to (x,y) space */
  float Y0 = j - t;
  float x0 = x - X0; /* The x,y distances from the cell origin */
  float y0 = y - Y0;

  /* For the 2D case, the simplex shape is an equilateral triangle.
  * Determine which simplex we are in. */
  int i1, j1; /* Offsets for second (middle) corner of simplex in (i,j) coords */
  if( x0 > y0 ) { i1 = 1; j1 = 0; } /* lower triangle, XY order: (0,0)->(1,0)->(1,1) */
  else { i1 = 0; j1 = 1; }      /* upper triangle, YX order: (0,0)->(0,1)->(1,1) */

  /* A step of (1,0) in (i,j) means a step of (1-c,-c) in (x,y), and
  * a step of (0,1) in (i,j) means a step of (-c,1-c) in (x,y), where
  * c = (3-sqrt(3))/6   */
  x1 = x0 - i1 + G2; /* Offsets for middle corner in (x,y) unskewed coords */
  y1 = y0 - j1 + G2;
  x2 = x0 - 1.0f + 2.0f * G2; /* Offsets for last corner in (x,y) unskewed coords */
  y2 = y0 - 1.0f + 2.0f * G2;

  /* Wrap the integer indices at 256, to avoid indexing perm[] out of bounds */
  ii = i % 256;
  jj = j % 256;

  /* Calculate the contribution from the three corners */
  t0 = 0.5f - x0 * x0 - y0 * y0;
  if( t0 < 0.0f ) t40 = t20 = t0 = n0 = gx0 = gy0 = 0.0f; /* No influence */
  else {
    grad2( perm[ii + perm[jj]], &gx0, &gy0 );
    t20 = t0 * t0;
    t40 = t20 * t20;
    n0 = t40 * ( gx0 * x0 + gy0 * y0 );
  }

  t1 = 0.5f - x1 * x1 - y1 * y1;
  if( t1 < 0.0f ) t21 = t41 = t1 = n1 = gx1 = gy1 = 0.0f; /* No influence */
  else {
    grad2( perm[ii + i1 + perm[jj + j1]], &gx1, &gy1 );
    t21 = t1 * t1;
    t41 = t21 * t21;
    n1 = t41 * ( gx1 * x1 + gy1 * y1 );
  }

  t2 = 0.5f - x2 * x2 - y2 * y2;
  if( t2 < 0.0f ) t42 = t22 = t2 = n2 = gx2 = gy2 = 0.0f; /* No influence */
  else {
    grad2( perm[ii + 1 + perm[jj + 1]], &gx2, &gy2 );
    t22 = t2 * t2;
    t42 = t22 * t22;
    n2 = t42 * ( gx2 * x2 + gy2 * y2 );
  }

  /* Add contributions from each corner to get the final noise value.
  * The result is scaled to return values in the interval [-1,1]. */
  noise = 40.0f * ( n0 + n1 + n2 );

  /* Compute derivative, if requested by supplying non-null pointers
  * for the last two arguments */
  if( ( 0 != dnoise_dx ) && ( 0 != dnoise_dy ) )
  {
    /*  A straight, unoptimised calculation would be like:
    *    *dnoise_dx = -8.0f * t20 * t0 * x0 * ( gx0 * x0 + gy0 * y0 ) + t40 * gx0;
    *    *dnoise_dy = -8.0f * t20 * t0 * y0 * ( gx0 * x0 + gy0 * y0 ) + t40 * gy0;
    *    *dnoise_dx += -8.0f * t21 * t1 * x1 * ( gx1 * x1 + gy1 * y1 ) + t41 * gx1;
    *    *dnoise_dy += -8.0f * t21 * t1 * y1 * ( gx1 * x1 + gy1 * y1 ) + t41 * gy1;
    *    *dnoise_dx += -8.0f * t22 * t2 * x2 * ( gx2 * x2 + gy2 * y2 ) + t42 * gx2;
    *    *dnoise_dy += -8.0f * t22 * t2 * y2 * ( gx2 * x2 + gy2 * y2 ) + t42 * gy2;
    */
    temp0 = t20 * t0 * ( gx0* x0 + gy0 * y0 );
    *dnoise_dx = temp0 * x0;
    *dnoise_dy = temp0 * y0;
    temp1 = t21 * t1 * ( gx1 * x1 + gy1 * y1 );
    *dnoise_dx += temp1 * x1;
    *dnoise_dy += temp1 * y1;
    temp2 = t22 * t2 * ( gx2* x2 + gy2 * y2 );
    *dnoise_dx += temp2 * x2;
    *dnoise_dy += temp2 * y2;
    *dnoise_dx *= -8.0f;
    *dnoise_dy *= -8.0f;
    *dnoise_dx += t40 * gx0 + t41 * gx1 + t42 * gx2;
    *dnoise_dy += t40 * gy0 + t41 * gy1 + t42 * gy2;
    *dnoise_dx *= 40.0f; /* Scale derivative to match the noise scaling */
    *dnoise_dy *= 40.0f;
  }
  return noise;
}



/** 3D simplex noise with derivatives.
* If the last tthree arguments are not null, the analytic derivative
* (the 3D gradient of the scalar noise field) is also calculated.
*/
float Perlin::sdnoise( float x, float y, float z,
               float *dnoise_dx, float *dnoise_dy, float *dnoise_dz )
{
  float n0, n1, n2, n3; /* Noise contributions from the four simplex corners */
  float noise;          /* Return value */
  float gx0, gy0, gz0, gx1, gy1, gz1; /* Gradients at simplex corners */
  float gx2, gy2, gz2, gx3, gy3, gz3;
  float x1, y1, z1, x2, y2, z2, x3, y3, z3;
  float t0, t1, t2, t3, t20, t40, t21, t41, t22, t42, t23, t43;
  float temp0, temp1, temp2, temp3;

  /* Skew the input space to determine which simplex cell we're in */
  float s = (x+y+z)*F3; /* Very nice and simple skew factor for 3D */
  float xs = x+s;
  float ys = y+s;
  float zs = z+s;
  int ii, i = FASTFLOOR(xs);
  int jj, j = FASTFLOOR(ys);
  int kk, k = FASTFLOOR(zs);

  float t = (float)(i+j+k)*G3; 
  float X0 = i-t; /* Unskew the cell origin back to (x,y,z) space */
  float Y0 = j-t;
  float Z0 = k-t;
  float x0 = x-X0; /* The x,y,z distances from the cell origin */
  float y0 = y-Y0;
  float z0 = z-Z0;

  /* For the 3D case, the simplex shape is a slightly irregular tetrahedron.
  * Determine which simplex we are in. */
  int i1, j1, k1; /* Offsets for second corner of simplex in (i,j,k) coords */
  int i2, j2, k2; /* Offsets for third corner of simplex in (i,j,k) coords */

  /* TODO: This code would benefit from a backport from the GLSL version! */
  if(x0>=y0) {
    if(y0>=z0)
    { i1=1; j1=0; k1=0; i2=1; j2=1; k2=0; } /* X Y Z order */
    else if(x0>=z0) { i1=1; j1=0; k1=0; i2=1; j2=0; k2=1; } /* X Z Y order */
    else { i1=0; j1=0; k1=1; i2=1; j2=0; k2=1; } /* Z X Y order */
  }
  else { // x0<y0
    if(y0<z0) { i1=0; j1=0; k1=1; i2=0; j2=1; k2=1; } /* Z Y X order */
    else if(x0<z0) { i1=0; j1=1; k1=0; i2=0; j2=1; k2=1; } /* Y Z X order */
    else { i1=0; j1=1; k1=0; i2=1; j2=1; k2=0; } /* Y X Z order */
  }

  /* A step of (1,0,0) in (i,j,k) means a step of (1-c,-c,-c) in (x,y,z),
  * a step of (0,1,0) in (i,j,k) means a step of (-c,1-c,-c) in (x,y,z), and
  * a step of (0,0,1) in (i,j,k) means a step of (-c,-c,1-c) in (x,y,z), where
  * c = 1/6.   */

  x1 = x0 - i1 + G3; /* Offsets for second corner in (x,y,z) coords */
  y1 = y0 - j1 + G3;
  z1 = z0 - k1 + G3;
  x2 = x0 - i2 + 2.0f * G3; /* Offsets for third corner in (x,y,z) coords */
  y2 = y0 - j2 + 2.0f * G3;
  z2 = z0 - k2 + 2.0f * G3;
  x3 = x0 - 1.0f + 3.0f * G3; /* Offsets for last corner in (x,y,z) coords */
  y3 = y0 - 1.0f + 3.0f * G3;
  z3 = z0 - 1.0f + 3.0f * G3;

  /* Wrap the integer indices at 256, to avoid indexing perm[] out of bounds */
  ii = i % 256;
  jj = j % 256;
  kk = k % 256;

  /* Calculate the contribution from the four corners */
  t0 = 0.6f - x0*x0 - y0*y0 - z0*z0;
  if(t0 < 0.0f) n0 = t0 = t20 = t40 = gx0 = gy0 = gz0 = 0.0f;
  else {
    grad3( perm[ii + perm[jj + perm[kk]]], &gx0, &gy0, &gz0 );
    t20 = t0 * t0;
    t40 = t20 * t20;
    n0 = t40 * ( gx0 * x0 + gy0 * y0 + gz0 * z0 );
  }

  t1 = 0.6f - x1*x1 - y1*y1 - z1*z1;
  if(t1 < 0.0f) n1 = t1 = t21 = t41 = gx1 = gy1 = gz1 = 0.0f;
  else {
    grad3( perm[ii + i1 + perm[jj + j1 + perm[kk + k1]]], &gx1, &gy1, &gz1 );
    t21 = t1 * t1;
    t41 = t21 * t21;
    n1 = t41 * ( gx1 * x1 + gy1 * y1 + gz1 * z1 );
  }

  t2 = 0.6f - x2*x2 - y2*y2 - z2*z2;
  if(t2 < 0.0f) n2 = t2 = t22 = t42 = gx2 = gy2 = gz2 = 0.0f;
  else {
    grad3( perm[ii + i2 + perm[jj + j2 + perm[kk + k2]]], &gx2, &gy2, &gz2 );
    t22 = t2 * t2;
    t42 = t22 * t22;
    n2 = t42 * ( gx2 * x2 + gy2 * y2 + gz2 * z2 );
  }

  t3 = 0.6f - x3*x3 - y3*y3 - z3*z3;
  if(t3 < 0.0f) n3 = t3 = t23 = t43 = gx3 = gy3 = gz3 = 0.0f;
  else {
    grad3( perm[ii + 1 + perm[jj + 1 + perm[kk + 1]]], &gx3, &gy3, &gz3 );
    t23 = t3 * t3;
    t43 = t23 * t23;
    n3 = t43 * ( gx3 * x3 + gy3 * y3 + gz3 * z3 );
  }

  /*  Add contributions from each corner to get the final noise value.
  * The result is scaled to return values in the range [-1,1] */
  noise = 28.0f * (n0 + n1 + n2 + n3);

  /* Compute derivative, if requested by supplying non-null pointers
  * for the last three arguments */
  if( ( 0 != dnoise_dx ) && ( 0 != dnoise_dy ) && ( 0 != dnoise_dz ))
  {
    /*  A straight, unoptimised calculation would be like:
    *     *dnoise_dx = -8.0f * t20 * t0 * x0 * dot(gx0, gy0, gz0, x0, y0, z0) + t40 * gx0;
    *    *dnoise_dy = -8.0f * t20 * t0 * y0 * dot(gx0, gy0, gz0, x0, y0, z0) + t40 * gy0;
    *    *dnoise_dz = -8.0f * t20 * t0 * z0 * dot(gx0, gy0, gz0, x0, y0, z0) + t40 * gz0;
    *    *dnoise_dx += -8.0f * t21 * t1 * x1 * dot(gx1, gy1, gz1, x1, y1, z1) + t41 * gx1;
    *    *dnoise_dy += -8.0f * t21 * t1 * y1 * dot(gx1, gy1, gz1, x1, y1, z1) + t41 * gy1;
    *    *dnoise_dz += -8.0f * t21 * t1 * z1 * dot(gx1, gy1, gz1, x1, y1, z1) + t41 * gz1;
    *    *dnoise_dx += -8.0f * t22 * t2 * x2 * dot(gx2, gy2, gz2, x2, y2, z2) + t42 * gx2;
    *    *dnoise_dy += -8.0f * t22 * t2 * y2 * dot(gx2, gy2, gz2, x2, y2, z2) + t42 * gy2;
    *    *dnoise_dz += -8.0f * t22 * t2 * z2 * dot(gx2, gy2, gz2, x2, y2, z2) + t42 * gz2;
    *    *dnoise_dx += -8.0f * t23 * t3 * x3 * dot(gx3, gy3, gz3, x3, y3, z3) + t43 * gx3;
    *    *dnoise_dy += -8.0f * t23 * t3 * y3 * dot(gx3, gy3, gz3, x3, y3, z3) + t43 * gy3;
    *    *dnoise_dz += -8.0f * t23 * t3 * z3 * dot(gx3, gy3, gz3, x3, y3, z3) + t43 * gz3;
    */
    temp0 = t20 * t0 * ( gx0 * x0 + gy0 * y0 + gz0 * z0 );
    *dnoise_dx = temp0 * x0;
    *dnoise_dy = temp0 * y0;
    *dnoise_dz = temp0 * z0;
    temp1 = t21 * t1 * ( gx1 * x1 + gy1 * y1 + gz1 * z1 );
    *dnoise_dx += temp1 * x1;
    *dnoise_dy += temp1 * y1;
    *dnoise_dz += temp1 * z1;
    temp2 = t22 * t2 * ( gx2 * x2 + gy2 * y2 + gz2 * z2 );
    *dnoise_dx += temp2 * x2;
    *dnoise_dy += temp2 * y2;
    *dnoise_dz += temp2 * z2;
    temp3 = t23 * t3 * ( gx3 * x3 + gy3 * y3 + gz3 * z3 );
    *dnoise_dx += temp3 * x3;
    *dnoise_dy += temp3 * y3;
    *dnoise_dz += temp3 * z3;
    *dnoise_dx *= -8.0f;
    *dnoise_dy *= -8.0f;
    *dnoise_dz *= -8.0f;
    *dnoise_dx += t40 * gx0 + t41 * gx1 + t42 * gx2 + t43 * gx3;
    *dnoise_dy += t40 * gy0 + t41 * gy1 + t42 * gy2 + t43 * gy3;
    *dnoise_dz += t40 * gz0 + t41 * gz1 + t42 * gz2 + t43 * gz3;
    *dnoise_dx *= 28.0f; /* Scale derivative to match the noise scaling */
    *dnoise_dy *= 28.0f;
    *dnoise_dz *= 28.0f;
  }
  return noise;
}

/** 4D simplex noise with derivatives.
* If the last four arguments are not null, the analytic derivative
* (the 4D gradient of the scalar noise field) is also calculated.
*/
float Perlin::sdnoise( float x, float y, float z, float w,
               float *dnoise_dx, float *dnoise_dy,
               float *dnoise_dz, float *dnoise_dw)
{
  float n0, n1, n2, n3, n4; // Noise contributions from the five corners
  float noise; // Return value
  float gx0, gy0, gz0, gw0, gx1, gy1, gz1, gw1; /* Gradients at simplex corners */
  float gx2, gy2, gz2, gw2, gx3, gy3, gz3, gw3, gx4, gy4, gz4, gw4;
  float t20, t21, t22, t23, t24;
  float t40, t41, t42, t43, t44;
  float x1, y1, z1, w1, x2, y2, z2, w2, x3, y3, z3, w3, x4, y4, z4, w4;
  float t0, t1, t2, t3, t4;
  float temp0, temp1, temp2, temp3, temp4;

  // Skew the (x,y,z,w) space to determine which cell of 24 simplices we're in
  float s = (x + y + z + w) * F4; // Factor for 4D skewing
  float xs = x + s;
  float ys = y + s;
  float zs = z + s;
  float ws = w + s;
  int ii, i = FASTFLOOR(xs);
  int jj, j = FASTFLOOR(ys);
  int kk, k = FASTFLOOR(zs);
  int ll, l = FASTFLOOR(ws);

  float t = (i + j + k + l) * G4; // Factor for 4D unskewing
  float X0 = i - t; // Unskew the cell origin back to (x,y,z,w) space
  float Y0 = j - t;
  float Z0 = k - t;
  float W0 = l - t;

  float x0 = x - X0;  // The x,y,z,w distances from the cell origin
  float y0 = y - Y0;
  float z0 = z - Z0;
  float w0 = w - W0;

  // For the 4D case, the simplex is a 4D shape I won't even try to describe.
  // To find out which of the 24 possible simplices we're in, we need to
  // determine the magnitude ordering of x0, y0, z0 and w0.
  // The method below is a reasonable way of finding the ordering of x,y,z,w
  // and then find the correct traversal order for the simplex we’re in.
  // First, six pair-wise comparisons are performed between each possible pair
  // of the four coordinates, and then the results are used to add up binary
  // bits for an integer index into a precomputed lookup table, simplex[].
  int c1 = (x0 > y0) ? 32 : 0;
  int c2 = (x0 > z0) ? 16 : 0;
  int c3 = (y0 > z0) ? 8 : 0;
  int c4 = (x0 > w0) ? 4 : 0;
  int c5 = (y0 > w0) ? 2 : 0;
  int c6 = (z0 > w0) ? 1 : 0;
  int c = c1 | c2 | c3 | c4 | c5 | c6; // '|' is mostly faster than '+'

  int i1, j1, k1, l1; // The integer offsets for the second simplex corner
  int i2, j2, k2, l2; // The integer offsets for the third simplex corner
  int i3, j3, k3, l3; // The integer offsets for the fourth simplex corner

  // simplex[c] is a 4-vector with the numbers 0, 1, 2 and 3 in some order.
  // Many values of c will never occur, since e.g. x>y>z>w makes x<z, y<w and x<w
  // impossible. Only the 24 indices which have non-zero entries make any sense.
  // We use a thresholding to set the coordinates in turn from the largest magnitude.
  // The number 3 in the "simplex" array is at the position of the largest coordinate.
  i1 = simplex[c][0]>=3 ? 1 : 0;
  j1 = simplex[c][1]>=3 ? 1 : 0;
  k1 = simplex[c][2]>=3 ? 1 : 0;
  l1 = simplex[c][3]>=3 ? 1 : 0;
  // The number 2 in the "simplex" array is at the second largest coordinate.
  i2 = simplex[c][0]>=2 ? 1 : 0;
  j2 = simplex[c][1]>=2 ? 1 : 0;
  k2 = simplex[c][2]>=2 ? 1 : 0;
  l2 = simplex[c][3]>=2 ? 1 : 0;
  // The number 1 in the "simplex" array is at the second smallest coordinate.
  i3 = simplex[c][0]>=1 ? 1 : 0;
  j3 = simplex[c][1]>=1 ? 1 : 0;
  k3 = simplex[c][2]>=1 ? 1 : 0;
  l3 = simplex[c][3]>=1 ? 1 : 0;
  // The fifth corner has all coordinate offsets = 1, so no need to look that up.

  x1 = x0 - i1 + G4; // Offsets for second corner in (x,y,z,w) coords
  y1 = y0 - j1 + G4;
  z1 = z0 - k1 + G4;
  w1 = w0 - l1 + G4;
  x2 = x0 - i2 + 2.0f * G4; // Offsets for third corner in (x,y,z,w) coords
  y2 = y0 - j2 + 2.0f * G4;
  z2 = z0 - k2 + 2.0f * G4;
  w2 = w0 - l2 + 2.0f * G4;
  x3 = x0 - i3 + 3.0f * G4; // Offsets for fourth corner in (x,y,z,w) coords
  y3 = y0 - j3 + 3.0f * G4;
  z3 = z0 - k3 + 3.0f * G4;
  w3 = w0 - l3 + 3.0f * G4;
  x4 = x0 - 1.0f + 4.0f * G4; // Offsets for last corner in (x,y,z,w) coords
  y4 = y0 - 1.0f + 4.0f * G4;
  z4 = z0 - 1.0f + 4.0f * G4;
  w4 = w0 - 1.0f + 4.0f * G4;

  // Wrap the integer indices at 256, to avoid indexing perm[] out of bounds
  ii = i & 0xff;
  jj = j & 0xff;
  kk = k & 0xff;
  ll = l & 0xff;

  // Calculate the contribution from the five corners
  t0 = 0.6f - x0*x0 - y0*y0 - z0*z0 - w0*w0;
  if(t0 < 0.0f) n0 = t0 = t20 = t40 = gx0 = gy0 = gz0 = gw0 = 0.0f;
  else {
    t20 = t0 * t0;
    t40 = t20 * t20;
    grad4(perm[ii+perm[jj+perm[kk+perm[ll]]]], &gx0, &gy0, &gz0, &gw0);
    n0 = t40 * ( gx0 * x0 + gy0 * y0 + gz0 * z0 + gw0 * w0 );
  }

  t1 = 0.6f - x1*x1 - y1*y1 - z1*z1 - w1*w1;
  if(t1 < 0.0f) n1 = t1 = t21 = t41 = gx1 = gy1 = gz1 = gw1 = 0.0f;
  else {
    t21 = t1 * t1;
    t41 = t21 * t21;
    grad4(perm[ii+i1+perm[jj+j1+perm[kk+k1+perm[ll+l1]]]], &gx1, &gy1, &gz1, &gw1);
    n1 = t41 * ( gx1 * x1 + gy1 * y1 + gz1 * z1 + gw1 * w1 );
  }

  t2 = 0.6f - x2*x2 - y2*y2 - z2*z2 - w2*w2;
  if(t2 < 0.0f) n2 = t2 = t22 = t42 = gx2 = gy2 = gz2 = gw2 = 0.0f;
  else {
    t22 = t2 * t2;
    t42 = t22 * t22;
    grad4(perm[ii+i2+perm[jj+j2+perm[kk+k2+perm[ll+l2]]]], &gx2, &gy2, &gz2, &gw2);
    n2 = t42 * ( gx2 * x2 + gy2 * y2 + gz2 * z2 + gw2 * w2 );
  }

  t3 = 0.6f - x3*x3 - y3*y3 - z3*z3 - w3*w3;
  if(t3 < 0.0f) n3 = t3 = t23 = t43 = gx3 = gy3 = gz3 = gw3 = 0.0f;
  else {
    t23 = t3 * t3;
    t43 = t23 * t23;
    grad4(perm[ii+i3+perm[jj+j3+perm[kk+k3+perm[ll+l3]]]], &gx3, &gy3, &gz3, &gw3);
    n3 = t43 * ( gx3 * x3 + gy3 * y3 + gz3 * z3 + gw3 * w3 );
  }

  t4 = 0.6f - x4*x4 - y4*y4 - z4*z4 - w4*w4;
  if(t4 < 0.0f) n4 = t4 = t24 = t44 = gx4 = gy4 = gz4 = gw4 = 0.0f;
  else {
    t24 = t4 * t4;
    t44 = t24 * t24;
    grad4(perm[ii+1+perm[jj+1+perm[kk+1+perm[ll+1]]]], &gx4, &gy4, &gz4, &gw4);
    n4 = t44 * ( gx4 * x4 + gy4 * y4 + gz4 * z4 + gw4 * w4 );
  }

  // Sum up and scale the result to cover the range [-1,1]
  noise = 27.0f * (n0 + n1 + n2 + n3 + n4); // TODO: The scale factor is preliminary!

  /* Compute derivative, if requested by supplying non-null pointers
  * for the last four arguments */
  if( ( 0 != dnoise_dx ) && ( 0 != dnoise_dy ) && ( 0 != dnoise_dz ) && ( 0 != dnoise_dw ) )
  {
    /*  A straight, unoptimised calculation would be like:
    *     *dnoise_dx = -8.0f * t20 * t0 * x0 * dot(gx0, gy0, gz0, gw0, x0, y0, z0, w0) + t40 * gx0;
    *    *dnoise_dy = -8.0f * t20 * t0 * y0 * dot(gx0, gy0, gz0, gw0, x0, y0, z0, w0) + t40 * gy0;
    *    *dnoise_dz = -8.0f * t20 * t0 * z0 * dot(gx0, gy0, gz0, gw0, x0, y0, z0, w0) + t40 * gz0;
    *    *dnoise_dw = -8.0f * t20 * t0 * w0 * dot(gx0, gy0, gz0, gw0, x0, y0, z0, w0) + t40 * gw0;
    *    *dnoise_dx += -8.0f * t21 * t1 * x1 * dot(gx1, gy1, gz1, gw1, x1, y1, z1, w1) + t41 * gx1;
    *    *dnoise_dy += -8.0f * t21 * t1 * y1 * dot(gx1, gy1, gz1, gw1, x1, y1, z1, w1) + t41 * gy1;
    *    *dnoise_dz += -8.0f * t21 * t1 * z1 * dot(gx1, gy1, gz1, gw1, x1, y1, z1, w1) + t41 * gz1;
    *    *dnoise_dw = -8.0f * t21 * t1 * w1 * dot(gx1, gy1, gz1, gw1, x1, y1, z1, w1) + t41 * gw1;
    *    *dnoise_dx += -8.0f * t22 * t2 * x2 * dot(gx2, gy2, gz2, gw2, x2, y2, z2, w2) + t42 * gx2;
    *    *dnoise_dy += -8.0f * t22 * t2 * y2 * dot(gx2, gy2, gz2, gw2, x2, y2, z2, w2) + t42 * gy2;
    *    *dnoise_dz += -8.0f * t22 * t2 * z2 * dot(gx2, gy2, gz2, gw2, x2, y2, z2, w2) + t42 * gz2;
    *    *dnoise_dw += -8.0f * t22 * t2 * w2 * dot(gx2, gy2, gz2, gw2, x2, y2, z2, w2) + t42 * gw2;
    *    *dnoise_dx += -8.0f * t23 * t3 * x3 * dot(gx3, gy3, gz3, gw3, x3, y3, z3, w3) + t43 * gx3;
    *    *dnoise_dy += -8.0f * t23 * t3 * y3 * dot(gx3, gy3, gz3, gw3, x3, y3, z3, w3) + t43 * gy3;
    *    *dnoise_dz += -8.0f * t23 * t3 * z3 * dot(gx3, gy3, gz3, gw3, x3, y3, z3, w3) + t43 * gz3;
    *    *dnoise_dw += -8.0f * t23 * t3 * w3 * dot(gx3, gy3, gz3, gw3, x3, y3, z3, w3) + t43 * gw3;
    *    *dnoise_dx += -8.0f * t24 * t4 * x4 * dot(gx4, gy4, gz4, gw4, x4, y4, z4, w4) + t44 * gx4;
    *    *dnoise_dy += -8.0f * t24 * t4 * y4 * dot(gx4, gy4, gz4, gw4, x4, y4, z4, w4) + t44 * gy4;
    *    *dnoise_dz += -8.0f * t24 * t4 * z4 * dot(gx4, gy4, gz4, gw4, x4, y4, z4, w4) + t44 * gz4;
    *    *dnoise_dw += -8.0f * t24 * t4 * w4 * dot(gx4, gy4, gz4, gw4, x4, y4, z4, w4) + t44 * gw4;
    */
    temp0 = t20 * t0 * ( gx0 * x0 + gy0 * y0 + gz0 * z0 + gw0 * w0 );
    *dnoise_dx = temp0 * x0;
    *dnoise_dy = temp0 * y0;
    *dnoise_dz = temp0 * z0;
    *dnoise_dw = temp0 * w0;
    temp1 = t21 * t1 * ( gx1 * x1 + gy1 * y1 + gz1 * z1 + gw1 * w1 );
    *dnoise_dx += temp1 * x1;
    *dnoise_dy += temp1 * y1;
    *dnoise_dz += temp1 * z1;
    *dnoise_dw += temp1 * w1;
    temp2 = t22 * t2 * ( gx2 * x2 + gy2 * y2 + gz2 * z2 + gw2 * w2 );
    *dnoise_dx += temp2 * x2;
    *dnoise_dy += temp2 * y2;
    *dnoise_dz += temp2 * z2;
    *dnoise_dw += temp2 * w2;
    temp3 = t23 * t3 * ( gx3 * x3 + gy3 * y3 + gz3 * z3 + gw3 * w3 );
    *dnoise_dx += temp3 * x3;
    *dnoise_dy += temp3 * y3;
    *dnoise_dz += temp3 * z3;
    *dnoise_dw += temp3 * w3;
    temp4 = t24 * t4 * ( gx4 * x4 + gy4 * y4 + gz4 * z4 + gw4 * w4 );
    *dnoise_dx += temp4 * x4;
    *dnoise_dy += temp4 * y4;
    *dnoise_dz += temp4 * z4;
    *dnoise_dw += temp4 * w4;
    *dnoise_dx *= -8.0f;
    *dnoise_dy *= -8.0f;
    *dnoise_dz *= -8.0f;
    *dnoise_dw *= -8.0f;
    *dnoise_dx += t40 * gx0 + t41 * gx1 + t42 * gx2 + t43 * gx3 + t44 * gx4;
    *dnoise_dy += t40 * gy0 + t41 * gy1 + t42 * gy2 + t43 * gy3 + t44 * gy4;
    *dnoise_dz += t40 * gz0 + t41 * gz1 + t42 * gz2 + t43 * gz3 + t44 * gz4;
    *dnoise_dw += t40 * gw0 + t41 * gw1 + t42 * gw2 + t43 * gw3 + t44 * gw4;

    *dnoise_dx *= 28.0f; /* Scale derivative to match the noise scaling */
    *dnoise_dy *= 28.0f;
    *dnoise_dz *= 28.0f;
    *dnoise_dw *= 28.0f;
  }

  return noise;
}




float Perlin::hnoise1( float x, int px, float octaveScaleFactor, float freqMult, int numFreqs )
{
  float sum = 0.f;
  float scale = 1.f;
  for( int i = 0 ; i < numFreqs ; i++ )
  {
    sum += pnoise(x,px) * scale; // get noise, reduce magnitude by scale (so "higher freq" noise has less effect)
    scale *= octaveScaleFactor ; // increase division factor, for next iteration
    x *= freqMult ; // "up the frequency" of next noise value.
  }
  return sum ;
}

// To wrap on a sphere:  use theta and phi for x and y
// Brushed metal: vary only ONE of x,y
//   (eg PerlinNoise2D( uv.x, _1_, 2, 2, 8 ) // (the 1 stays constant across the 2d face)
//   (OR you could just generate a line of 1d noise and copy it)
float Perlin::hnoise2( float x, float y, int px, int py, float octaveScaleFactor, float freqMult, int numFreqs )
{
  float sum = 0.f;
  float p[2],scale = 1.f;

  p[0] = x;
  p[1] = y;
  for( int i=0 ; i < numFreqs ; i++ )
  {
    sum += pnoise(p[0],p[1],px,py) * scale;
    scale *= octaveScaleFactor ;
    p[0] *= freqMult ;
    p[1] *= freqMult ;
  }
  return sum ;
}

float Perlin::hnoise3(float x, float y, float z, int px, int py, int pz, float octaveScaleFactor, float freqMult, int numFreqs )
{
  float sum = 0.f;
  float p[3],scale = 1.f;

  p[0] = x;
  p[1] = y;
  p[2] = z;
  for( int i = 0 ; i < numFreqs ; i++ )
  {
    sum += pnoise(p[0],p[1],p[2],px,py,pz) * scale;
    scale *= octaveScaleFactor ;
    p[0] *= freqMult ;
    p[1] *= freqMult ;
    p[2] *= freqMult ;
  }
  return sum ;
}


