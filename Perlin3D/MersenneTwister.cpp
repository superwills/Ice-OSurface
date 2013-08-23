#include "MersenneTwister.h"

namespace MersenneTwister
{
  /* initializes mt[N] with a seed */
  void init_genrand(unsigned int s)
  {
    mt[0]= s & 0xffffffffU;
    for (mti=1; mti<N; mti++) {
        mt[mti] = 
	    (1812433253U * (mt[mti-1] ^ (mt[mti-1] >> 30)) + mti); 
        // See Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier.
        // In the previous versions, MSBs of the seed affect 
        // only MSBs of the array mt[].
        // 2002/01/09 modified by Makoto Matsumoto
        mt[mti] &= 0xffffffffU;
        // for >32 bit machines
    }
  }

  /* initialize by an array with array-length */
  /* init_key is the array for initializing keys */
  /* key_length is its length */
  /* slight change for C++, 2004/2/26 */
  void init_by_array(unsigned int init_key[], int key_length)
  {
    int i, j, k;
    init_genrand(19650218U);
    i=1; j=0;
    k = (N>key_length ? N : key_length);
    for (; k; k--) {
        mt[i] = (mt[i] ^ ((mt[i-1] ^ (mt[i-1] >> 30)) * 1664525U))
          + init_key[j] + j; /* non linear */
        mt[i] &= 0xffffffffU; /* for WORDSIZE > 32 machines */
        i++; j++;
        if (i>=N) { mt[0] = mt[N-1]; i=1; }
        if (j>=key_length) j=0;
    }
    for (k=N-1; k; k--) {
        mt[i] = (mt[i] ^ ((mt[i-1] ^ (mt[i-1] >> 30)) * 1566083941U))
          - i; // non linear
        mt[i] &= 0xffffffffU; // for WORDSIZE > 32 machines * this is unnecessary if you just use `int` instead of long.
        i++;
        if (i>=N) { mt[0] = mt[N-1]; i=1; }
    }

    mt[0] = 0x80000000U; // MSB is 1; assuring non-zero initial array
  }

  // generates a random number on [0,0xffffffff]-interval
  unsigned int genrand_int32()
  {
    unsigned int y;
    static unsigned int mag01[2]={0x0UL, MATRIX_A};
    // mag01[x] = x * MATRIX_A  for x=0,1

    if (mti >= N) { // generate N words at one time
      int kk;

      if (mti == N+1)   // if init_genrand() has not been called,
        init_genrand(5489U); // a default initial seed is used

      for (kk=0;kk<N-M;kk++) {//227 iterations
        y = (mt[kk]&UPPER_MASK)|(mt[kk+1]&LOWER_MASK);
        mt[kk] = mt[kk+M] ^ (y >> 1) ^ mag01[y & 0x1U];
      }
      for (;kk<N-1;kk++) {//goes the 397 remaining iterations (624 iterations total)
        y = (mt[kk]&UPPER_MASK)|(mt[kk+1]&LOWER_MASK);
        mt[kk] = mt[kk+(M-N)] ^ (y >> 1) ^ mag01[y & 0x1U];
      }
      y = (mt[N-1]&UPPER_MASK)|(mt[0]&LOWER_MASK);
      mt[N-1] = mt[M-1] ^ (y >> 1) ^ mag01[y & 0x1U];

      mti = 0;
    }

    y = mt[mti++];

    // Tempering
    y ^= (y >> 11);
    y ^= (y << 7) & 0x9d2c5680U;
    y ^= (y << 15) & 0xefc60000U;
    y ^= (y >> 18);

    return y;
  }

  // generates a random number on [0,1)-real-interval 
  double genrand_real()
  {
    return genrand_int32()*(1.0/4294967295.0); 
    // divided by 0xffffffff ( 2^32 - 1 )
  }

  // generates a random number on [0,1) with 53-bit resolution
  double genrand_res53() 
  { 
    unsigned long a=genrand_int32()>>5, b=genrand_int32()>>6; 
    return(a*67108864.0+b)*(1.0/9007199254740992.0); 
  } 
  // These real versions are due to Isaku Wada, 2002/01/09 added

  void initMersenneTwister()
  {
    unsigned int init[4]={0x123, 0x234, 0x345, 0x456} ;
    int length=4;
    init_by_array(init, length);
  }
} ;

