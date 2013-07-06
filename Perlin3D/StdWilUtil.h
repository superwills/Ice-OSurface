#include <stdio.h>
#include <stdlib.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <stdarg.h>
#include <time.h>
#include <list>
#include <vector>
#include <map>
#include <string>
using namespace std ;

#include "MersenneTwister.h"

#ifndef STDWILUTIL_H
#define STDWILUTIL_H

#define DEGTORADCONST 0.017453292519943295769236907684886
#define RADTODEGCONST 57.295779513082320876798154814105
#define RADIANS(degreeMeasure) (degreeMeasure*DEGTORADCONST)
#define DEGREES(radianMeasure) (radianMeasure*RADTODEGCONST)
#define FREE(str) if(str){free(str);str=0;}
#define DESTROY(OBJ) if(OBJ){delete (OBJ); (OBJ)=0;}
#define DESTROY_ARRAY(ARR) if(ARR){delete[] (ARR); (ARR)=0;}
#define OVERWRITE(OBJ,WITHME) {if(OBJ) delete(OBJ); (OBJ)=WITHME;}

#define INITARR2(arr,a,b) (arr)[0]=a,(arr)[1]=b
#define INITARR3(arr,a,b,c) (arr)[0]=a,(arr)[1]=b,(arr)[2]=c
#define INITARR4(arr,a,b,c,d) (arr)[0]=a,(arr)[1]=b,(arr)[2]=c,(arr)[3]=d
#define INITARR5(arr,a,b,c,d,e) (arr)[0]=a,(arr)[1]=b,(arr)[2]=c,(arr)[3]=d,(arr)[4]=e

#define every(intVal,cycle) (intVal%cycle==0)

#define YesOrNo(x) (x?"Yes":"No")

// Gives the biggest of x,y, or z
#define max3(x,y,z) max(max(x,y),z)
#define min3(x,y,z) min(min(x,y),z)

// if you have axis 0, these two defines give you 1 & 2, for example
#define OTHERAXIS1(axis) ((axis+1)%3)
#define OTHERAXIS2(axis) ((axis+2)%3)

// code that is common to marching cubes/tets
// For naming the axis edge a vertex is on
//              0   1   2   3   4   5
enum AxisEdge{ PX, NX, PY, NY, PZ, NZ } ;
// EVEN AXES have (AxisEdge%2==0).
// YOUR AXIS INDEX is AxisEdge/2 (PX,NX=>0, PY,NY=>1, PZ,NZ=>2)

/// 7 .9's (0.9999999) is the FURTHEST something can be before disappearing
// This is because 0.99999999 is 1.0 when converted to double. We use 6 9's just in case.
const static float Z_FAR_AWAY = 0.9999f ;
const static float Z_IN_FRONT = -0.9999 ;
const static float Z_FARTHEST_AWAY = 0.999999f ;
const static float Z_IN_YOUR_FACE = -0.999999f ;
// 1 is so far away it can't be seen, 0.99999 is the furthest wawy
// something can be and still be seen
// -0.99999 is the closest.
const static float Z_HIDDEN = 100.0f ; // 

// To make something appear ONE_LEVEL below IN_FRONT,
// use IN_FRONT + ONE_LEVEL_DOWN.
const static float Z_ONE_LEVEL_DOWN = 0.0001f ;
const static float Z_ONE_LEVEL_UP = -0.0001f ;

// skip is a synonym for "skipping" a loop iteration
#define skip continue

// bail is for bailing out of a function
#define bail return

// "failure" lol
#define bailure(x) error(x); return



const static float SQRT2F = sqrtf( 2.0f ) ;
//#define RADIANS(degrees) (degrees*0.01745329251)
#define EPS_MIN 1e-6f

inline bool IsPow2( int x )
{
  // 110001101 && (
  //   110001101 &
  //   110001100 ==
  //   110001100  // FALSE
  //   000000000
  // )
  
  // Expected:
  // 100000 && (
  //   100000 &
  //   011111 ==
  //   000000 TRUE
  // )
  
  // the x && in front ensure 0 isn't considered a power of 2
  return x && ((x & (x-1))==0) ;
}

// The 'i' is for int, there is a log2 for double in stdclib
inline unsigned int log2i( unsigned int x )
{
  unsigned int log2Val = 0 ;
  // Count push off bits to right until 0
  // 101 => 10 => 1 => 0
  // which means hibit was 3rd bit, its value is 2^3
  while( x>>=1 ) log2Val++;  // div by 2 until find log2.  log_2(63)=5.97, so
  // take that as 5, (this is a traditional integer function!)
  // eg x=63 (111111), log2Val=5 (last one isn't counted by the while loop)
  return log2Val ;
}

// Returns the value of the highest bit in the number.
// Logically 2^( floor(log2(x)) ) gives the result,
// If the number is a power of 2, then
// it's just the number itself, if x isn't a
// power of 2, its the next lower power of 2.
inline unsigned int hibit( unsigned int x )
{
  unsigned int log2Val = log2i( x ) ; // e.g. 5
  return  (1 << log2Val) ;   // finds 2^5=32, 2^0=1
}

struct Gaussian
{
  float amplitude, centerPos, stddev, stddev2 ;
  
  Gaussian():amplitude(1.f),centerPos(0.f),stddev(1.f) { 
    stddev2=stddev*stddev;
  }
  
  Gaussian( float amp, float center, float widthStdDev ):
    amplitude(amp),centerPos(center),stddev(widthStdDev){
    stddev2=stddev*stddev;
  }
  
  float operator()( float x )
  {
    float diff2 = (x-centerPos) ;
    diff2 *= diff2 ;
    return amplitude * exp( -( .5f*diff2/(stddev2) ) ) ;
  }
};

// ( n ) =    n!/
// ( k )  k!(n-k)!
//
// also:
// (n) = ( n )
// (k)   ( n-k )
int binomial( int n, int k ) ;


// PrevPow2 can alias hibit. PrevPow2(63)=32, and hibit(63)=32 as well,
// they do the same thing.
#define PrevPow2 hibit

inline unsigned int NextPow2( unsigned int x )
{
  if( !x ) return 1 ; // avoid inane result if x==0. this result is wrong, but 0 isnt' a power of anything.

  x--; // if you pass pow2, you get same # back, ie pass 8, you get 8 back.
  unsigned int log2Val = log2i( x ) ;
  return (1 << (log2Val+1)) ;// NEXT pow2 will be increase this value by 1
}

inline unsigned int TwoToThe( unsigned int n )
{
  return 1 << n ;
}

inline bool isNear( float a, float b )
{
  return fabsf( a - b ) <= EPS_MIN ;
}

inline bool isNear( short a, short b, short tolerance )
{
  return abs( a - b ) <= tolerance ;
}

// returns true if `val` is within `eps` units of `reference`
inline bool isNear( float val, float reference, float eps )
{
  return fabsf( val-reference ) < eps ;
}

inline bool isBetween( float val, float lowerBound, float upperBound )
{
  return lowerBound <= val && val <= upperBound ;
}

//inline bool isBetween( float val, float boundA, float boundB )
//{
//  return (boundA <= val && val <= boundB) ||
//         (boundB <= val && val <= boundA) ;
//}

inline float lerp( float t, float A, float B ){
  return A + (B-A)*t ; // == A*(1.f-t) + B*t ;
}
inline float unlerp( float v, float A, float B ){
  return (v-A)/(B-A) ;
}


#include <limits.h>
// random between 0 and 1
inline float randFloat()
{
  // arc4random() is 0..4B (UINT_MAX).  It is NOT ULONG_MAX.
  // on ios ULONG_MAX=UINT_MAX, but on mac ULONG_MAX is ULONGLONG_MAX (9 trillion million or whatever)
  //return (float)arc4random() / UINT_MAX ; // THIS IS BAD.
  return MersenneTwister::genrand_int32()*(1.0/4294967295.0);  // same as genrand_real1();
}

// -1,1 => -1 + ( rand between 0 and 2 )
inline float randFloat( float low, float high )
{
  //return low + ((float)arc4random() / UINT_MAX)*(high-low) ;
  return low + (high-low)*randFloat() ;
}

// between low and (high-1)
inline int randInt( int low, int high )
{
  //return low + arc4random() % (high-low) ;
  return low + MersenneTwister::genrand_int32() % (high-low) ;
}

const char* getRandomInsult() ;

inline int randSign()
{
  return -1 + 2*randInt(0,2) ; // -1+0 or -1+2=+1
}

inline void randSeed( unsigned long seed )
{
  MersenneTwister::init_genrand( seed ) ;
}

inline void fillRandom( void* dst, int bytes )
{
  unsigned int* d = (unsigned int*)dst; // look at data as int32's
  int len= bytes/4; // last few may not be written
  for( int i = 0; i < len ; i++ )
    d[i] = (int)MersenneTwister::genrand_int32() ;
}


// Reverses a mapping of map: T -> vector<S> to map: S -> vector<T>
template<typename T, typename S>
void reverseMapping( map< T, vector<S> >& oMapping, map< S, vector<T> >& revMapping )
{
  for( typename map< T, vector<S> >::iterator iter = oMapping.begin() ; iter != oMapping.end() ; ++iter )
  {
    for( int i = 0 ; i < iter->second.size() ; i++ )
    {
      pair< typename map< S, vector<T> >::iterator, bool >   res = 
        revMapping.insert( make_pair( iter->second[i], vector<T>() ) ) ;
      
      res.first->second.push_back( iter->first ) ;
    }
  }
}

template<typename T>
bool contains( const list<T>& container, const T& elt )
{
  return find( container.begin(), container.end(), elt ) != container.end() ;
}

template<typename T>
bool contains( const vector<T>& container, const T& elt )
{
  return find( container.begin(), container.end(), elt ) != container.end() ;
}

inline float& clamp( float& x, float minVal, float maxVal ) {
  if( x < minVal ) x = minVal ;
  else if( x > maxVal ) x = maxVal ;
  return x ;
}

inline float& clampBelow( float& x, float minVal ) {
  if( x < minVal ) x = minVal ;
  return x ;
}
// for when x is an r-value
//inline float clampBelow( float x, float minVal ) {
//  if( x < minVal ) x = minVal ;
//  return x ;
//}

inline float& clampAbove( float& x, float maxVal ) {
  if( x > maxVal ) x = maxVal ;
  return x ;
}

inline int& clamp( int& x, int minVal, int maxVal ) {
  if( x < minVal ) x = minVal ;
  else if( x > maxVal ) x = maxVal ;
  return x ;
}

// clamp below by 0
inline float clamp_0( float x ) {
  if( x < 0.0f ) return 0.0f ;
  else return x ;
}

// clamp below by 0 and above by 1
inline float clamp_01( float x ) {
  if( x < 0.0f ) return 0.0f ;
  else if( x > 1.0f ) return 1.0f ;
  else return x ;
}

// clamp below by -1 and above by +1
inline float clamp_11( float x ) {
  if( x < -1.0f ) return -1.0f ;
  else if( x > 1.0f ) return 1.0f ;
  else return x ;
}

inline float clampedCopy( float x, float minVal, float maxVal ) {
  if( x < minVal ) return minVal ;
  else if( x > maxVal ) return maxVal ;
  else return x ;
}

inline float sign( float f ){
  return f > 0 ? 1.f : -1.f ;
}

inline int sign( int i ){
  return i > 0 ? 1 : -1 ;
}

#define cycleFlag( val, MINVAL, MAXVAL ) ( ++val>MAXVAL?val=MINVAL:val )
#define decycleFlag( val, MINVAL, MAXVAL ) ( --val<MINVAL?val=MAXVAL:val )

/// Logging.

enum ErrorLevel
{
  Info    = 1 << 0, // 1
  Warning = 1 << 1, // 2
  Error   = 1 << 2  // 4
} ;



enum ConsoleColor
{
  ConsoleRed,ConsoleYellow,ConsoleGray
} ;



double getClockS() ;



extern const char* progname ;
extern const char* ErrorLevelName[] ;

// String.






// THESE ARE MACROS SO THAT THE C-FUNCTIONS CAN OPERATE ON THE POINTER
// WITHOUT HAVING TO HAVE str=cstrNextWord(str); SYNTAX.
// INSTEAD YOU CAN JUST WRITE cstrNextWord(str);.
// another way to achieve this is cStrNextWord(&str),
// but passing a double ptr is tedious.

/// Skip ALL whitespace to the next word
/// in the string.  If there is no next word,
/// you will get a NULL
#define cstrNextWord(str) {str=strchr(str, ' '); if(str){while( (*str) && isspace(*str) ){++str;} if(!(*str)){str=0;}}}
//                         1. find next ' '      2.  if there was a ' ', adv past all whitespace   3. if nothing but white space null out str
// DONE THIS WAY FOR A REASON BUT COULD USE REVIEW

#define cstrNextLine( str )  while( *str && *str!='\n' ) ++str;


// Really terrible, this is NOT platform independent, though I thought it was.
// On WINDOWS, "\n" matches "\r\n", (isn't that weird), 
// but on MAC, "\n" matches the (essentially superfluous) "\n" ONLY
// while leaving the "\r" there (which acts totally as a newline on mac.
// On Linux, there would only be a "\n".
inline void cstrNullLastNL(char* str)
{
  char* nl=strrchr(str,'\n'); // \r\n so \n FIRST.
  if( nl )  *nl=0; 
                             
  char* rl=strrchr(str,'\r');
  if( rl )  *rl=0;  
}

inline void cstrNullNextSpace(char* str)
{
  char* ns=strchr(str,' ') ;
  if( ns )  *ns=0 ;
}

inline int cstrCountNonWhitespace( const char* str )
{
  int c=0;
  while( *str ){
    if( !isspace(*str) ) c++ ;
    str++;
  }
  return c;
}

int cFilesize( FILE* file ) ;

void cFileReadBinary( void* dstPtr, const char* filename ) ;

/// returns TRUE of a character x
/// is a whitespace character
///    9 == horizontal tab
///   10 == line feed,
///   11 == "vertical tab" (not that anyone uses it..)
///   13 == carriage return
///   32 == space
//#define IS_WHITE(x) (x==9 || x==10 || x==11 || x==13 || x==32) // use isspace() instead

tm* getCurrentTime();

char* getCurrentTimeString();

char* catcpy( char* s1, char* s2 ) ;



// Moves ELT_TO_REPLACE_WITH in the
// place that ELT_TO_DESTROY _was_.
template <typename T> 
void replaceElt( list<T>& theList,
  typename list<T>::iterator ELT_TO_DESTROY,
  typename list<T>::iterator ELT_TO_REPLACE_WITH )
{
  // delete the one you need to delete, point to one after it
  ELT_TO_DESTROY = theList.erase( ELT_TO_DESTROY ) ;

  // move ELT_TO_REPLACE_WITH just in front of the one after you removed..
  // ie replacing it.
  theList.splice( ELT_TO_DESTROY, theList, ELT_TO_REPLACE_WITH ) ;  
}

// use:
//CREATE_MEMBER_DETECTOR(normal);
//if( Detect_normal<Vertex>::value ) { code to run } 
// Check is __still runtime__, this is bad.


/////////////////LOGGING


// decorates the log message with [appname][thread][error level][current time]:  message
void logDecorate( int logLevel, short color, const char *fmt, va_list args );

string logDecorateGetString( int logLevel, const char *fmt, va_list args ) ;

void error( const char *fmt, ... );
void warning( const char *fmt, ... );

#define dissolve(x) x
//#define dissolve(x)

void info( short iColor, const char *fmt, ... );

void info( const char *fmt, ... );

string makeString( const char *fmt, ... );

string makeString( const char *fmt, va_list args ) ;

//template<typename T>
//inline void clamp( T& x, T minVal, T maxVal ) {
//  if( x < minVal ) x = minVal ;
//  if( x > maxVal ) x = maxVal ;
//}



struct WaveGen
{
  // Has to be a ptr to support write into foreign buffer w/o copy.
  //vector<short>* data ;
  int samplingRate ;
  float Ts ;
  short* data ;
  int n, startSample, endSample ; // #samples
  
  // I MUST know sampling rate, and # samples.
  WaveGen( short *iData, int iNumSamples, int iSamplingRate );
  //~WaveGen(){ delete data ; }
  void zero() ;
  
  void addSinFifth( float freq );
  void addSinWave( float freq );
  void addSinWave( float freq, float mul );
  void addSinWave( float freq, float mul, float add ) ;
  
  void setSinFifth( float freq );
  void setSinWave( float freq );
  void setSinWave( float freq, float mul );
  void setSinWave( float freq, float mul, float add ) ;
  
  void setSinWavePart( float freq, short tolerance, int iStartSample );
  
  
  void pulse( float pulseDownS, float pulseUpS ) ;
  void fm( float fModulator, float fCarrier, float ampModulator ) ;

} ;



#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#endif

// TIMER class, only available if C++ available
class Timer
{
  #ifdef _WIN32
  LARGE_INTEGER startTime ;
  double fFreq ;
  #else
  timeval startTime ;
  #endif
  
public:
  Timer() {
    #ifdef _WIN32
    LARGE_INTEGER freq ;
    QueryPerformanceFrequency( &freq ) ;
    fFreq = (double)freq.QuadPart ;
    #else
    gettimeofday( &startTime, NULL ) ;
    #endif
    reset();
  }

  void reset() {
    #ifdef _WIN32
    QueryPerformanceCounter( &startTime ) ;
    #else
    gettimeofday( &startTime, NULL ) ;
    #endif
  }

  // Gets the most up to date time.
  double getTime() const {
    #ifdef _WIN32
    LARGE_INTEGER endTime ;
    QueryPerformanceCounter( &endTime ) ;
    return ( endTime.QuadPart - startTime.QuadPart ) / fFreq ; // as double
    #else
    timeval endTime ;
    gettimeofday( &endTime, NULL ) ;
    return (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_usec - startTime.tv_usec)/1e6 ;
    #endif
  }
} ;


extern double startTime ;


#endif