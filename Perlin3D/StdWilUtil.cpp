#include "StdWilUtil.h"

const char* getRandomInsult()
{
  static const char* insults[] = {
    "ninny", //"nincompoop",
    "numpty", 
    "Wally",
    "Reclining Renion",
    "man",
    "girlfriend",
    "girl",
    "gurl",
    "Betsy",
    "my friend",
    
    //"Silly Goose",
    //"Stern Widget",
    //"Silly Harry",
    //"Rastafarian",
    //"total Jorge",
    //"lack of social awareness Megaman",
    //"Jordan",
    //"Jelly",
    //"Johnston",
    //"Jaxon",
    //"Jemmidi",
    //"Jeramco",
    //"Jelenik",
    
  } ;
  
  return insults[ randInt(0,10) ] ;
}

int binomial( int n, int k )
{
  int nums = 1;
  int dens = 1;
  for( int i = 1 ; i <= k ; i++ )
  {
    nums *= ( n - k + i ) ;
    dens *= i ;
  }
  return nums/dens ;
}

double getClockS()
{
  //return [NSDate timeIntervalSinceReferenceDate] ; // NSTimeInterval is always specified in seconds 
  return 0.0 ;
}

int cFilesize( FILE* file )
{
  fseek( file, 0, SEEK_END ) ;
  int sizeVar = (int)ftell( file ) ;
  rewind( file ) ;
  return sizeVar ;
}

void cFileReadBinary( void* dstPtr, const char* filename )
{
  FILE *file = fopen( filename,"rb" ) ;
  if( file )
  {
    fseek( file, 0, SEEK_END ) ;
    int fileSize = (int)ftell( file ) ;
    dstPtr = malloc( fileSize ) ;
    rewind( file ) ;
    fread( dstPtr, fileSize, 1, file ) ;
    fclose( file ) ;
  }
  else
  {
    printf( "Can't open '%s'\n", filename ) ;
  }
}

tm* getCurrentTime()
{
  static time_t raw ;

  // grab the current time
  time( &raw ) ;

  // Now create that timeinfo struct
  static tm* timeinfo ;
  timeinfo = localtime( &raw ) ;

  return timeinfo ;
}

char* getCurrentTimeString()
{
  // write time into timeBuff
  static char timeBuf[ 256 ] ;
  strftime( timeBuf, 255, "%c", getCurrentTime() ) ;
  return timeBuf ;
}

// writes s1 and s2 into a new string and returns it
char* catcpy( char* s1, char* s2 )
{
  char* res = (char*)malloc( strlen(s1)+strlen(s2)+1 ) ;
  sprintf( res, "%s%s", s1, s2 ) ;
  return res ;
}



// LOGGING
const char* progname = "prog" ;

//  Info    = 1 << 0, // 1
//  Warning = 1 << 1, // 2
//  Error   = 1 << 2  // 4
const char* ErrorLevelName[] = {
  "", //0
  "Info", //1
  "Warning", //2
  "",//3
  "Error", //4
  "","","" //5,6,7
} ;

// decorates the log message with [appname][thread][error level][current time]:  message
void logDecorate( int logLevel, short color, const char *fmt, va_list args )
{
  // to be threadsafe, removed static
  char msgBuffer[ 4096 ] ;  // oops. Had a 623 char error (from shader) and it err-d out.
  vsprintf( msgBuffer, fmt, args ) ;
  
  // write time into timeBuff. Should be about 8 chars hh:mm:ss
  char timeBuf[ 32 ] ;
  strftime( timeBuf, 255, "%X", getCurrentTime() ) ;

  // Put it all together
  char buf[ 4096 ] ;
  
  sprintf( buf, "[ %s ][ %s ][ %s ]:  %s", progname, ErrorLevelName[ logLevel ], timeBuf, msgBuffer ) ;
  
  //printf( "%s\n", buf ) ; // don't want inserted.
  puts( buf ) ;
}

string logDecorateGetString( int logLevel, const char *fmt, va_list args )
{
  // to be threadsafe, removed static
  char msgBuffer[ 4096 ] ;  // oops. Had a 623 char error (from shader) and it err-d out.
  vsprintf( msgBuffer, fmt, args ) ;
  
  // write time into timeBuff. Should be about 8 chars hh:mm:ss
  char timeBuf[ 32 ] ;
  strftime( timeBuf, 255, "%X", getCurrentTime() ) ;

  // Put it all together
  char buf[ 4096 ] ;
  
  sprintf( buf, "[ %s ][ %s ][ %s ]:  %s", progname, ErrorLevelName[ logLevel ], timeBuf, msgBuffer ) ;
  
  //printf( "%s\n", buf ) ; // don't want inserted.
  return string( buf ) ;
}

void error( const char *fmt, ... )
{
  va_list lp ;
  va_start( lp, fmt ) ;

  logDecorate( ErrorLevel::Error, ConsoleRed, fmt, lp ) ;
}

void warning( const char *fmt, ... )
{
  va_list lp ;
  va_start( lp, fmt ) ;

  logDecorate( ErrorLevel::Warning, ConsoleYellow, fmt, lp ) ;
}

void info( short iColor, const char *fmt, ... )
{
  va_list lp ;
  va_start( lp, fmt ) ;

  logDecorate( ErrorLevel::Info, iColor, fmt, lp ) ;
}

void info( const char *fmt, ... )
{
  va_list lp ;
  va_start( lp, fmt ) ;

  logDecorate( ErrorLevel::Info, ConsoleGray, fmt, lp ) ;
}

// THESE ARE ABOUT CONVENIENCE, NOT EFFICIENCY.
// There are A LOT of allocations that go on here:
//  - the 4096 chr buffer
//  - the copy of the 4096 chr buffer (which is truncated at
//    exactly the right len)
// If we let msgBuffer be static, it will perform better but
// then won't be threadsafe.
string makeString( const char *fmt, ... )
{
  va_list args ;
  va_start( args, fmt ) ;
  char msgBuffer[ 4096 ] ; // max len.  Can also use a strlen call to avoid over alloc
  vsprintf( msgBuffer, fmt, args ) ;
  return string( msgBuffer ) ; // ALWAYS COPIES WHAT'S IN MSGBUFFER.
}

// Makes std c++ string from fmt and args.
string makeString( const char *fmt, va_list args )
{
  char msgBuffer[ 4096 ] ;  // oops. Had a 623 char error (from shader) and it err-d out.
  vsprintf( msgBuffer, fmt, args ) ;
  return string( msgBuffer ) ; // wrap it up
}



WaveGen::WaveGen( short *iData, int iNumSamples, int iSamplingRate ) :
  data(iData), n(iNumSamples), samplingRate(iSamplingRate)
{
  Ts = 1.0f/samplingRate ; // actual time between samples.
  
  startSample=0;
  endSample=n;
}

void WaveGen::zero()
{
  memset( data, 0, n*sizeof(short) ) ;
}

void WaveGen::addSinFifth( float freq )
{
  addSinWave( freq, 0.5f ) ;
  addSinWave( 1.5*freq, 0.5f ) ;
}

void WaveGen::addSinWave( float freq )
{
  float t = -Ts ; // First t+=Ts makes it 0
  for( int i = startSample ; i < endSample ; i++ )
    data[i] += (short)(SHRT_MAX*( sinf( (2*M_PI) * freq * (t+=Ts) ) ) ) ;
}

void WaveGen::addSinWave( float freq, float mul )
{
  float t = -Ts ;
  for( int i = startSample ; i < endSample ; i++ )
    data[i] += (short)(SHRT_MAX*( clamp_11( mul*sinf( (2*M_PI) * freq * (t+=Ts) ) ) ) ) ;
}

void WaveGen::addSinWave( float freq, float mul, float add )
{
  float t = -Ts ;
  for( int i = startSample ; i < endSample ; i++ )
    data[i] += (short)(SHRT_MAX*( clamp_11( add + mul*sinf( (2*M_PI) * freq * (t+=Ts) ) ) ) ) ;
}

void WaveGen::setSinFifth( float freq )
{
  setSinWave( freq, 0.5f ) ;
  addSinWave( 1.5*freq, 0.5f ) ;
}

void WaveGen::setSinWave( float freq )
{
  float t = -Ts ; // First t+=Ts makes it 0
  for( int i = startSample ; i < endSample ; i++ )
    data[i] = (short)(SHRT_MAX*( sinf( (2*M_PI) * freq * (t+=Ts) ) ) ) ;
}

void WaveGen::setSinWave( float freq, float mul )
{
  float t = -Ts ;
  for( int i = startSample ; i < endSample ; i++ )
    data[i] = (short)(SHRT_MAX*( clamp_11( mul*sinf( (2*M_PI) * freq * (t+=Ts) ) ) ) ) ;
}

void WaveGen::setSinWave( float freq, float mul, float add )
{
  float t = -Ts ;
  for( int i = startSample ; i < endSample ; i++ )
    data[i] = (short)(SHRT_MAX*( clamp_11( add + mul*sinf( (2*M_PI) * freq * (t+=Ts) ) ) ) ) ;
}

// this function has no point, it still pops.
void WaveGen::setSinWavePart( float freq, short tolerance, int iStartSample )
{
  // find close to 0
  
  int i = iStartSample ;
  while( i < n && i < iStartSample+1000 )
  {
    if( isNear( data[i], 0, tolerance ) ) // SHRT_MAX is 32767. oh like say a threshold of 30
    {
      printf( "%d near 0\n", data[i] ) ;
      break;
    }
    i++ ;
  }
  
  // go from startSample+off to N, 
  startSample=i ;
  // go to end.
  setSinWave( freq ) ;
  
  //then from 0 to startSample-1.
  startSample=0;
  endSample=iStartSample-1;
  setSinWave( freq ) ;
  
  endSample = n ;
}

// imposes a chopping effect
void WaveGen::pulse( float pulseDownS, float pulseUpS )
{
  int sampsDown = pulseDownS*samplingRate;
  int sampsUp = pulseUpS*samplingRate;
  
  int i = 0;
  while( i < n )
  {
    int j = 0 ;
    for( ; j < sampsDown && i < n ; j++ )
      data[i+j] = 0 ;
    i += j ;
    i += sampsUp ; // don't do anything to these
  }
}

void WaveGen::fm( float fModulator, float fCarrier, float ampModulator )
{
  float t = 0 ;
  for( int i = 0 ; i < n ; i++ )
  {
    // you go 
    float s1 = ampModulator*( 1.f + 0.5f*sinf( (2*M_PI) * fModulator * t ) ) ;
    data[i] = (short)( SHRT_MAX*clamp_11( s1*sinf( (2*M_PI) * fCarrier * t ) ) ) ;
    t += Ts ;
  }
}








