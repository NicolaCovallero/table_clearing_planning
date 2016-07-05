/** @file 
*	@brief A file with utilities functions
*
*/
/** @fn void tic()
 * @brief Start measuring the time
 * @details This function implements the tic Matlab function style to measure easily the elapsed time.
 * reference: http://stackoverflow.com/questions/13485266/how-to-have-matlab-tic-toc-in-c
 * \n Usage:
 * \code
 * tic();
 * ..do something
 * toc();
 * \endcode
 */
 /** @fn void toc()
 * @brief Stop measuring the time and print the elapsed time
 * @details This function implements the toc Matlab function style to measure easily the elapsed time.
 * reference: http://stackoverflow.com/questions/13485266/how-to-have-matlab-tic-toc-in-c
 * \n Usage:
 * \code
 * tic();
 * ..do something
 * toc();
 * \endcode
 */
#ifndef _UTILITIES_H
#define _UTILITIES_H

#include <iostream>
#include <stack>
#include <ctime>
#include <pcl/common/eigen.h>
#include <sys/time.h>
#include <unistd.h>

/**
 * @brief Oriented Bounding Box 
 * @details Used for the objects
 * 
 */
struct OBB{ 
  double deep; ///<x dimension - dir1 or 2
  double width; ///< y dimension - dir 3 or 4
  double height; ///< z dimension
};

// tic toc matla b style
// reference: http://stackoverflow.com/questions/13485266/how-to-have-matlab-tic-toc-in-c
void tic();
void toc();

namespace util{

	/* Remove if already defined */
	typedef long long int64; 
	typedef unsigned long long uint64;

	// reference: http://stackoverflow.com/questions/1861294/how-to-calculate-execution-time-of-a-code-snippet-in-c

	/* Returns the amount of milliseconds elapsed since the UNIX epoch. Works on both
	 * windows and linux. */
	static uint64 GetTimeMs64()
	{
	 /* Linux */
	 struct timeval tv;

	 gettimeofday(&tv, NULL);

	 uint64 ret = tv.tv_usec;
	 /* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
	 ret /= 1000;

	 /* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
	 ret += (tv.tv_sec * 1000);

	 return ret;
	}
}

#endif
