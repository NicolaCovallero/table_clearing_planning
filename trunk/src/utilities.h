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

/**
 * @brief Axis Aligned Bounding Box
 * @details Used for the objects
 * 
 */
struct AABB{
  double deep; ///<x dimension - dir1 or 2
  double width; ///< y dimension - dir 3 or 4
  double height; ///< z dimension
};


// tic toc matla b style
// reference: http://stackoverflow.com/questions/13485266/how-to-have-matlab-tic-toc-in-c
void tic();
void toc();

#endif
