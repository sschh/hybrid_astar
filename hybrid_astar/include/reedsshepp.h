/*!
  \file reedsshepp.h
   \brief A reedsshepp path class for finding analytical solutions to the problem of the shortest path.

  It has been copied from https://github.com/AndrewWalker/Dubins-Curves/ under the WTFPL. Please refer to the author in case of questions.
  \author Andrew Walker
*/

#ifndef DUBINS_H
#define DUBINS_H

// Path types
#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

// Error codes
#define EDUBOK        (0)   // No error
#define EDUBCOCONFIGS (1)   // Colocated configurations
#define EDUBPARAM     (2)   // Path parameterisitation error
#define EDUBBADRHO    (3)   // the rho value is invalid
#define EDUBNOPATH    (4)   // no connection between configurations with this word

namespace HybridAStar {

// The various types of solvers for each of the path types
typedef int (*DubinsWord)(double, double, double, double* );

// A complete list of the possible solvers that could give optimal paths
extern DubinsWord reedsshepp_words[];

typedef struct
{
    double qi[3];       // the initial configuration
    double param[3];    // the lengths of the three segments
    double rho;         // model forward velocity / model angular velocity
    int type;           // path type. one of LSL, LSR, ...
} DubinsPath;

/**
 * Callback function for path sampling
 *
 * @note the q parameter is a configuration
 * @note the t parameter is the distance along the path
 * @note the user_data parameter is forwarded from the caller
 * @note return non-zero to denote sampling should be stopped
 */
typedef int (*DubinsPathSamplingCallback)(double q[3], double t, void* user_data);

/**
 * Generate a path from an initial configuration to
 * a target configuration, with a specified maximum turning
 * radii
 *
 * A configuration is (x, y, theta), where theta is in radians, with zero
 * along the line x = 0, and counter-clockwise is positive
 *
 * @param q0    - a configuration specified as an array of x, y, theta
 * @param q1    - a configuration specified as an array of x, y, theta
 * @param rho   - turning radius of the vehicle (forward velocity divided by maximum angular velocity)
 * @param path  - the resultant path
 * @return      - non-zero on error
 */
int reedsshepp_init( double q0[3], double q1[3], double rho, DubinsPath* path);

/**
 * Calculate the length of an initialised path
 *
 * @param path - the path to find the length of
 */
double reedsshepp_path_length( DubinsPath* path );

/**
 * Extract an integer that represents which path type was used
 *
 * @param path    - an initialised path
 * @return        - one of LSL, LSR, RSL, RSR, RLR or LRL (ie/ 0-5 inclusive)
 */
int reedsshepp_path_type( DubinsPath * path );

/**
 * Calculate the configuration along the path, using the parameter t
 *
 * @param path - an initialised path
 * @param t    - a length measure, where 0 <= t < reedsshepp_path_length(path)
 * @param q    - the configuration result
 * @returns    - non-zero if 't' is not in the correct range
 */
int reedsshepp_path_sample( DubinsPath* path, double t, double q[3]);

/**
 * Walk along the path at a fixed sampling interval, calling the
 * callback function at each interval
 *
 * @param path      - the path to sample
 * @param cb        - the callback function to call for each sample
 * @param user_data - optional information to pass on to the callback
 * @param stepSize  - the distance along the path for subsequent samples
 */
int reedsshepp_path_sample_many( DubinsPath* path, DubinsPathSamplingCallback cb, double stepSize, void* user_data );

/**
 * Convenience function to identify the endpoint of a path
 *
 * @param path - an initialised path
 * @param q    - the configuration result
 */
int reedsshepp_path_endpoint( DubinsPath* path, double q[3] );

/**
 * Convenience function to extract a subset of a path
 *
 * @param path    - an initialised path
 * @param t       - a length measure, where 0 < t < reedsshepp_path_length(path)
 * @param newpath - the resultant path
 */
int reedsshepp_extract_subpath( DubinsPath* path, double t, DubinsPath* newpath );

// Only exposed for testing purposes
int reedsshepp_LSL( double alpha, double beta, double d, double* outputs );
int reedsshepp_RSR( double alpha, double beta, double d, double* outputs );
int reedsshepp_LSR( double alpha, double beta, double d, double* outputs );
int reedsshepp_RSL( double alpha, double beta, double d, double* outputs );
int reedsshepp_LRL( double alpha, double beta, double d, double* outputs );
int reedsshepp_RLR( double alpha, double beta, double d, double* outputs );

}
#endif // DUBINS_H
