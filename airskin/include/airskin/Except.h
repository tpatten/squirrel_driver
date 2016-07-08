/**
 * Convenient informative exceptions.
 *
 * @author Michael Zillich
 *
 * @version $Id: Except.h,v 1.4 2009/04/24 09:22:35 mz Exp mz $
 */

#ifndef Z_EXCEPT_H
#define Z_EXCEPT_H

#include <cstring>
#include <stdexcept>
#include <sstream>

/**
 * A slightly hacky way to only get the pure file name without the whole path
 * from __FILE__
 */
#define __THIS_FILE__ ((strrchr(__FILE__, '/') ?: __FILE__ - 1) + 1)

/**
 * This is a convenient macro for throwing more descriptive exceptions.
 * instead of
 *   throw some_error("overflow error");
 *   which creates the output
 *   "overflow error"
 * use:
 *   THROW(some_error, "there were " << n << " " << animals << " in the tree.");
 *   to produce this output:
 *   "DumbFile.cc:StupidFunction:28: There were 42 elephants in the tree."
 *
 * Note that there is a class hierarchy of C++ standard exceptions
 * (see stdexcept):
 * exception
 *   logic_error
 *     domain_error, invalid_argument, length_error, out_of_range
 *   runtime_error
 *     range_error, overflow_error, underflow_error
 * Any class derived from logic_error or runtime_error can be the first argument
 * of THROW. (Note that the base class exception does not take a string argument
 * in the constructor.
 */
#ifndef THROW
#define THROW(ex,msg)\
{\
  std::ostringstream os;\
  os << __THIS_FILE__ << ':' << __FUNCTION__ << ':' << __LINE__ << ": " << msg;\
  throw ex(os.str());\
}
#endif

#ifndef __HERE__
#define __HERE__   __THIS_FILE__, __FUNCTION__, __LINE__
#endif

/**
 * An informative exception class.
 *
 * Similar to the above, but using a proper exception class.
 * Example:
 *   Except(__FILE__, __FUNCTION__, __LINE__, "There were %d %s in the tree.",
 *          n, animals);
 * output:
 *   "DumbFile.cc:StupidFunction:28: There were 42 elephants in the tree."
 * Note: You can use the __HERE__ macro to get shorter statements:
 *   Except(__HERE__, "There were %d %s in the tree.", 42, "elephants");
 */
class Except : public std::exception
{
private:
  std::string _what;

public:
  Except(const char *file, const char *function, int line,
         const char *format, ...) throw();
  virtual ~Except() throw() {}
  virtual const char* what() const throw() {return _what.c_str();}
  void Set(const std::string &s) {_what = s;}
};

#endif

