/**
 * Convenient informative exceptions.
 *
 * Define EXCEPTION_BACKTRACE if You want a function backtrace printed each time
 * an exception is thrown (only for Linux).
 *
 * @author Michael Zillich
 *
 * @version $Id: Except.cc,v 1.2 2009/01/08 16:23:10 mz Exp $
 */

#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#ifdef EXCEPTION_BACKTRACE
#include <execinfo.h>
#endif
#include <airskin/Except.h>

/**
 * Except constructor.
 * @param file     (in) __FILE__ macro
 * @param function (in) __FUNCTION__ macro
 * @param line     (in) __LINE__ macro
 * @param format   (in) printf-style format string
 */
Except::Except(const char *file, const char *function, int line,
               const char *format, ...) throw()
{
  static char what[1024];
  static char msg[1024];
  va_list arg_list;
  va_start(arg_list, format);
  vsnprintf(what, 1024, format, arg_list);
  va_end(arg_list);
  snprintf(msg, 1024, "%s:%s:%d: %s", file, function, line, what);
  _what = msg;

#ifdef EXCEPTION_BACKTRACE
  // print a function backtrace, good to know even if no one catches this
  // exception
  void *array[25];
  int n = backtrace(array, 25);
  char ** symbols = backtrace_symbols(array, n);
  printf("* throwing exception, function backtrace:\n");
  for (int i = 0; i < n; i++)
    printf("%s\n", symbols[i]);
  free(symbols);
#endif
}

