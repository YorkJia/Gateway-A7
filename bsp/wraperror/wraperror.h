#ifndef __WRAP_ERROR_H
#define __WRAP_ERROR_H

#include <unp.h>		/* for definition of errno */



/*	Nonfatal error related to a system call.
 *  Print a message and return. */
void	err_ret(const char *fmt, ...);

/*	Fatal error related to a system call.
 *  Print a message and return. */
 void	err_sys(const char *fmt, ...);

/*	Fatal error related to a system call.
 *  print a message, dump core, and terminate. */
void	err_dump(const char *fmt, ...);

 /* Nonfatal error unrelated to a system call.
  *	print a message and return. */
void	err_msg(const char *fmt, ...);

 /* Fatal error unrelated to a system call.
  *	print a message and terminate. */
void	err_quit(const char *fmt, ...);

#endif
