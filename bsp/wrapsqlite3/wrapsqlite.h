#ifndef	__WRAP_SQLITE_H
#define __WRAP_SQLITE_H

#include <unp.h>
#include <sqlite3.h>



/* the wrap for the sqlite3_open,
 * if return err, printf the err msg, and return */
int	Sqlite3_open(const char *filename, sqlite3 **ppDb);


int Sqlite3_exec(
	sqlite3 *db,														/* An  open database */
	const char *sql,													/* SQL to be evaluated */
	int (*callback)(void *, int, char**, char**),	/* Callback function */
	void *arg1,															/* 1st argument to callback */
	char **errmsg 														/* Error msg written here */
);

int Sqlite3_prepare_v2(
  sqlite3 *db,            /* Database handle */
  const char *zSql,       /* SQL statement, UTF-8 encoded */
  int nByte,              /* Maximum length of zSql in bytes. */
  sqlite3_stmt **ppStmt,  /* OUT: Statement handle */
  const char **pzTail     /* OUT: Pointer to unused portion of zSql */
);


#endif 

