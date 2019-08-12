#include <wrapsqlite.h>
#include <wraperror.h>
#include <sqlite3.h>
/* the wrap for the sqlite3_open,
 * if return err, printf the err msg, and return */
int
Sqlite3_open(const char *filename, sqlite3 **ppDb)
{
	int res;
	if( (res = sqlite3_open(filename, ppDb)) != SQLITE_OK){
		err_msg("sqlite3 open error.");
	}
	return res;
}


int Sqlite3_exec(
	sqlite3 *db,														/* An  open database */
	const char *sql,													/* SQL to be evaluated */
	int (*callback)(void *, int, char**, char**),	/* Callback function */
	void *arg1,															/* 1st argument to callback */
	char **errmsg 														/* Error msg written here */
)
{
	int res;
	if( (res = sqlite3_exec(db, sql, callback, arg1, errmsg)) != SQLITE_OK){
		sqlite3_close(db);
		err_msg("sqlite3_exec fail.");
	}
	return res;
}

int Sqlite3_prepare_v2(
  sqlite3 *db,            /* Database handle */
  const char *zSql,       /* SQL statement, UTF-8 encoded */
  int nByte,              /* Maximum length of zSql in bytes. */
  sqlite3_stmt **ppStmt,  /* OUT: Statement handle */
  const char **pzTail     /* OUT: Pointer to unused portion of zSql */
)
{
	int res;
	const char *msg = "prepare error.";

	if( (res = sqlite3_prepare_v2(db, zSql, nByte, ppStmt, pzTail)) != SQLITE_OK){
		sqlite3_close(db);
		err_msg( msg );
	}
	return res;
}


