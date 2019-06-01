#ifndef _JSC_API_H
#define _JSC_API_H

#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif

#ifdef MYDLLINTERN
#define EXPORT __declspec (dllexport) 
#else
#define EXPORT __declspec (dllimport) 
#endif

typedef void* JSValue;

EXPORT void JSSetLogFile(FILE *file);
EXPORT void JSInit();
EXPORT void JSDestroy();
EXPORT void JSTestPrint(const char *msg);
EXPORT void JSFreeValue(JSValue value);
EXPORT void JSHandleEvents();
EXPORT int JSHaveEvents();

//if an error occurs, JSExec will return the exception and set *error to 1.
//remember to call JSFreeValue on the result
EXPORT JSValue JSExec(const char *script, int *error);

#ifdef __cplusplus
}
#endif
#endif /* _JSC_API_H */
