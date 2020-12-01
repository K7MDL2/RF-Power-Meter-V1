#ifndef UTITLITIES_H
#define UTITLITIES_H
/*******************************************************************************
 * TYPES
 *******************************************************************************/
#include <stdint.h>

/*******************************************************************************
 * MACROS
 *******************************************************************************/
#define ClearArray(arr) memset(arr, 0, sizeof(arr) / sizeof(arr[0]))
#define ClearString(str) memset(str, 0, strlen(str))
/*
    #define ArrayCopy(d, s, l) \
    ClearArray(d);         \
    memcpy(d, s, l)
*/    
#define StringCopy(d, s) \
    ClearString(d);      \
    memcpy(d, s, strlen(s))
#define StringAppend(d, s) strcat(d, s)
/*******************************************************************************
 * FUNCTIONS
 *******************************************************************************/
unsigned char ArrayLength(char *);

// char *utoa(char *str, unsigned int value, int radix);
#endif
