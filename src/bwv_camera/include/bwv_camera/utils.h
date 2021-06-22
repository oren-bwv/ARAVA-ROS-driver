#ifndef __UTILS_H__
#define __UTILS_H__

/// thread creation wrapper
///
/// @param funcion a pointer to thread fuction with the prototype
///~~~~~~~~~~~~~~~~~~~~~~.cpp
///	void func(void* args)
///~~~~~~~~~~~~~~~~~~~~~~
/// @param args the thread pass argument
/// @param result thread return value
void* CreateThread(void* function, void* args, int* result);

/// sleep function wrapper
/// @param ms sleep time in milliseconds
void Sleep(int ms);


#endif // __UTILS_H__
