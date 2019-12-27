/*
 * C++ support functions.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include <stdlib.h>

void *operator new(size_t size)
{
	return malloc(size);
}

void *operator new[](size_t size)
{
	return malloc(size);
}

void operator delete(void *ptr, unsigned int)
{
	free(ptr);
}

void operator delete(void *ptr)
{
	free(ptr);
}

void operator delete[](void *ptr)
{
	free(ptr);
}

void operator delete[](void *ptr, unsigned int)
{
	free(ptr);
}
