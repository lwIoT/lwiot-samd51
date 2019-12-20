/*
 *
 */

int __attribute__((weak)) _write(int file, char *ptr, int len); /* Remove GCC compiler warning */

int __attribute__((weak)) _write(int file, char *ptr, int len)
{
	return 0;
}

int __attribute__((weak)) _read(int file, char *ptr, int len); /* Remove GCC compiler warning */

int __attribute__((weak)) _read(int file, char *ptr, int len)
{
	return 0;
}
