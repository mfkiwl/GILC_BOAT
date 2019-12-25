/*
 * Author   : dsf90
 * Date     : 2018.12.7
 * Breif    : data log for gilc.
 * Version  : 0.1
 * Hisotry  : first created
 * Form     : ANSI
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "log.h"
#include "GILC_IOFile.h"

typedef int(*print_callback) (const char *, ...);

static int s_iDebugLevel = 0;
static print_callback s_log_callback = NULL;

void gilc__set_log_level(int level)
{
	if (level >= 0)
		s_iDebugLevel = level;
	else
		s_iDebugLevel = 0;
}

void gilc__set_log_print_callback(int(*fun) (const char *, ...))
{
	if (fun)
		s_log_callback = fun;
}

/*dsf90:2018.1217,实测库log函数名不能与进程log函数同名*/
void gilc_debug_log(unsigned char level, const char *file, int line, char* fmt,...)
{  
	if (!s_iDebugLevel || level > s_iDebugLevel)
	{
		return;
	}

	char buf[2048] = {0};
	static const char * prefix[] = {"Log","Err","Info","Debug","Test" };

	if(level)
		sprintf(buf, "[ %s ] %s(%d): ", prefix[level], file, line);

	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)buf+strlen(buf),fmt,ap);
	va_end(ap);

	if (s_log_callback)
	{
		s_log_callback(buf);
	}
	else
	{
		fprintf_log(buf);
		printf("%s",buf);
	}
}

void gilc_log_hex(char *head,int line_size,void *data,int size)  
{ 
	if (!s_iDebugLevel)
	{
		return;
	}

    int i; 
	unsigned char *addr = (unsigned char *)data;
	char buf[2048] = {0};
	sprintf(buf,"%s",head); 
	for(i=0;i<size;i++) 
	{ 
		if(i%line_size==0) 
		{ 
			sprintf(buf+strlen(buf),"\r\n"); 
		} 
		sprintf(buf+strlen(buf),"%02X ",addr[i]); 
	} 
	sprintf(buf+strlen(buf),"\r\n"); 

	if (s_log_callback)
	{
		s_log_callback(buf);
	}
	else
	{
		fprintf_log(buf);
		printf("%s", buf);
	}
}

