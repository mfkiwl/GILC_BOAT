/*
 * Author   : dsf90
 * Date     : 2018.8.16
 * Breif    : data save for gilc.
 * Version  : 0.1
 * Hisotry  : first created
 * Form     : ANSI
 */
#ifndef _GILC_LOG_H__
#define _GILC_LOG_H__

#include<stdio.h>

#ifdef __cplusplus
extern "C" {
#endif				/*__cplusplus*/

#define gilc_log(fmt,...)   gilc_debug_log(0, NULL, 0, fmt ,##__VA_ARGS__)
#define loge(fmt,...)       gilc_debug_log(1, __FILE__, __LINE__, fmt ,##__VA_ARGS__)
#define logi(fmt,...)       gilc_debug_log(2, __FILE__, __LINE__, fmt ,##__VA_ARGS__)
#define logd(fmt,...)       gilc_debug_log(3, __FILE__, __LINE__, fmt ,##__VA_ARGS__)
#define logt(fmt,...)       gilc_debug_log(4, __FILE__, __LINE__, fmt ,##__VA_ARGS__)

extern FILE *s_flog;

void gilc_debug_log(unsigned char level, const char *file, int line, char* fmt, ...);
void gilc_log_hex(char *head, int line_size, void *data, int size);

#ifdef __cplusplus
}
#endif			/*__cplusplus*/

#endif
