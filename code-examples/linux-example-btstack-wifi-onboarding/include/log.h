/*
 * $ Copyright Cypress Semiconductor $
 */

/** @file
 *  DEBUG LOG MACRO
 *
 *
 */

#ifndef __LOG_H__
#define __LOG_H__

#define TAG ""
#define TRACE_MSG(f_, ...) printf((f_), ##__VA_ARGS__), printf("\n")
#define TRACE_LOG(f_, ...) printf("%s", TAG), printf("[%s]:", __func__), printf((f_), ##__VA_ARGS__), printf("\n")
#define TRACE_WARNING(f_, ...) printf("%s[WNG]", TAG), printf("[%s]:", __func__), printf((f_), ##__VA_ARGS__), printf("\n")
#define TRACE_ERR(f_, ...) printf("%s[ERROR]", TAG), printf("[%s]:", __func__), printf((f_), ##__VA_ARGS__), printf("\n")

#if 0
#define TRACE_LOG_TIME(f_, ...) printf("%s ", timestamp()), printf("%s", TAG), printf("[%s]:", __func__), printf((f_), ##__VA_ARGS__), printf("\n")
#define TRACE_ERR_TIME(f_, ...) printf("%s ", timestamp()), printf("%s[ERROR]", TAG), printf("[%s]:", __func__), printf((f_), ##__VA_ARGS__), printf("\n")
char * timestamp(); 
char * timestamp(){
    time_t now = time(NULL); 
    char * time = asctime(gmtime(&now));
    time[strlen(time)-1] = '\0';
    return time;
}
#endif


#endif

