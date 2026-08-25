#ifndef HOST_UTIL_LOG_H
#define HOST_UTIL_LOG_H

void RTOS_Test_Log(const char* tag, const char* fmt, ...);

#define UTIL_LOG_E(tag, ...) RTOS_Test_Log((tag), __VA_ARGS__)

#endif /* HOST_UTIL_LOG_H */
