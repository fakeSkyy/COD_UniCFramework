/**
 * @file case_runner.h
 * @author Kiro
 * @date 2026/8/23
 * @version 1.0
 */

#ifndef APPLICATION_CASE_RUNNER_H
#define APPLICATION_CASE_RUNNER_H

#include <stdio.h>
#include <string.h>

#include "unity.h"

#define APP_CASE(name)                                                                             \
    do                                                                                             \
    {                                                                                              \
        if (strcmp(argv[1], #name) == 0)                                                           \
        {                                                                                          \
            UNITY_BEGIN();                                                                         \
            RUN_TEST(test_##name);                                                                 \
            return UNITY_END();                                                                    \
        }                                                                                          \
    } while (0)

#define APP_CASES_BEGIN()                                                                          \
    do                                                                                             \
    {                                                                                              \
        if (argc != 2)                                                                             \
        {                                                                                          \
            fprintf(stderr, "usage: %s <case>\n", argv[0]);                                        \
            return 2;                                                                              \
        }                                                                                          \
    } while (0)

#define APP_CASES_END()                                                                            \
    do                                                                                             \
    {                                                                                              \
        fprintf(stderr, "unknown case: %s\n", argv[1]);                                            \
        return 2;                                                                                  \
    } while (0)

#endif /* APPLICATION_CASE_RUNNER_H */
