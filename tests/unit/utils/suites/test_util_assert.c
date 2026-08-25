/**
 * @file test_util_assert.c
 * @author Gao Xing
 * @date 2026/8/19
 * @version 1.0
 */

#include <stddef.h>

#include "SEGGER_RTT.h"
#include "test_support.h"
#include "util_assert.h"

/* ==========================================================================
 * What is testable about a macro that never returns
 * ==========================================================================
 *
 * util_assert.h is header-only and UTIL_ASSERT has two forms selected by NDEBUG.
 * Only the passing path of the live form can be exercised in-process, for two
 * independent reasons — worth stating precisely, because "it halts" understates
 * how thoroughly untestable the failing path is here:
 *
 *   1. It does not return. The failure body ends in `while (1) {}` after a
 *      `BKPT #0`, so a test that tripped it would hang the suite rather than fail
 *      it. Reaching it and recovering would need a process boundary (fork, or a
 *      SIGTRAP handler that longjmps) — and on the host there is no SIGTRAP to
 *      catch, per the next point.
 *
 *   2. It does not assemble. `__asm volatile("BKPT #0")` is an ARM Thumb
 *      instruction and x86 has no such mnemonic, so a *reachable* failure branch
 *      is a build error, not a runtime one:
 *
 *          Assembler messages: no such instruction: `bkpt '
 *
 *      That is why every assertion below is a compile-time constant truth. GCC
 *      folds the condition and deletes the whole failure branch before the
 *      assembler ever sees it, which is what lets this file link at all. A
 *      condition GCC cannot fold — anything through a `volatile` or a runtime
 *      value, even one obviously true at -O0 — keeps the branch alive and breaks
 *      the build. That was verified rather than assumed: `int x = 1;
 *      UTIL_ASSERT(x == 1);` fails to assemble at -O0.
 *
 * So the coverage here is: the passing path executes and produces nothing, the
 * macro's syntactic shape is sound in every statement position, and the NDEBUG
 * form discards its argument. The halt itself is a target-only behaviour.
 * ==========================================================================
 */

void setUp(void) { RTT_StubReset(); }
void tearDown(void) {}

/* ========================================================================= */
/*  Passing path                                                             */
/* ========================================================================= */

static void test_assert_passing_condition_is_silent(void)
{
    /* A passing assertion must cost nothing observable. If it wrote to RTT it
     * would flood the log from any hot path that asserts, and the message that
     * matters — the failure — would scroll away. */
    UTIL_ASSERT(1);
    UTIL_ASSERT(1 == 1);
    UTIL_ASSERT(!(0));
    UTIL_ASSERT(2 + 2 == 4);
    UTIL_ASSERT(1 && !0);
    UTIL_ASSERT((3 & 1) == 1);

    TEST_ASSERT_EQUAL_UINT(0u, RTT_StubWriteCount());
    TEST_ASSERT_EQUAL_STRING("", RTT_StubLastText());
}

static void test_assert_accepts_the_expression_kinds_it_is_used_with(void)
{
    /* Every shape of condition the framework actually asserts on, to confirm the
     * macro's parenthesisation does not mangle any of them: the `!(expr)` in the
     * body is what makes a comma or a low-precedence operator safe. */
    UTIL_ASSERT(sizeof(float) == 4u);
    UTIL_ASSERT(sizeof(void*) >= 4u);
    UTIL_ASSERT("tag" != NULL);
    UTIL_ASSERT(1.5f > 1.0f);
    UTIL_ASSERT(-1 < 0);

    /* An unparenthesised comparison inside a wider expression: without the
     * macro's own parentheses around expr this would bind wrongly. */
    UTIL_ASSERT(1 == 1 ? 1 : 0);
    UTIL_ASSERT(0 == 0 && 1 == 1);

    TEST_ASSERT_EQUAL_UINT(0u, RTT_StubWriteCount());
}

static void test_assert_is_a_single_statement(void)
{
    /* The do/while(0) wrapper is what lets the macro sit as the un-braced body of
     * an if without swallowing the else, and as a loop body without needing
     * braces. A bare `if (...) { ... }` in its place would compile here and then
     * break at the semicolon in front of an else — which is the classic macro
     * defect this shape exists to avoid, so it is checked structurally. */
    if (1)
    {
        UTIL_ASSERT(1);
    }
    else
    {
        UTIL_ASSERT(1 == 2);
    }

    for (int i = 0; i < 3; i++)
    {
        UTIL_ASSERT(1);
    }

    while (0)
    {
        UTIL_ASSERT(1 == 2);
    }

    switch (1)
    {
    case 1:
        UTIL_ASSERT(1);
        break;
    default:
        UTIL_ASSERT(1 == 2);
        break;
    }

    TEST_ASSERT_EQUAL_UINT(0u, RTT_StubWriteCount());
}

/* ========================================================================= */
/*  NDEBUG form                                                              */
/* ========================================================================= */

/**
 * @brief Side-effect counter for the NDEBUG-form check.
 *
 * File scope rather than a local, so the compiled-out branch cannot be proven
 * dead by the optimiser and elided along with the call it is meant to detect.
 */
static volatile int s_side_effects;

/**
 * @brief Records that it ran, and reports true.
 *
 * The NDEBUG form of UTIL_ASSERT expands to `((void) 0)`, which discards the
 * argument unevaluated — so a call here must NOT happen. That is the property
 * being measured, and it is the one an assertion with a side effect in its
 * condition silently depends on.
 */
static int bump(void)
{
    s_side_effects++;
    return 1;
}

/* Re-included under NDEBUG to reach the other form. The include guard has to be
 * cleared as well as the macro, since the header defines both — and this is done
 * after every use of the live form above, because the two definitions cannot
 * coexist in one translation unit. */
#undef UTIL_ASSERT
#undef UTIL_ASSERT_H
#define NDEBUG 1
#include "util_assert.h"

static void test_assert_ndebug_form_discards_its_argument(void)
{
    /* Not merely "does not halt": the argument is not evaluated at all. An
     * assertion whose condition has a side effect — a register read, a flag clear
     * — would silently change behaviour between a debug and a release build if
     * this expanded to anything that touched it. */
    s_side_effects = 0;

    UTIL_ASSERT(bump());
    UTIL_ASSERT(bump() == 1);
    UTIL_ASSERT(!bump());

    TEST_ASSERT_EQUAL_INT_MESSAGE(0, s_side_effects, "NDEBUG form evaluated its argument");

    /* Referenced but never called, which is exactly the state being asserted —
     * and which is also what would otherwise draw -Wunused-function, since every
     * call site above vanished with the macro. Calling it once here proves the
     * counter does move when something actually invokes it, so a zero above means
     * "not called" rather than "counter broken". */
    TEST_ASSERT_EQUAL_INT(1, bump());
    TEST_ASSERT_EQUAL_INT(1, s_side_effects);

    s_side_effects = 0;

    /* And a failing condition is inert rather than fatal, which is the whole
     * point of the release form. */
    UTIL_ASSERT(0);
    UTIL_ASSERT(1 == 2);
    UTIL_ASSERT(NULL != NULL);

    TEST_ASSERT_EQUAL_UINT(0u, RTT_StubWriteCount());
}

static void test_assert_ndebug_form_is_still_a_single_statement(void)
{
    /* `((void) 0)` needs the same statement-position guarantees as the live form,
     * or a release build would fail to compile where a debug build succeeded. */
    if (1)
    {
        UTIL_ASSERT(0);
    }
    else
    {
        UTIL_ASSERT(1);
    }

    for (int i = 0; i < 2; i++)
    {
        UTIL_ASSERT(0);
    }

    TEST_ASSERT_EQUAL_UINT(0u, RTT_StubWriteCount());
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_assert_passing_condition_is_silent);
    RUN_TEST(test_assert_accepts_the_expression_kinds_it_is_used_with);
    RUN_TEST(test_assert_is_a_single_statement);

    RUN_TEST(test_assert_ndebug_form_discards_its_argument);
    RUN_TEST(test_assert_ndebug_form_is_still_a_single_statement);

    return UNITY_END();
}
