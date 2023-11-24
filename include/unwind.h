#pragma once

#include <setjmp.h>
#include <stdlib.h>

/* The maximum _ex wrapper call nesting depth + 1 (that is, the allowed depth is
 * one less than UNWIND_MAX_DEPTH). This is because the 0th jmp_buf is unused. */
#define UNWIND_MAX_DEPTH 16

#if defined(_WIN64)
/* This is to avoid longjmp crashing because of stack unwinding.
 * It is incompatible with the execution of generated code. */
#undef setjmp
#define setjmp(env) __builtin_setjmp(env)
#undef longjmp
#define longjmp(buf, val) __builtin_longjmp(buf, val)
#endif

/* We don't use our assert() because it would call tlib_abort which is a bad idea
 * in this context. We don't use <assert.h> either because we want these checks
 * to happen in release mode too - running off of either end would cause hard to
 * diagnose problems. */
#define unwind_assert(p) \
    do {                 \
        if (!(p))        \
            abort();     \
    } while (0)

extern __thread struct unwind_state {
    jmp_buf envs[UNWIND_MAX_DEPTH];
    int32_t env_idx;
} unwind_state;

#define PUSH_ENV() ({                                           \
    unwind_assert(unwind_state.env_idx < UNWIND_MAX_DEPTH - 1); \
    setjmp(unwind_state.envs[++unwind_state.env_idx]);          \
})

#define POP_ENV()                                 \
    do {                                          \
        --unwind_state.env_idx;                   \
        unwind_assert(unwind_state.env_idx >= 0); \
    } while (0)

/* value macros */
#define EXC_VALUE_0(RET, NAME, PLACEHOLDER) \
    RET NAME##_ex()                         \
    {                                       \
        RET ret = PLACEHOLDER;              \
        if (PUSH_ENV() == 0) {              \
            ret = NAME();                   \
        }                                   \
        POP_ENV();                          \
        return ret;                         \
    }

#define EXC_VALUE_1(RET, NAME, PLACEHOLDER, PARAMT1, PARAM1) \
    RET NAME##_ex(PARAMT1 PARAM1)                            \
    {                                                        \
        RET ret = PLACEHOLDER;                               \
        if (PUSH_ENV() == 0) {                               \
            ret = NAME(PARAM1);                              \
        }                                                    \
        POP_ENV();                                           \
        return ret;                                          \
    }

#define EXC_VALUE_2(RET, NAME, PLACEHOLDER, PARAMT1, PARAM1, PARAMT2, PARAM2) \
    RET NAME##_ex(PARAMT1 PARAM1, PARAMT2 PARAM2)                             \
    {                                                                         \
        RET ret = PLACEHOLDER;                                                \
        if (PUSH_ENV() == 0) {                                                \
            ret = NAME(PARAM1, PARAM2);                                       \
        }                                                                     \
        POP_ENV();                                                            \
        return ret;                                                           \
    }

#define EXC_VALUE_3(RET, NAME, PLACEHOLDER, PARAMT1, PARAM1, PARAMT2, PARAM2, PARAMT3, PARAM3) \
    RET NAME##_ex(PARAMT1 PARAM1, PARAMT2 PARAM2, PARAMT3 PARAM3)                              \
    {                                                                                          \
        RET ret = PLACEHOLDER;                                                                 \
        if (PUSH_ENV() == 0) {                                                                 \
            ret = NAME(PARAM1, PARAM2, PARAM3);                                                \
        }                                                                                      \
        POP_ENV();                                                                             \
        return ret;                                                                            \
    }

/* pointer macros */
#define EXC_POINTER_0(RET, NAME) EXC_VALUE_0(RET, NAME, NULL)

/* int macros */
#define EXC_INT_0(RET, NAME) EXC_VALUE_0(RET, NAME, 0)
#define EXC_INT_1(RET, NAME, PARAMT1, PARAM1) EXC_VALUE_1(RET, NAME, 0, PARAMT1, PARAM1)
#define EXC_INT_2(RET, NAME, PARAMT1, PARAM1, PARAMT2, PARAM2) EXC_VALUE_2(RET, NAME, 0, PARAMT1, PARAM1, PARAMT2, PARAM2)
#define EXC_INT_3(RET, NAME, PARAMT1, PARAM1, PARAMT2, PARAM2, PARAMT3, PARAM3) EXC_VALUE_3(RET, NAME, 0, PARAMT1, PARAM1, PARAMT2, PARAM2, PARAMT3, PARAM3)

/* void macros */
#define EXC_VOID_0(NAME)       \
    void NAME##_ex()           \
    {                          \
        if (PUSH_ENV() == 0) { \
            NAME();            \
        }                      \
        POP_ENV();             \
    }

#define EXC_VOID_1(NAME, PARAMT1, PARAM1) \
    void NAME##_ex(PARAMT1 PARAM1)        \
    {                                     \
        if (PUSH_ENV() == 0) {            \
            NAME(PARAM1);                 \
        }                                 \
        POP_ENV();                        \
    }

#define EXC_VOID_2(NAME, PARAMT1, PARAM1, PARAMT2, PARAM2) \
    void NAME##_ex(PARAMT1 PARAM1, PARAMT2 PARAM2)         \
    {                                                      \
        if (PUSH_ENV() == 0) {                             \
            NAME(PARAM1, PARAM2);                          \
        }                                                  \
        POP_ENV();                                         \
    }

#define EXC_VOID_3(NAME, PARAMT1, PARAM1, PARAMT2, PARAM2, PARAMT3, PARAM3) \
    void NAME##_ex(PARAMT1 PARAM1, PARAMT2 PARAM2, PARAMT3 PARAM3)          \
    {                                                                       \
        if (PUSH_ENV() == 0) {                                              \
            NAME(PARAM1, PARAM2, PARAM3);                                   \
        }                                                                   \
        POP_ENV();                                                          \
    }
