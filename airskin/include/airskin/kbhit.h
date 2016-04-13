#ifndef KBHIT_H
#define KBHIT_H

/**
 * Enable kbhit() behaviour.
 * NOTE: not thread-safe!
 */
extern void kbhit_enable();

/**
 * Disable kbhit() behaviour.
 * NOTE: not thread-safe!
 */
extern void kbhit_disable();

/**
 * Returns true if a key was hit. Returns immediately.
 * Use getchar() then to get the actual key.
 */
extern bool kbhit();

#endif

