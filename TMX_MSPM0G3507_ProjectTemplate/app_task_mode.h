#ifndef APP_TASK_MODE_H
#define APP_TASK_MODE_H

/*
 * Single-task test switch.
 *
 * 0: normal acceptance firmware, wait for button/UART task commands.
 * 1: run question 1 only, A -> B.
 * 2: run question 2 only, A -> B -> C -> D -> A.
 * 3: run question 3 only, A -> C -> B -> D -> A, one lap.
 */
#define APP_SINGLE_TEST_TASK_ID 0U

/* Delay before an auto-started single-task test, leaving time to place the car. */
#define APP_SINGLE_TEST_START_DELAY_MS 1000U

#endif /* APP_TASK_MODE_H */
