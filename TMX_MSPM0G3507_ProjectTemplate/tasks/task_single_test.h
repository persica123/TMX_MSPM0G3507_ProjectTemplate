#ifndef TASK_SINGLE_TEST_H
#define TASK_SINGLE_TEST_H

/*
 * Compile-time single-task entry points.
 *
 * This header is included after task_dispatcher.h, so the normal task
 * preparation helpers and task functions are already visible in this
 * translation unit.
 */

#include <stdint.h>

#include "app_task_ids.h"
#include "app_task_mode.h"
#include "app_services.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "tasks/question1_only.h"
#include "tasks/question2_only.h"
#include "tasks/question3_only.h"

static task_id_t single_test_selected_task(void)
{
#if APP_SINGLE_TEST_TASK_ID == 1U
    return TASK_ID_1;
#elif APP_SINGLE_TEST_TASK_ID == 2U
    return TASK_ID_2;
#elif APP_SINGLE_TEST_TASK_ID == 3U
    return TASK_ID_3;
#else
    return TASK_ID_NONE;
#endif
}

static uint8_t run_single_task_test_if_enabled(void)
{
    task_id_t task_id = single_test_selected_task();

    if (task_id == TASK_ID_NONE) {
        return 0U;
    }

    TB6612_Brake();
    lc_printf("SINGLE_TEST armed: task=%u autostart_delay=%lu ms\r\n",
        task_id,
        (uint32_t)APP_SINGLE_TEST_START_DELAY_MS);
    delay_ms_with_st011(APP_SINGLE_TEST_START_DELAY_MS);

    if (task_id == TASK_ID_1) {
        run_question1_only_test();
    } else if (task_id == TASK_ID_2) {
        run_question2_only_test();
    } else {
        run_question3_only_test();
    }

    TB6612_Brake();
    st011_finish_pending_pulse();
    lc_printf("SINGLE_TEST finished: task=%u\r\n", task_id);
    return 1U;
}

#endif /* TASK_SINGLE_TEST_H */
