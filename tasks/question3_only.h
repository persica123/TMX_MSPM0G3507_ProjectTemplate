#ifndef QUESTION3_ONLY_H
#define QUESTION3_ONLY_H

#include "app_task_ids.h"

static void run_question3_only_test(void)
{
    prepare_task_start(TASK_ID_3);
    run_race_laps(1U);
}

#endif /* QUESTION3_ONLY_H */
