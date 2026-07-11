#ifndef QUESTION1_ONLY_H
#define QUESTION1_ONLY_H

#include "app_task_ids.h"

static void run_question1_only_test(void)
{
    prepare_task_start(TASK_ID_1);
    run_task1_ab();
}

#endif /* QUESTION1_ONLY_H */
