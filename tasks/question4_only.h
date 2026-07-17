#ifndef QUESTION4_ONLY_H
#define QUESTION4_ONLY_H

#include "app_task_ids.h"

/* 第四问专用单任务测试入口，不复用或改写前三问的参数选择。 */
static void run_question4_only_test(void)
{
    prepare_task_start(TASK_ID_4);
    run_task4_laps();
}

#endif /* QUESTION4_ONLY_H */
