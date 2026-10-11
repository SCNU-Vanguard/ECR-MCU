#ifndef CDD_BMI088_H__
#define CDD_BMI088_H__
#include "component/common/types.h"

typedef struct { int16_t acc[3]; int16_t gyro[3]; } bmi088_raw_t;   /* 原始 LSB 值，XYZ */

bool    bmi088_present(void);                    /* 初始化时两颗芯片 ID 都对才为 true */
int32_t bmi088_read_raw(bmi088_raw_t *out);      /* 任务里调用 */
#endif
