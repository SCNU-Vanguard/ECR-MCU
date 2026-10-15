/**
 * @file normal_filter.h
 * @author guatai (2508588132@qq.com)
 * @brief 定长滑动平均与中值滤波。
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __NORMAL_FILTER_H__
#define __NORMAL_FILTER_H__

#include <stdint.h>
#include <stdbool.h>

/*-------------------------普通滤波器---------------------------*/
/*-----------------------------------------------------------*/
#define MAF_MAXSIZE 100
#define MIDF_MAXSIZE 100

//滑动平均滤波器
typedef struct
{
  float num[MAF_MAXSIZE];
  uint8_t lenth;
  uint8_t pot; //当前位置
  float total_value;
  float aver_value;

} moving_average_filter_t;	//最大设置MAF_MaxSize个

//索引数组负责记录数据进入数据窗口时pot是第几号
//提供删除数据的索引
//中值滤波器
typedef struct
{
  float data_num[MIDF_MAXSIZE];	//数值数组
  int data_index_num[MIDF_MAXSIZE];
  uint8_t lenth;

  uint8_t index_pot;	//始终指向下一个要删除的pot
  float median_data;
} median_filter_t;	//最大设置MAF_MaxSize个

extern moving_average_filter_t KEY_W, KEY_A, KEY_S, KEY_D;
extern moving_average_filter_t MOUSE_X, MOUSE_Y;

/* 以下接口均要求非空、独占的已初始化实例；输入输出单位相同，不处理 NaN。
 * 初始化/清空时以零填满窗口，启动阶段的零样本也参与均值和中值。 */
void Average_Add(moving_average_filter_t *Aver, float add_data);
/** pre=0 为最新样本，按窗口长度循环回溯；并非返回 aver_value。 */
float Average_Get(moving_average_filter_t *Aver, uint16_t pre);
/** lenth 须 >= 1，大于 MAF_MAXSIZE 时截断；零长度不能用于 Average_Add/Get。 */
void Average_Init(moving_average_filter_t *Aver, uint8_t lenth);
void Average_Clear(moving_average_filter_t *Aver);
void Average_Fill(moving_average_filter_t *Aver, float temp);	//往滑动滤波填充某个值

/** 更新 median_data；偶数窗口取排序后较大的中央值，零长度时忽略。 */
void Median_Add(median_filter_t *Median, float add_data);
/** pre=0 为最新样本，按窗口循环回溯；获取中值请读取 median_data。零长度返回 0。 */
float Median_Get(median_filter_t *Median, uint16_t pre);
/** lenth 推荐 1～MIDF_MAXSIZE，超出上限会截断；实例存储由调用者持有。 */
void Median_Init(median_filter_t *Median, uint8_t lenth);
void Median_Clear(median_filter_t *Median);

#endif /* __NORMAL_FILTER_H__ */
