/**
******************************************************************************
 * @file    normal_filter.c
 * @brief 定长滑动平均与中值滤波。
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include "component/filter/normal/normal_filter.h"

/*-------------------------普通滤波器---------------------------*/
/*-----------------------------------------------------------*/

moving_average_filter_t KEY_W, KEY_A, KEY_S, KEY_D;
moving_average_filter_t MOUSE_X, MOUSE_Y;

/**
 * @brief    Average_Init
 * @note    滑动滤波器初始化，设置长度
 * @param  None
 * @retval None
 * @author  RobotPilots
 */
void Average_Init(moving_average_filter_t *Aver, uint8_t lenth)
{
  uint16_t i;

  for (i = 0; i < MAF_MAXSIZE; i++)
    Aver->num[i] = 0;

  if (lenth > MAF_MAXSIZE)
  {
    lenth = MAF_MAXSIZE;
  }

  Aver->lenth = lenth;
  Aver->pot = 0;
  Aver->aver_value = 0;
  Aver->total_value = 0;
}

/**
 * @brief    Average_Add
 * @note    滑动平均滤波器进入队列，先进先出
 * @param  None
 * @retval None
 * @author  RobotPilots
 */
void Average_Add(moving_average_filter_t *Aver, float add_data)
{

  Aver->total_value -= Aver->num[Aver->pot];
  Aver->total_value += add_data;

  Aver->num[Aver->pot] = add_data;

  Aver->aver_value = ((Aver->total_value) / (Aver->lenth));
  Aver->pot++;

  if (Aver->pot == Aver->lenth)
  {
    Aver->pot = 0;
  }
}

/**
 * @brief    Average_Get
 * @note    获取第前pre次的数据，如果超出数组长度则取记录的最早的数据
 * @param  None
 * @retval None
 * @author  RobotPilots
 */
float Average_Get(moving_average_filter_t *Aver, uint16_t pre)
{
  float member;
  uint8_t temp;

  if (Aver->pot != 0)
  {
    temp = Aver->pot - 1;
  }
  else
  {
    temp = Aver->lenth - 1;
  }

  if (pre > Aver->lenth)
    pre = pre % Aver->lenth;

  if (pre > temp)
  {
    pre = Aver->lenth + temp - pre;
  }
  else
  {
    pre = temp - pre;
  }

  member = Aver->num[pre];

  return member;
}

/**
 * @brief    Average_Clear
 * @note    滑动滤波器清空
 * @param  None
 * @retval None
 * @author  RobotPilots
 */
void Average_Clear(moving_average_filter_t *Aver)
{
  uint16_t i;

  for (i = 0; i < MAF_MAXSIZE; i++)
    Aver->num[i] = 0;

  Aver->pot = 0;
  Aver->aver_value = 0;
  Aver->total_value = 0;
}

/**
 * @brief    Average_Fill
 * @note    滑动滤波器填充某个值
 * @param  None
 * @retval None
 * @author  RobotPilots
 */
void Average_Fill(moving_average_filter_t *Aver, float temp)
{
  uint16_t i;

  for (i = 0; i < (Aver->lenth); i++)
    Aver->num[i] = temp;

  Aver->pot = 0;
  Aver->aver_value = temp;
  Aver->total_value = temp * (Aver->lenth);
}

//add_data的索引与index_pot绑定
void Median_Add(median_filter_t *Median, float add_data)
{
  if (Median->lenth <= 0)
  {
    return;
  }

  uint16_t pot = 0;

  //遍历寻找要删掉的数据,pot最终停留位置就是要删除的数据
  for (pot = 0; pot < Median->lenth; pot++)
  {
    if (Median->data_index_num[pot] == Median->index_pot)
    {
      break;
    }
  }

  /* index_pot 标识被淘汰的最旧样本；移动数值时一起移动年龄索引。
   * 先检查边界再读取邻项，长度为 1 和插入两端时也不访问窗口外。 */
  if (pot == Median->lenth) return;
  while (pot + 1U < Median->lenth && Median->data_num[pot + 1U] < add_data)
  {
    Median->data_num[pot] = Median->data_num[pot + 1U];
    Median->data_index_num[pot] = Median->data_index_num[pot + 1U];
    ++pot;
  }
  while (pot > 0U && Median->data_num[pot - 1U] > add_data)
  {
    Median->data_num[pot] = Median->data_num[pot - 1U];
    Median->data_index_num[pot] = Median->data_index_num[pot - 1U];
    --pot;
  }

  Median->data_num[pot] = add_data;
  Median->data_index_num[pot] = Median->index_pot;

  Median->index_pot++;
  if (Median->index_pot == Median->lenth)
  {
    Median->index_pot = 0;
  }

  Median->median_data = Median->data_num[Median->lenth / 2];
}

float Median_Get(median_filter_t *Median, uint16_t pre)
{
  if (Median->lenth == 0U) return 0.0f;
  /* 排序位置不等于时间顺序，按年龄索引查找历史样本。 */
  uint16_t age = pre % Median->lenth;
  uint16_t index = (Median->index_pot + Median->lenth - 1U - age) % Median->lenth;
  for (uint16_t i = 0U; i < Median->lenth; ++i)
    if (Median->data_index_num[i] == index) return Median->data_num[i];
  return 0.0f;
}

void Median_Init(median_filter_t *Median, uint8_t lenth)
{
  uint16_t i;

  for (i = 0; i < MIDF_MAXSIZE; i++)
  {
    Median->data_num[i] = 0;
    Median->data_index_num[i] = i;
  }

  if (lenth > MIDF_MAXSIZE)
  {
    lenth = MIDF_MAXSIZE;
  }

  Median->lenth = lenth;
  Median->index_pot = 0;
  Median->median_data = Median->data_num[Median->lenth / 2];
}

void Median_Clear(median_filter_t *Median)
{
  uint16_t i;

  for (i = 0; i < MIDF_MAXSIZE; i++)
  {
    Median->data_num[i] = 0;
    Median->data_index_num[i] = i;
  }

  Median->median_data = Median->data_num[Median->lenth / 2];
  Median->index_pot = 0;
}

/*-------------------------普通滤波器---------------------------*/
/*-----------------------------------------------------------*/
