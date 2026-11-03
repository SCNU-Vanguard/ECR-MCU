#ifndef INTERBOARD_CTRL_H__
#define INTERBOARD_CTRL_H__
#include "interboard_config.h"

/* 最近一帧云台请求。离线或还没收到时返回 false，不写 request。 */
bool interboard_ctrl_get(interboard_request_t *request);
#endif
