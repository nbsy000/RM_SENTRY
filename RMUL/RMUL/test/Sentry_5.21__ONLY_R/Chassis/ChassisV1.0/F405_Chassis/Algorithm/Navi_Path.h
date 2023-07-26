#ifndef __NAVI_PATH_H
#define __NAVI_PATH_H

#include "main.h"

#define PATH_NUM 20   //最大路径数量

extern PathInfoTypedef PathInforms[PATH_NUM];
void PathInit(void);
void chassisNavigate(ChassisState_t *ChassisInfo, PathInfoTypedef *PathInfo);

#endif

