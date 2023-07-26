#ifndef PATH_H_
#define PATH_H_

#define PATH_MAX0 527 //最大路径数  
#define PATH_MAX1 520
#define PATH_MAX2 155
#define PATH_MAX3 1
#define PATH_MAX4 1
#define PATH_MAX5 1334
/************************* Path *************************/
//导航的路径信息
typedef struct
{
    float x;   //x坐标
    float y;    //y坐标
    float V_Theta;  //速度方向
    float alpha;    //期望位姿
    float delta;  // ？？
    float v;   //速度
    uint8_t preview;   //超前滞后
} NavigationPoints; 

//所有路径的数组
typedef struct
{
    const NavigationPoints *PathDotsInfoArray; //point to the inception of pathI[PathDotsNum]
    uint16_t PathDotsNum;
} PathInfoTypedef;

extern const NavigationPoints path0[PATH_MAX0];
extern const NavigationPoints path1[PATH_MAX1];
extern const NavigationPoints path2[PATH_MAX2];
extern const NavigationPoints path3[PATH_MAX3];
extern const NavigationPoints path4[PATH_MAX4];
extern const NavigationPoints path5[PATH_MAX5];
#endif
