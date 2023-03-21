#include<stdio.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#include "mainMujoco.h"
#ifdef __cplusplus
}
#endif /* __cplusplus */

int main(int argc, const char** argv)
{
    printf("Mujoco started.\n");
    int ret = mainMujoco(argc, argv);
    return ret;
}
