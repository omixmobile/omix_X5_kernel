// Antaiui <AI_BSP_FLASHLIGHT> <xieht> <2021-06-03> side torch porting begin

#ifndef __AI_SIDE_TORCH_H__
#define __AI_SIDE_TORCH_H__

extern int get_torch_level(void);
extern int set_side_torch(int level);

extern int side_torch_init(struct device *dev);
extern int side_torch_exit(struct device *dev);

#endif
// Antaiui <AI_BSP_FLASHLIGHT> <xieht> <2021-06-03> side torch porting end