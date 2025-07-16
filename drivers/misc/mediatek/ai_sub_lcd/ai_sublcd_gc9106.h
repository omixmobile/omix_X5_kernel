

#ifndef __AI_SIDE_TORCH_H__
#define __AI_SIDE_TORCH_H__

extern void lcm_spi_interface(unsigned char *sbuf, size_t spilen, unsigned char mode);

struct gc9106_send_data {
	unsigned char type;
	size_t num;
	unsigned char data[16];
};

 typedef enum
{
	WRITE_CMD=0,
	WRITE_DATA=1,
	READ_DATA=2,
	CMD_DELAY=3
}GC9106_OP_TYPE;

static struct gc9106_send_data ai_gc9106_init_parm[] = {

    {WRITE_CMD, 1, {0xFE}},			
    {WRITE_CMD, 1, {0xEF}},
    {WRITE_CMD, 1, {0xB0}},			
    {WRITE_DATA, 1, {0xC0}}, 
    {WRITE_CMD, 1, {0xB2}},		
    {WRITE_DATA, 1, {0x24}}, 
    {WRITE_CMD, 1, {0xB3}},		
    {WRITE_DATA, 1, {0x03}},
    {WRITE_CMD, 1, {0xB7}},		
    {WRITE_DATA, 1, {0x01}},  
    {WRITE_CMD, 1, {0xB6}},			
    {WRITE_DATA, 1, {0x19}}, 
    {WRITE_CMD, 1, {0xAC}},
    {WRITE_DATA, 1, {0xDB}},
    {WRITE_CMD, 1, {0xAB}},
    {WRITE_DATA, 1, {0x0f}},
    {WRITE_CMD, 1, {0x3A}},		
    {WRITE_DATA, 1, {0x05}}, 
    {WRITE_CMD, 1, {0xB4}},	
    {WRITE_DATA, 1, {0x04}},
    {WRITE_CMD, 1, {0xA8}},	
    {WRITE_DATA, 1, {0x0C}},
    {WRITE_CMD, 1, {0xb8}},
    {WRITE_DATA, 1, {0x08}},
    {WRITE_CMD, 1, {0xED}},	
    {WRITE_DATA, 1, {0x03}},
    {WRITE_CMD, 1, {0xea}},
    {WRITE_DATA, 1, {0x9f}}, 
    {WRITE_CMD, 1, {0xc6}},
    {WRITE_DATA, 1, {0x2a}}, 
    {WRITE_CMD, 1, {0xc7}},	
    {WRITE_DATA, 1, {0x10}},
    
    {WRITE_CMD, 1, {0xF0}},
    {WRITE_DATA, 14,{0x09,0x35,0x2a,0x4a,0xA9,0x39,0x35,0x60,0x00,0x14,0x0a,0x16,0x10,0x1F}},
    {WRITE_CMD, 1, {0xF1}},
    {WRITE_DATA, 14,{0x13,0x24,0x55,0x3c,0xc6,0x16,0x3f,0x60,0x08,0x06,0x0d,0x1f,0x1c,0x10}},
    
    {CMD_DELAY,120,{0x0}},  //delay 120ms
	{WRITE_CMD, 1,{0x11}},
	
	{CMD_DELAY,120,{0x0}},  //delay 120ms
	{WRITE_CMD, 1,{0x29}},
	
	{WRITE_CMD, 1,{0x2A}},
	{WRITE_DATA,4,{0x00,0x18,0x00,0x67}},   // Xstart -- Xend
	
	{WRITE_CMD, 1,{0x2c}},
};

static struct gc9106_send_data ai_gc9106_suspend_parm[] = {
	{WRITE_CMD, 1,{0xfe}},
	{WRITE_CMD, 1,{0xef}},
	{WRITE_CMD, 1,{0x28}},
	{CMD_DELAY,120,{0x0}},  //delay 120ms
	{WRITE_CMD, 1,{0x10}},
	{CMD_DELAY,50,{0x0}},  //delay 50ms
};
static struct gc9106_send_data ai_gc9106_resume_parm[] = {
	{WRITE_CMD, 1,{0xfe}},
	{WRITE_CMD, 1,{0xef}},
	{WRITE_CMD, 1,{0x11}},
	{CMD_DELAY,120,{0x0}},  //delay 120ms
	{WRITE_CMD, 1,{0x29}},
	
	{WRITE_CMD, 1,{0x2A}},
	{WRITE_DATA,4,{0x00,0x18,0x00,0x67}},   // Xstart -- Xend
	
	{WRITE_CMD, 1,{0x2c}},
};
#endif
