#ifndef __DBUS_H
#define __DBUS_H

/***********RC***********/
typedef __packed struct
{
	uint16_t ch0;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint8_t s1;
	uint8_t s2;
}Remote;
typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;
typedef __packed struct
{
		uint16_t w_,s_,a_,d_,q_,e_,r_,f_,g_,z_,x_,c_,v_,b_,shift_,ctrl_;
}Key;

typedef struct
{
	Remote rc;
	Mouse mouse;
	Key key;
    int8_t RCrecvd,RCDisconnectCnt;
	uint8_t *rx_buffer;
}RC_Ctl_t;



void RemoteReceive(volatile uint8_t * const ptr_sbus_rx_buffer);
void reset_remote(void);
RC_Ctl_t getRCData(void);

#endif // __DBUS_H
