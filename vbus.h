#include <stdint.h>

struct vbus_pkt 
{
	uint16_t	dst;
	uint16_t	src;
	uint16_t	cmd;
	uint8_t		proto;
	uint8_t		frames;
	uint8_t		aux_data[];
};

void *vbus_init(const char *vbus_fn, bool do_stty,
    void (*filter)(void *, struct vbus_pkt *), void *argp);
void *vbus_join(void *);
