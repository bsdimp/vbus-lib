
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "vbus.h"

/*
 * Simple framekwork for reading packets from vbus.
 *
 * It's a rs-485-like situation: there's just 1 wires on the bus for
 * a differential signal. Only the master can talk, unless the slave
 * talks in response to a command from the master.
 *
 * pkt : SYNC HDR FRAME*
 *
 * SYNC: 0xaa
 *
 * HDR:	DST-LSB DST-MSB SRC-LSB SRC-MSB PROTO-REV CMD-LSB CMD-MSB FRAME-CNT CRC
 *
 * FRAME: BYTE0 BYTE1 BYTE2 BYTE3 MSB CRC
 *
 * There's FRAME-CNT frames following the header (not reflected in the above psuedo-grammar) 
 * CRC is sum of prior bytes, negated, with top bit cleared.
 * 0x80 is never set except for SYNC byte.
 * For the frame, MSB contains the most significant bits for each of the prior 4 bytes.
 *
 * To use this library, call vbus_init. This specifies a filter that we call for each
 * packet. This filter is called with NULL before we exit the thread. All calls are done
 * in the context of the created thread. All locking is the responsibility of the calling
 * program. vbus_init returns a cookie.
 *
 * vbus_join waits on the vbus cookie for the thread to complete.
 */

pthread_key_t vbus_key;

#define VBUS_SYNC	0xaa
#define VBUS_WIRE_HDR	9
#define VBUS_WIRE_FRAME	6
#define VBUS_FRAME_LEN	4

struct vbus_softc 
{
	int		fd;
	bool		do_stty;
	struct termios	termios;
	struct termios	raw_termios;
	void		(*filter)(void *, struct vbus_pkt *);
	void		*argp;
	pthread_t	td;
};

static void
vbus_cleanup(struct vbus_softc *sc)
{

	if (sc != NULL) {
		if (sc->do_stty)
			tcsetattr(sc->fd, TCSANOW, &sc->termios);
		if (sc->fd != -1)
			close(sc->fd);
		if (sc->filter)
			sc->filter(sc->argp, NULL);
	}
	free(sc);
}

static void
vbus_dtor(void *xsc)
{
	struct vbus_softc *sc = xsc;
	
	vbus_cleanup(sc);
}

static void
vbus_fini()
{

	pthread_exit(NULL);
}

static uint8_t
vbus_getc(struct vbus_softc *sc)
{
	uint8_t ch;

	if (read(sc->fd, &ch, 1) != 1)
		vbus_fini();

	return ch;
}

static bool
vbus_crc_ok(uint8_t *buffer, size_t len)
{
	uint8_t sum = 0;

	for (int i = 0; i < len - 1; i++)
		sum += buffer[i];
	sum = ~sum & 0x7f;
	if (sum != buffer[len - 1])
		return false;
	return true;
}

static struct vbus_pkt *
vbus_parse_header(uint8_t *buffer) 
{
	uint8_t sum;
	struct vbus_pkt *pkt;

	if (!vbus_crc_ok(buffer, VBUS_WIRE_HDR))
		return NULL;
	pkt = malloc(sizeof(struct vbus_pkt) + 4 * buffer[7]);
	if (pkt == NULL)
		return NULL;
	pkt->dst = buffer[0] | ((uint16_t)buffer[1] << 8);
	pkt->src = buffer[2] | ((uint16_t)buffer[3] << 8);
	pkt->proto = buffer[4];
	pkt->cmd = buffer[5] | ((uint16_t)buffer[6] << 8);
	pkt->frames = buffer[7];

	return pkt;
}

static int
vbus_read_frame(struct vbus_softc *sc, uint8_t *frame)
{
	uint8_t buffer[VBUS_WIRE_FRAME];
	int i;

	for (i = 0; i < VBUS_WIRE_FRAME; i++) {
		buffer[i] = vbus_getc(sc);
		if (buffer[i] & 0x80)
			return -1;
	}
	if (!vbus_crc_ok(buffer, sizeof(buffer)))
		return -1;
	frame[0] = buffer[0] | (buffer[4] & 1) ? 0x80 : 0;
	frame[1] = buffer[1] | (buffer[4] & 2) ? 0x80 : 0;
	frame[2] = buffer[2] | (buffer[4] & 4) ? 0x80 : 0;
	frame[3] = buffer[3] | (buffer[4] & 8) ? 0x80 : 0;

	return (0);
}

static void *
vbus_thread(void *xsc)
{
	struct vbus_softc *sc = xsc;
	struct vbus_pkt *pkt;
	uint8_t ch, buffer[VBUS_WIRE_HDR];
	int i;

	if (pthread_setspecific(vbus_key, sc)) {
		vbus_cleanup(sc);
		pthread_exit((void *)(uintptr_t)errno);
	}

	while (true) {
	top:;
		do {
			ch = vbus_getc(sc);
		} while (ch != VBUS_SYNC);
		for (i = 0; i < VBUS_WIRE_HDR; i++) {
			buffer[i] = vbus_getc(sc);
			if (buffer[i] & 0x80)
				goto top;
		}
		pkt = vbus_parse_header(buffer);
		if (pkt == NULL)
			continue;
		for (i = 0; i < pkt->frames; i++) {
			if (vbus_read_frame(sc, pkt->aux_data + i * VBUS_FRAME_LEN))
				goto top;
		}
		sc->filter(sc->argp, pkt);
		free(pkt);
	}
}

void *
vbus_init(
	const char *vbus_fn,				// filename to read from. - == stdin
	bool do_stty,					// set the tty modes to raw, 9600, etc? 
	void (*filter)(void *, struct vbus_pkt *),	// per-packet function to call (pkt NULL fini)
	void *argp)					// user supplied arg
{
	struct vbus_softc *sc = NULL;
	
	// XXX once?
	if (vbus_key == 0 && pthread_key_create(&vbus_key, vbus_dtor))
		goto err;
	sc = malloc(sizeof(struct vbus_softc));
	if (sc == NULL)
		goto err;
	memset(sc, 0, sizeof(*sc));
	sc->filter = filter;
	sc->argp = argp;
	if (vbus_fn != NULL || strcmp(vbus_fn, "-") != 0)
		sc->fd = open(vbus_fn, O_RDWR);
	/* else sc->fd == 0 or stdin which is useful for testing */
	if (sc->fd == -1)
		goto err;
	if (do_stty) {
		if (tcgetattr(sc->fd, &sc->termios))
			goto err;
		sc->raw_termios = sc->termios;
		cfmakeraw(&sc->raw_termios);
		cfsetspeed(&sc->raw_termios, B9600);
		if (tcsetattr(sc->fd, TCSANOW, &sc->raw_termios))
			goto err;
		sc->do_stty = do_stty;
	}
	if (pthread_create(&sc->td, NULL, vbus_thread, sc))
		goto err;
	return sc;
err:
	vbus_cleanup(sc);
	return NULL;
}

void *
vbus_join(void *xsc)
{
	struct vbus_softc *sc = xsc;
	void *rv;
	
	pthread_join(sc->td, &rv);
	return rv;
}
