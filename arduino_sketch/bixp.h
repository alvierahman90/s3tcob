#ifndef BIXP_H
#define BIXP_H

#define BIXP_CTRL_BEANS 0x00
#define BIXP_CTRL_BANANAS 0x01
#define BIXP_CTRL_ESCAPE 0x02
#define BIXP_CTRL_REQ_SEND_BEANS 0x03
#define BIXP_CTRL_SURE_BEANS 0x04
#define BIXP_CTRL_BEGIN_BEANS 0x05
#define BIXP_CTRL_END_BEANS 0x06
#define BIXP_CTRL_COOL_BEANS 0x07
#define BIXP_CTRL_BEGIN_CHECKSUM 0x08
#define BIXP_CTRL_END_CHECKSUM 0x09
#define BIXP_CTRL_NO_BEANS 0x0a
#define BIXP_CTRL_WRONG_BEANS 0x0b

#define BIXP_STATE_DEFAULT 0
#define BIXP_STATE_RECV_BEANS (1 << 1)
#define BIXP_STATE_RECV_CHECKSUM (1 << 2)
#define BIXP_STATE_ESCAPE (1 << 3)

#define BIXP_HANDLE_SERIAL_RESPONSE 1 // argument *response has been set, should be sent in response
#define BIXP_HANDLE_SERIAL_BANANAS (1 << 1) // BANANAS control sequence received
#define BIXP_HANDLE_SERIAL_UNHANDLED (1 << 2) // Function could not process transmission
#define BIXP_HANDLE_SERIAL_BEANS_READY (1 << 3) // Buf is ready to be read (transmission ended successfully)
#define BIXP_HANDLE_SERIAL_NO_BEANS (1 << 4) // Transmission failed
#define BIXP_HANDLE_SERIAL_CKSUM_FAILED (1 << 5) // Checksum validation failed

unsigned int bixp_handle_serial(char *buf, unsigned int buf_maxlen, unsigned int *buf_len, char *cksum, unsigned int cksum_maxlen, unsigned int *cksum_len, char *state, char next_byte, char *response);

#endif
