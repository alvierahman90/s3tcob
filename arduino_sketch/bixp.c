#include "bixp.h"

#define CHECKSUM_OK 0
#define CHECKSUM_NOT_OK 1


int add_to_buffer(char buf[], unsigned int buf_maxlen, unsigned int *buf_len, char next_byte) {
  if (*buf_len >= buf_maxlen) {
    return 1;
  }

  buf[*buf_len++] = next_byte;
}

int check_checksum(char *checksum, int len) {
  return CHECKSUM_OK;
}

unsigned int bixp_handle_serial(char *buf, unsigned int buf_maxlen, unsigned int *buf_len, char *cksum, unsigned int cksum_maxlen, unsigned int *cksum_len, char *state, char next_byte, char *response) {
  int rc;
  
  if (*state && BIXP_STATE_ESCAPE) {
    if (*state && BIXP_STATE_RECV_BEANS) rc = add_to_buffer(buf, buf_maxlen, buf_len, next_byte);
    else if (*state && BIXP_STATE_RECV_CHECKSUM) rc = add_to_buffer(cksum, cksum_maxlen, cksum_len, next_byte);
    else rc = 1;

    if (rc != 0) {
      *response = BIXP_CTRL_NO_BEANS;
      return BIXP_HANDLE_SERIAL_RESPONSE | BIXP_HANDLE_SERIAL_NO_BEANS;
    }

    *state = *state && (~BIXP_STATE_ESCAPE);
    
    return 0;
  }

  switch (next_byte) {
    case BIXP_CTRL_BEANS:
      *response = BIXP_CTRL_BANANAS;
      return BIXP_HANDLE_SERIAL_RESPONSE;
      
    case BIXP_CTRL_BANANAS:
      return BIXP_HANDLE_SERIAL_BANANAS;
      
    case BIXP_CTRL_ESCAPE:
      *state = *state | BIXP_CTRL_ESCAPE;
      return 0;
      
    case BIXP_CTRL_REQ_SEND_BEANS:
      rc = 0;
      if (*state & (BIXP_STATE_RECV_BEANS | BIXP_STATE_RECV_CHECKSUM)) {
        *response = BIXP_CTRL_NO_BEANS;
        *state = BIXP_STATE_DEFAULT;
        rc |= BIXP_HANDLE_SERIAL_NO_BEANS;
      } else {
        *response = BIXP_CTRL_SURE_BEANS;
      }
      
      return rc | BIXP_HANDLE_SERIAL_RESPONSE;
      
    case BIXP_CTRL_BEGIN_BEANS:
      if (*state & (BIXP_STATE_RECV_BEANS | BIXP_STATE_RECV_CHECKSUM)) {
        *response = BIXP_CTRL_NO_BEANS;
        *state = BIXP_STATE_DEFAULT;
        return BIXP_HANDLE_SERIAL_RESPONSE | BIXP_HANDLE_SERIAL_NO_BEANS;
      }
      
      *state = *state | BIXP_STATE_RECV_BEANS;
      *buf_len = 0;
      
      return 0;
      
    case BIXP_CTRL_END_BEANS:
      if(!(*state & BIXP_STATE_RECV_BEANS)) {
        *response = BIXP_CTRL_NO_BEANS;
        *state = BIXP_STATE_DEFAULT;
        return BIXP_HANDLE_SERIAL_RESPONSE | BIXP_HANDLE_SERIAL_NO_BEANS;
      }

      *state = *state & (~BIXP_STATE_RECV_BEANS);
      *response = BIXP_CTRL_COOL_BEANS;
      
      return BIXP_HANDLE_SERIAL_RESPONSE;
      
    case BIXP_CTRL_BEGIN_CHECKSUM:
      if (*state && (BIXP_STATE_RECV_BEANS | BIXP_STATE_RECV_CHECKSUM)) {
        *response = BIXP_CTRL_NO_BEANS;
        *state = BIXP_STATE_DEFAULT;
        return BIXP_HANDLE_SERIAL_RESPONSE | BIXP_HANDLE_SERIAL_NO_BEANS;
      }

      *state = *state | BIXP_STATE_RECV_CHECKSUM;
      *cksum_len = 0;
      
      return 0;

    case BIXP_CTRL_END_CHECKSUM:
      if(!(*state && BIXP_STATE_RECV_CHECKSUM)) {
        *response = BIXP_CTRL_NO_BEANS;
        return BIXP_HANDLE_SERIAL_RESPONSE | BIXP_HANDLE_SERIAL_NO_BEANS;
      }

      *state = *state & (~BIXP_STATE_RECV_CHECKSUM);

      if (check_checksum(cksum, cksum_len) == CHECKSUM_OK) {
        *response = BIXP_CTRL_COOL_BEANS;
      } else {
        *response = BIXP_CTRL_WRONG_BEANS;
      }

      *state = BIXP_STATE_DEFAULT;
            
      return BIXP_HANDLE_SERIAL_RESPONSE | BIXP_HANDLE_SERIAL_BEANS_READY;
      
    default:
      if (*state & BIXP_STATE_RECV_BEANS) {
        rc = add_to_buffer(buf, buf_maxlen, buf_len, next_byte);

        if (rc == 0) return 0;
      
        *response = BIXP_CTRL_NO_BEANS;
        *state = BIXP_STATE_DEFAULT;
        
        return BIXP_HANDLE_SERIAL_RESPONSE | BIXP_HANDLE_SERIAL_NO_BEANS;
      }
      
      if (*state & BIXP_STATE_RECV_CHECKSUM) {
        rc = add_to_buffer(cksum, cksum_maxlen, cksum_len, next_byte);

        if (rc == 0) return 0;

        *response = BIXP_CTRL_NO_BEANS;
        *state = BIXP_STATE_DEFAULT;

        return BIXP_HANDLE_SERIAL_RESPONSE | BIXP_HANDLE_SERIAL_NO_BEANS;
      }
    
      *response = BIXP_CTRL_NO_BEANS;
      *state = BIXP_STATE_DEFAULT;
      
      return BIXP_HANDLE_SERIAL_UNHANDLED | BIXP_HANDLE_SERIAL_RESPONSE;
  }

  return 0;
}
