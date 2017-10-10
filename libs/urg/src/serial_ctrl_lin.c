/*!
  \file
  \brief �V���A���ʐM (Linux, Mac ����)

  Serial Communication Interface ����


  \author Satofumi KAMIMURA

  $Id: serial_ctrl_lin.c 1559 2009-12-01 13:13:08Z satofumi $
*/

#include "serial_errno.h"
#include "ring_buffer.h"

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#if defined WINDOWS_OS
#include <time.h>
#else
#include <sys/time.h>
#include <sys/select.h>
#include <sys/types.h>
#endif

//#include <ctype.h>

enum {
  False = 0,
  True,
};


enum {
  InvalidFd = -1,
};


void serial_initialize(serial_t *serial)
{
  serial->fd_ = InvalidFd;
  serial->errno_ = SerialNoError;
  serial->has_last_ch_ = False;

  ring_initialize(&serial->ring_, serial->buffer_, RingBufferSizeShift);
}


/* �ڑ� */
int serial_connect(serial_t *serial, const char *device, long baudrate)
{
  int flags = 0;
  int ret = 0;

  serial_initialize(serial);

#ifndef MAC_OS
  enum { O_EXLOCK = 0x0 }; /* Linux �ł͎g���Ȃ��̂Ń_�~�[��쐬���Ă��� */
#endif
  serial->fd_ = open(device, O_RDWR | O_EXLOCK | O_NONBLOCK | O_NOCTTY);
  if (serial->fd_ < 0) {
    /* �ڑ��Ɏ��s */
    strerror_r(errno, serial->error_string_, SerialErrorStringSize);
    return SerialConnectionFail;
  }

  flags = fcntl(serial->fd_, F_GETFL, 0);
  fcntl(serial->fd_, F_SETFL, flags & ~O_NONBLOCK);

  /* �V���A���ʐM�̏����� */
  tcgetattr(serial->fd_, &serial->sio_);
  serial->sio_.c_iflag = 0;
  serial->sio_.c_oflag = 0;
  serial->sio_.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
  serial->sio_.c_cflag |= CS8 | CREAD | CLOCAL;
  serial->sio_.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN);

  serial->sio_.c_cc[VMIN] = 0;
  serial->sio_.c_cc[VTIME] = 0;

  /* �{�[���[�g�̕ύX */
  ret = serial_setBaudrate(serial, baudrate);
  if (ret < 0) {
    return ret;
  }

  /* �V���A������\���̂̏����� */
  serial->has_last_ch_ = False;

  return 0;
}


/* �ؒf */
void serial_disconnect(serial_t *serial)
{
  if (serial->fd_ >= 0) {
    close(serial->fd_);
    serial->fd_ = InvalidFd;
  }
}


int serial_isConnected(const serial_t *serial)
{
  return ((serial == NULL) || (serial->fd_ == InvalidFd)) ? 0 : 1;
}


/* �{�[���[�g�̐ݒ� */
int serial_setBaudrate(serial_t *serial, long baudrate)
{
  long baudrate_value = -1;

  switch (baudrate) {
  case 4800:
    baudrate_value = B4800;
    break;

  case 9600:
    baudrate_value = B9600;
    break;

  case 19200:
    baudrate_value = B19200;
    break;

  case 38400:
    baudrate_value = B38400;
    break;

  case 57600:
    baudrate_value = B57600;
    break;

  case 115200:
    baudrate_value = B115200;
    break;

  default:
    return SerialSetBaudrateFail;
  }

  /* �{�[���[�g�ύX */
  cfsetospeed(&serial->sio_, baudrate_value);
  cfsetispeed(&serial->sio_, baudrate_value);
  tcsetattr(serial->fd_, TCSADRAIN, &serial->sio_);
  serial_clear(serial);

  return 0;
}


/* ���M */
int serial_send(serial_t *serial, const char *data, int data_size)
{
  if (! serial_isConnected(serial)) {
    return SerialConnectionFail;
  }
  return write(serial->fd_, data, data_size);
}


static int waitReceive(serial_t* serial, int timeout)
{
  fd_set rfds;
  struct timeval tv;

  // �^�C���A�E�g�ݒ�
  FD_ZERO(&rfds);
  FD_SET(serial->fd_, &rfds);

  tv.tv_sec = timeout / 1000;
  tv.tv_usec = (timeout % 1000) * 1000;

  if (select(serial->fd_ + 1, &rfds, NULL, NULL,
             (timeout < 0) ? NULL : &tv) <= 0) {
    /* �^�C���A�E�g���� */
    return 0;
  }
  return 1;
}


static int internal_receive(char data[], int data_size_max,
                            serial_t* serial, int timeout)
{
  int filled = 0;

  if (data_size_max <= 0) {
    return 0;
  }

  while (filled < data_size_max) {
    int require_n;
    int read_n;

    if (! waitReceive(serial, timeout)) {
      break;
    }

    require_n = data_size_max - filled;
    read_n = read(serial->fd_, &data[filled], require_n);
    if (read_n <= 0) {
      /* �ǂݏo���G���[�B���݂܂ł̎�M��e�Ŗ߂� */
      break;
    }
    filled += read_n;
  }
  return filled;
}


/* ��M */
int serial_recv(serial_t *serial, char* data, int data_size_max, int timeout)
{
  int filled;
  int read_n;
  int buffer_size;

  if (data_size_max <= 0) {
    return 0;
  }

  /* �����߂����P������� ��΁A�����o�� */
  filled = 0;
  if (serial->has_last_ch_ != False) {
    data[0] = serial->last_ch_;
    serial->has_last_ch_ = False;
    ++filled;
  }

  if (! serial_isConnected(serial)) {
    if (filled > 0) {
      return filled;
    }
    return SerialConnectionFail;
  }

  buffer_size = ring_size(&serial->ring_);
  read_n = data_size_max - filled;
  if (buffer_size < read_n) {
    // �����O�o�b�t�@��̃f�[�^�ő���Ȃ���΁A�f�[�^��ǂݑ���
    char buffer[RingBufferSize];
    int n = internal_receive(buffer,
                             ring_capacity(&serial->ring_) - buffer_size,
                             serial, 0);
    ring_write(&serial->ring_, buffer, n);
  }
  buffer_size = ring_size(&serial->ring_);

  // �����O�o�b�t�@��̃f�[�^��Ԃ�
  if (read_n > buffer_size) {
    read_n = buffer_size;
  }
  if (read_n > 0) {
    ring_read(&serial->ring_, &data[filled], read_n);
    filled += read_n;
  }

  // �f�[�^��^�C���A�E�g�t���œǂݏo��
  filled += internal_receive(&data[filled],
                             data_size_max - filled, serial, timeout);
  return filled;
}


/* �P���������߂� */
void serial_ungetc(serial_t *serial, char ch)
{
  serial->has_last_ch_ = True;
  serial->last_ch_ = ch;
}


void serial_clear(serial_t* serial)
{
  tcdrain(serial->fd_);
  tcflush(serial->fd_, TCIOFLUSH);
  ring_clear(&serial->ring_);
  serial->has_last_ch_ = False;
}
