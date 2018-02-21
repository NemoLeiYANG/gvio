#include "gvio/gimbal/sbgc.hpp"

namespace gvio {

int set_interface_attribs(int fd, int speed, int parity) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    printf("error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK; // disable break processing
  tty.c_lflag = 0;        // no signaling chars, no echo,
  // no canonical processing
  tty.c_oflag = 0;     // no remapping, no delays
  tty.c_cc[VMIN] = 0;  // read doesn't block
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls,
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    printf("error %d from tcsetattr", errno);
    return -1;
  }

  return 0;
}

void set_blocking(int fd, int should_block) {
  struct termios tty;

  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    printf("error %d from tggetattr", errno);
    return;
  }

  tty.c_cc[VMIN] = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    printf("error %d setting term attributes", errno);
  }
}

void SBGCFrame::printFrame() {
  int i;

  // print header
  printf("[%d]: %c\n", 0, '>');
  printf("[%d]: %c\n", 1, this->cmd_id);
  printf("[%d]: %d\n", 2, this->data_size);
  printf("[%d]: %d\n", 3, this->header_checksum);

  // print body
  for (i = 4; i < (this->data_size + 4); i++) {
    printf("[%d]: %d\n", i, this->data[i]);
  }
  printf("[%d]: %d\n", i, this->data_checksum);
}

void SBGCFrame::buildHeader(uint8_t cmd_id, uint8_t data_size) {
  this->cmd_id = cmd_id;
  this->data_size = data_size;
  this->header_checksum = (this->cmd_id + this->data_size) % 256;
}

void SBGCFrame::buildDataChecksum() {
  this->data_checksum = 0x0;
  for (int i = 0; i < this->data_size; i++) {
    this->data_checksum += this->data[i];
  }
  this->data_checksum = this->data_checksum % 256;
}

void SBGCFrame::buildBody(uint8_t *data) {
  this->data = data;
  this->buildDataChecksum();
}

void SBGCFrame::buildFrame(int cmd_id, uint8_t *data, int data_size) {
  this->buildHeader((uint8_t) cmd_id, (uint8_t) data_size);
  this->buildBody(data);
}

void SBGCFrame::buildFrame(int cmd_id) {
  this->buildHeader((uint8_t) cmd_id, (uint8_t) 0);
  this->buildBody(NULL);
}

int SBGCFrame::parseHeader(uint8_t *data) {
  uint8_t expected_checksum;

  // Pre-check
  if (data[0] != '>') {
    return -1;
  }

  // parse header
  this->cmd_id = data[1];
  this->data_size = data[2];
  this->header_checksum = data[3];

  // check the header checksum
  expected_checksum = (this->cmd_id + this->data_size) % 256;
  if (this->header_checksum != expected_checksum) {
    return -1;
  }

  return 0;
}

int SBGCFrame::parseBody(uint8_t *data) {
  uint8_t i;
  uint8_t expected_checksum;

  // setup
  expected_checksum = 0x0;
  this->data = (uint8_t *) malloc(sizeof(uint8_t) * this->data_size);

  // parse body
  for (i = 0; i < this->data_size; i++) {
    this->data[i] = data[4 + i]; // +4 because header is 4 bytes
    expected_checksum += data[4 + i];
  }
  this->data_checksum = data[4 + i];

  // check the body checksum
  expected_checksum = expected_checksum % 256;
  if (this->data_checksum != expected_checksum) {
    // LOG_ERROR("Failed body checksum!");
    free(this->data);
    return -1;
  }

  return 0;
}

int SBGCFrame::parseFrame(uint8_t *data) {
  int retval;

  // Header
  retval = this->parseHeader(data);
  if (retval == -1) {
    // LOG_ERROR("Failed to parse header!");
    return -1;
  }

  // Body
  retval = this->parseBody(data);
  if (retval == -1) {
    // LOG_ERROR("Failed to parse body!");
    return -1;
  }

  return 0;
}

void SBGCRealtimeData::printData() {
  // ACCELEROMOETER AND GYROSCOPE
  printf("accelerometer: %.2f\t%.2f\t%.2f\n",
         this->accel(0),
         this->accel(1),
         this->accel(2));
  printf("gyroscope: %.2f\t%.2f\t%.2f\n",
         this->gyro(0),
         this->gyro(1),
         this->gyro(2));
  printf("\n");

  // ANGLES
  printf("camera_angles: %.2f\t%.2f\t%.2f\n",
         this->camera_angles(0),
         this->camera_angles(1),
         this->camera_angles(2));
  printf("frame_angles: %.2f\t%.2f\t%.2f\n",
         this->frame_angles(0),
         this->frame_angles(1),
         this->frame_angles(2));
  printf("rc_angles: %.2f\t%.2f\t%.2f\n",
         this->rc_angles(0),
         this->rc_angles(1),
         this->rc_angles(2));

  // MISC
  printf("cycle_time: %d\n", this->cycle_time);
  printf("i2c_error_count: %d\n", this->i2c_error_count);
  printf("system_error: %d\n", this->system_error);
  printf("battery_level: %d\n\n", this->battery_level);
}

SBGC::SBGC() {
  this->connected = false;

  this->port = "";
  this->serial = -1;

  this->board_version = 0;
  this->firmware_version = 0;
  this->debug_mode = 0;
  this->board_features = 0;
  this->connection_flags = 0;
}

SBGC::SBGC(const std::string &port) {
  this->connected = false;

  this->port = port;
  this->serial = -1;

  this->board_version = 0;
  this->firmware_version = 0;
  this->debug_mode = 0;
  this->board_features = 0;
  this->connection_flags = 0;
}

int SBGC::connect() {
  // Open serial port
  this->serial = open(this->port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (this->serial < 0) {
    return -1;
  }

  // Configure serial commnication
  set_interface_attribs(this->serial, B115200, 0);
  set_blocking(this->serial, 1);
  LOG_INFO("Connected to SBGC!");

  this->connected = true;
  return 0;
}

int SBGC::disconnect() {
  if (close(this->serial) != 0) {
    LOG_ERROR("Failed to disconnect from SBGC!");
    return -1;

  } else {
    LOG_INFO("Disconnect from SBGC!");
    return 0;
  }
}

int SBGC::sendFrame(const SBGCFrame &cmd) {
  // Check connection
  if (this->connected == false) {
    return -1;
  }

  // Check command data size
  int data_size_limit;
  data_size_limit = SBGC_CMD_MAX_BYTES - SBGC_CMD_PAYLOAD_BYTES;
  if (cmd.data_size >= data_size_limit) {
    return -1;
  }

  // Header
  ssize_t retval = 0;
  const uint8_t start = 0x3E; // ">" character
  retval += write(this->serial, &start, 1);
  retval += write(this->serial, &cmd.cmd_id, 1);
  retval += write(this->serial, &cmd.data_size, 1);

  // Body
  retval += write(this->serial, &cmd.header_checksum, 1);
  retval += write(this->serial, cmd.data, cmd.data_size);
  retval += write(this->serial, &cmd.data_checksum, 1);

  // Flush
  tcflush(this->serial, TCIOFLUSH); // very critical
  usleep(10 * 1000);
  if (retval != (5 + cmd.data_size)) {
    LOG_ERROR("Opps! frame wasn't sent completely!");
  }

  return 0;
}

int SBGC::readFrame(const uint8_t read_length, SBGCFrame &frame) {
  // Check connection
  if (this->connected == false) {
    return -1;
  }

  // Send query
  uint8_t buffer[150];
  int16_t nb_bytes = read(this->serial, buffer, read_length);
  if (nb_bytes <= 0 || nb_bytes != read_length) {
    // LOG_ERROR("Failed to read SBGC frame!");
    return -1;
  }

  // Parse sbgc frame
  int retval = frame.parseFrame(buffer);
  if (retval == -1) {
    // LOG_ERROR("Failed to parse SBGC frame!");
    return -1;
  }

  return 0;
}

int SBGC::on() {
  SBGCFrame cmd;
  cmd.buildFrame(CMD_MOTORS_ON);
  return this->sendFrame(cmd);
}

int SBGC::off() {
  // Check connection
  if (this->connected == false) {
    return -1;
  }

  // Turn off motor control
  uint8_t data[13];
  data[0] = MODE_NO_CONTROL;
  for (int i = 1; i < 13; i++) {
    data[i] = 0;
  }

  // Send frame
  SBGCFrame cmd;
  cmd.buildFrame(CMD_CONTROL, data, 13);
  int retval = this->sendFrame(cmd);
  if (retval != 0) {
    LOG_ERROR("Failed to turn motor control off!");
  }

  // Turn off motors
  cmd.buildFrame(CMD_MOTORS_OFF);
  retval = this->sendFrame(cmd);
  if (retval != 0) {
    LOG_ERROR("Failed to turn motor control off!");
  }

  return 0;
}

int SBGC::reset() {
  // Check connection
  if (this->connected == false) {
    return -1;
  }

  if (this->off() || this->on()) {
    LOG_ERROR("Failed to reset SBGC!");
    return -1;
  }

  return 0;
}

int SBGC::getBoardInfo() {
  int retval;
  SBGCFrame frame;

  // Request board info
  frame.buildFrame(CMD_BOARD_INFO);
  retval = this->sendFrame(frame);
  if (retval == -1) {
    LOG_ERROR("Failed to request SBGC board info!");
    return -1;
  }

  // Obtain board info
  retval = this->readFrame(CMD_BOARD_INFO_FRAME_SIZE, frame);
  if (retval == -1) {
    LOG_ERROR("Failed to parse SBGC frame for board info!");
    return -1;
  }

  // Set object board info
  this->board_version = frame.data[0];
  this->firmware_version = (frame.data[2] << 8) | (frame.data[1] & 0xff);
  this->debug_mode = frame.data[3];
  this->board_features = (frame.data[5] << 8) | (frame.data[4] & 0xff);
  this->connection_flags = frame.data[6];

  // Clean up
  free(frame.data);

  return 0;
}

int SBGC::getRealtimeData4() {
  int retval;

  // Request real time data
  SBGCFrame frame;
  frame.buildFrame(CMD_REALTIME_DATA_4);
  retval = this->sendFrame(frame);
  if (retval == -1) {
    // LOG_ERROR("Failed to request SBGC realtime data!");
    return -1;
  }

  // Obtain real time data
  retval = this->readFrame(129, frame);
  if (retval == -1) {
    // LOG_ERROR("Failed to parse SBGC frame for realtime data!" <<
    // std::endl;
    return -1;
  }

  // Parse real time data
  // Accelerometer and gyroscope
  this->data.accel(0) = S16BIT(frame.data, 1, 0);
  this->data.gyro(0) = S16BIT(frame.data, 3, 2);
  this->data.accel(1) = S16BIT(frame.data, 5, 4);
  this->data.gyro(1) = S16BIT(frame.data, 7, 6);
  this->data.accel(2) = S16BIT(frame.data, 9, 8);
  this->data.gyro(2) = S16BIT(frame.data, 11, 10);

  this->data.accel(0) = (ACC_UNIT) * this->data.accel(0);
  this->data.accel(1) = (ACC_UNIT) * this->data.accel(1);
  this->data.accel(2) = (ACC_UNIT) * this->data.accel(2);

  this->data.gyro(0) = (GYRO_UNIT) * this->data.gyro(0);
  this->data.gyro(1) = (GYRO_UNIT) * this->data.gyro(1);
  this->data.gyro(2) = (GYRO_UNIT) * this->data.gyro(2);

  // Angles
  this->data.camera_angles(0) = S16BIT(frame.data, 33, 32);
  this->data.camera_angles(1) = S16BIT(frame.data, 35, 34);
  this->data.camera_angles(2) = S16BIT(frame.data, 37, 36);

  this->data.frame_angles(0) = S16BIT(frame.data, 39, 38);
  this->data.frame_angles(1) = S16BIT(frame.data, 41, 40);
  this->data.frame_angles(2) = S16BIT(frame.data, 43, 42);

  this->data.rc_angles(0) = S16BIT(frame.data, 45, 44);
  this->data.rc_angles(1) = S16BIT(frame.data, 47, 45);
  this->data.rc_angles(2) = S16BIT(frame.data, 49, 46);

  this->data.encoder_angles(0) = S16BIT(frame.data, 64, 63);
  this->data.encoder_angles(1) = S16BIT(frame.data, 66, 65);
  this->data.encoder_angles(2) = S16BIT(frame.data, 68, 67);

  this->data.camera_angles(0) = (DEG_PER_BIT) * this->data.camera_angles(0);
  this->data.camera_angles(1) = (DEG_PER_BIT) * this->data.camera_angles(1);
  this->data.camera_angles(2) = (DEG_PER_BIT) * this->data.camera_angles(2);

  this->data.frame_angles(0) = (DEG_PER_BIT) * this->data.frame_angles(0);
  this->data.frame_angles(1) = (DEG_PER_BIT) * this->data.frame_angles(1);
  this->data.frame_angles(2) = (DEG_PER_BIT) * this->data.frame_angles(2);

  this->data.rc_angles(0) = (DEG_PER_BIT) * this->data.rc_angles(0);
  this->data.rc_angles(1) = (DEG_PER_BIT) * this->data.rc_angles(1);
  this->data.rc_angles(2) = (DEG_PER_BIT) * this->data.rc_angles(2);

  this->data.encoder_angles(0) = (DEG_PER_BIT) * this->data.encoder_angles(0);
  this->data.encoder_angles(1) = (DEG_PER_BIT) * this->data.encoder_angles(1);
  this->data.encoder_angles(2) = (DEG_PER_BIT) * this->data.encoder_angles(2);

  // Misc
  this->data.cycle_time = U16BIT(frame.data, 51, 50);
  this->data.i2c_error_count = U16BIT(frame.data, 53, 52);
  this->data.system_error = U16BIT(frame.data, 15, 14);
  this->data.battery_level = U16BIT(frame.data, 56, 55);

  // this->data.printData();

  return 0;
}
int SBGC::getRealtimeData() {
  int retval;
  SBGCFrame frame;

  // Request real time data
  frame.buildFrame(CMD_REALTIME_DATA_3);
  retval = this->sendFrame(frame);
  if (retval == -1) {
    LOG_ERROR("Failed to request SBGC realtime data!");
    return -1;
  }

  // Obtain real time data
  retval = this->readFrame(68, frame);
  if (retval == -1) {
    LOG_ERROR("Failed to parse SBGC frame for realtime data!");
    return -1;
  }

  // Parse real time data
  // Accelerometer and gyroscope
  this->data.accel(0) = S16BIT(frame.data, 1, 0);
  this->data.gyro(0) = S16BIT(frame.data, 3, 2);
  this->data.accel(1) = S16BIT(frame.data, 5, 4);
  this->data.gyro(1) = S16BIT(frame.data, 7, 6);
  this->data.accel(2) = S16BIT(frame.data, 9, 8);
  this->data.gyro(2) = S16BIT(frame.data, 11, 10);

  this->data.accel(0) = (ACC_UNIT) * this->data.accel(0);
  this->data.accel(1) = (ACC_UNIT) * this->data.accel(1);
  this->data.accel(2) = (ACC_UNIT) * this->data.accel(2);

  this->data.gyro(0) = (GYRO_UNIT) * this->data.gyro(0);
  this->data.gyro(1) = (GYRO_UNIT) * this->data.gyro(1);
  this->data.gyro(2) = (GYRO_UNIT) * this->data.gyro(2);

  // Angles
  this->data.camera_angles(0) = S16BIT(frame.data, 33, 32);
  this->data.camera_angles(1) = S16BIT(frame.data, 35, 34);
  this->data.camera_angles(2) = S16BIT(frame.data, 37, 36);

  this->data.frame_angles(0) = S16BIT(frame.data, 39, 38);
  this->data.frame_angles(1) = S16BIT(frame.data, 41, 40);
  this->data.frame_angles(2) = S16BIT(frame.data, 43, 42);

  this->data.rc_angles(0) = S16BIT(frame.data, 45, 44);
  this->data.rc_angles(1) = S16BIT(frame.data, 47, 45);
  this->data.rc_angles(2) = S16BIT(frame.data, 49, 46);

  this->data.camera_angles(0) = (DEG_PER_BIT) * this->data.camera_angles(0);
  this->data.camera_angles(1) = (DEG_PER_BIT) * this->data.camera_angles(1);
  this->data.camera_angles(2) = (DEG_PER_BIT) * this->data.camera_angles(2);

  this->data.frame_angles(0) = (DEG_PER_BIT) * this->data.frame_angles(0);
  this->data.frame_angles(1) = (DEG_PER_BIT) * this->data.frame_angles(1);
  this->data.frame_angles(2) = (DEG_PER_BIT) * this->data.frame_angles(2);

  this->data.rc_angles(0) = (DEG_PER_BIT) * this->data.rc_angles(0);
  this->data.rc_angles(1) = (DEG_PER_BIT) * this->data.rc_angles(1);
  this->data.rc_angles(2) = (DEG_PER_BIT) * this->data.rc_angles(2);

  // misc
  this->data.cycle_time = U16BIT(frame.data, 51, 50);
  this->data.i2c_error_count = U16BIT(frame.data, 53, 52);
  this->data.system_error = U16BIT(frame.data, 15, 14);
  this->data.battery_level = U16BIT(frame.data, 56, 55);

  // this->data.printData();

  return 0;
}

int SBGC::getAnglesExt() {
  int retval;
  // SBGCRealtimeData data;

  // Request real time data
  SBGCFrame frame;
  frame.buildFrame(CMD_GET_ANGLES_EXT);
  retval = this->sendFrame(frame);
  if (retval == -1) {
    LOG_ERROR("Failed to request SBGC realtime data!");
    return -1;
  }

  // Obtain real time data
  retval = this->readFrame(54, frame);
  if (retval == -1) {
    LOG_ERROR("Failed to parse SBGC frame for realtime data!");
    return -1;
  }

  // Roll
  this->data.camera_angles(0) = S16BIT(frame.data, 1, 0);
  this->data.rc_angles(0) = S16BIT(frame.data, 3, 2);
  this->data.encoder_angles(0) = S16BIT(frame.data, 8, 4);

  // Pitch
  this->data.camera_angles(1) = S16BIT(frame.data, 19, 18);
  this->data.rc_angles(1) = S16BIT(frame.data, 21, 20);
  this->data.encoder_angles(1) = S16BIT(frame.data, 26, 22);

  this->data.camera_angles(2) = S16BIT(frame.data, 37, 36);
  this->data.rc_angles(2) = S16BIT(frame.data, 39, 38);
  this->data.encoder_angles(2) = S16BIT(frame.data, 44, 40);

  this->data.camera_angles(0) = (DEG_PER_BIT) * this->data.camera_angles(0);
  this->data.camera_angles(1) = (DEG_PER_BIT) * this->data.camera_angles(1);
  this->data.camera_angles(2) = (DEG_PER_BIT) * this->data.camera_angles(2);

  this->data.rc_angles(0) = (DEG_PER_BIT) * this->data.rc_angles(0);
  this->data.rc_angles(1) = (DEG_PER_BIT) * this->data.rc_angles(1);
  this->data.rc_angles(2) = (DEG_PER_BIT) * this->data.rc_angles(2);

  this->data.encoder_angles(0) = (DEG_PER_BIT) * this->data.encoder_angles(0);
  this->data.encoder_angles(1) = (DEG_PER_BIT) * this->data.encoder_angles(1);
  this->data.encoder_angles(2) = (DEG_PER_BIT) * this->data.encoder_angles(2);

  this->data.printData();

  return 0;
}

int SBGC::setAngle(const double roll, const double pitch, const double yaw) {
  // adjust roll, pitch and yaw
  int16_t roll_adjusted = (int16_t)(roll / DEG_PER_BIT);
  int16_t pitch_adjusted = (int16_t)(pitch / DEG_PER_BIT);
  int16_t yaw_adjusted = (int16_t)(yaw / DEG_PER_BIT);

  // control mode
  uint8_t data[13];
  data[0] = MODE_ANGLE;

  // speed roll
  data[1] = 0;
  data[2] = 0;

  // angle roll
  data[3] = ((roll_adjusted >> 0) & 0xff);
  data[4] = ((roll_adjusted >> 8) & 0xff);

  // speed pitch
  data[5] = 0;
  data[6] = 0;

  // angle pitch
  data[7] = ((pitch_adjusted >> 0) & 0xff);
  data[8] = ((pitch_adjusted >> 8) & 0xff);

  // speed yaw
  data[9] = 0;
  data[10] = 0;

  // angle yaw
  data[11] = ((yaw_adjusted >> 0) & 0xff);
  data[12] = ((yaw_adjusted >> 8) & 0xff);

  // build frame and send
  SBGCFrame frame;
  frame.buildFrame(CMD_CONTROL, data, 13);
  this->sendFrame(frame);

  return 0;
}

int SBGC::setSpeedAngle(const double roll,
                        const double pitch,
                        const double yaw,
                        const double roll_speed,
                        const double pitch_speed,
                        const double yaw_speed) {
  // Adjust roll, pitch and yaw
  int16_t roll_adjusted = (int16_t)(roll / DEG_PER_BIT);
  int16_t pitch_adjusted = (int16_t)(pitch / DEG_PER_BIT);
  int16_t yaw_adjusted = (int16_t)(yaw / DEG_PER_BIT);
  int16_t roll_speed_adjusted = (int16_t)(roll_speed / DEG_SEC_PER_BIT);
  int16_t pitch_speed_adjusted = (int16_t)(pitch_speed / DEG_SEC_PER_BIT);
  int16_t yaw_speed_adjusted = (int16_t)(yaw_speed / DEG_SEC_PER_BIT);

  // Control mode
  uint8_t data[13];
  data[0] = MODE_SPEED_ANGLE;

  // Speed roll
  data[1] = ((roll_speed_adjusted >> 0) & 0xff);
  data[2] = ((roll_speed_adjusted >> 8) & 0xff);

  // Angle roll
  data[3] = ((roll_adjusted >> 0) & 0xff);
  data[4] = ((roll_adjusted >> 8) & 0xff);

  // Speed pitch
  data[5] = ((pitch_speed_adjusted >> 0) & 0xff);
  data[6] = ((pitch_speed_adjusted >> 8) & 0xff);

  // Angle pitch
  data[7] = ((pitch_adjusted >> 0) & 0xff);
  data[8] = ((pitch_adjusted >> 8) & 0xff);

  // Speed yaw
  data[9] = ((yaw_speed_adjusted >> 0) & 0xff);
  data[10] = ((yaw_speed_adjusted >> 8) & 0xff);

  // Angle yaw
  data[11] = ((yaw_adjusted >> 0) & 0xff);
  data[12] = ((yaw_adjusted >> 8) & 0xff);

  // Build frame and send
  SBGCFrame frame;
  frame.buildFrame(CMD_CONTROL, data, 13);
  this->sendFrame(frame);

  return 0;
}

} // namespace gvio
