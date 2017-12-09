#include "gvio/kitti/raw/oxts.hpp"

namespace gvio {

int OXTS::parseOXTS(const std::string &oxts_dir) {
  // Get list of oxts files
  const std::string oxts_data_dir = strip(oxts_dir) + "/data";
  std::vector<std::string> oxts_files;
  if (list_dir(oxts_data_dir, oxts_files) != 0) {
    return -1;
  }

  // Parse oxts files
  std::sort(oxts_files.begin(), oxts_files.end());
  for (std::string file_name : oxts_files) {
    // Load oxts file
    const std::string file_path = oxts_data_dir + "/" + file_name;
    std::ifstream oxt_file(file_path.c_str());

    // Load the data
    std::string line;
    std::getline(oxt_file, line);
    const std::vector<double> array = parseArray(line);

    // Store
    const double lat = array[0];
    const double lon = array[1];
    const double alt = array[2];
    const double roll = array[3];
    const double pitch = array[4];
    const double yaw = array[5];
    const double vn = array[6];
    const double ve = array[7];
    const double vf = array[8];
    const double vl = array[9];
    const double vu = array[10];
    const double ax = array[11];
    const double ay = array[12];
    const double az = array[13];
    const double af = array[14];
    const double al = array[15];
    const double au = array[16];
    const double wx = array[17];
    const double wy = array[18];
    const double wz = array[19];
    const double wf = array[20];
    const double wl = array[21];
    const double wu = array[22];
    const double pos_acc = array[23];
    const double vel_acc = array[24];

    gps.emplace_back(lat, lon, alt);
    rpy.emplace_back(roll, pitch, yaw);
    v_G.emplace_back(vn, ve, vu);
    v_B.emplace_back(vf, vl, vu);
    a_G.emplace_back(ax, ay, az);
    a_B.emplace_back(af, al, au);
    w_G.emplace_back(wx, wy, wz);
    w_B.emplace_back(wf, wl, wu);
    pos_accuracy.push_back(pos_acc);
    vel_accuracy.push_back(vel_acc);
  }

  return 0;
}

int OXTS::parseSingleTimeStamp(const std::string &line, double *s) {
  // Parse datetime string
  unsigned int year, month, day, hour, minute, second, milliseconds;
  int scanned = std::sscanf(line.c_str(),
                            "%4u-%2u-%2u %2u:%2u:%2u.%3u",
                            &year,
                            &month,
                            &day,
                            &hour,
                            &minute,
                            &second,
                            &milliseconds);
  if (scanned != 7) {
    return -1;
  }

  // Convert datetime to milliseconds since epoch (1970)
  struct tm t;
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;
  *s = (double) mktime(&t) + milliseconds / 1000.0;

  return 0;
}

int OXTS::parseTimeStamps(const std::string &oxts_dir) {
  // Setup parse
  const std::string file_path = strip(oxts_dir) + "/timestamps.txt";
  std::string line;
  std::ifstream timestamps_file(file_path.c_str());

  // Get first timestamp
  double ts_first;
  std::getline(timestamps_file, line);
  this->parseSingleTimeStamp(line, &ts_first);
  this->timestamps.push_back(0.0);

  // Parse the rest of timestamps
  double ts;
  while (std::getline(timestamps_file, line)) {
    this->parseSingleTimeStamp(line, &ts);
    this->timestamps.push_back(ts - ts_first);
  }

  return 0;
}

int OXTS::load(const std::string &oxts_dir) {
  if (this->parseOXTS(oxts_dir) != 0) {
    return -1;
  }
  if (this->parseTimeStamps(oxts_dir) != 0) {
    return -1;
  }

  return 0;
}

} // namespace gvio
