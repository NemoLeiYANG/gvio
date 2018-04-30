#include "gvio/dataset/kitti/raw/oxts.hpp"

namespace gvio {

int OXTSEntry::load(const std::string &file_path) {
  // Load file
  std::ifstream oxt_file(file_path.c_str());
  if (oxt_file.good() == false) {
    return -1;
  }

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

  this->gps = Vec3{lat, lon, alt};
  this->rpy = Vec3{roll, pitch, yaw};
  this->v_G = Vec3{ve, vn, vu};
  this->v_B = Vec3{vf, vl, vu};
  this->a_G = Vec3{ax, ay, az};
  this->a_B = Vec3{af, al, au};
  this->w_G = Vec3{wx, wy, wz};
  this->w_B = Vec3{wf, wl, wu};
  this->pos_accuracy = pos_acc;
  this->vel_accuracy = vel_acc;

  return 0;
}

int OXTS::loadOXTS(const std::string &oxts_dir) {
  // Get list of oxts files
  const std::string oxts_data_dir = strip(oxts_dir) + "/data";
  std::vector<std::string> oxts_files;
  if (list_dir(oxts_data_dir, oxts_files) != 0) {
    return -1;
  }
  std::sort(oxts_files.begin(), oxts_files.end());

  // Get first GPS point
  OXTSEntry first_entry;
  const std::string file_path = oxts_data_dir + "/" + oxts_files[0];
  if (first_entry.load(file_path) != 0) {
    return -1;
  }

  // Store initial conditions
  const Vec3 gps_ref = first_entry.gps;
  gps.emplace_back(first_entry.gps);
  rpy.emplace_back(first_entry.rpy);
  p_G.emplace_back(0, 0, 0);
  v_G.emplace_back(first_entry.v_G);
  v_B.emplace_back(first_entry.v_B);
  a_G.emplace_back(first_entry.a_G);
  a_B.emplace_back(first_entry.a_B);
  w_G.emplace_back(first_entry.w_G);
  w_B.emplace_back(first_entry.w_B);
  pos_accuracy.push_back(first_entry.pos_accuracy);
  vel_accuracy.push_back(first_entry.vel_accuracy);

  // Parse oxts files
  for (size_t i = 1; i < oxts_files.size(); i++) {
    // Parse single oxts file
    OXTSEntry entry;
    const std::string file_path = oxts_data_dir + "/" + oxts_files[i];
    if (entry.load(file_path) != 0) {
      return -1;
    }

    // Calculate local position
    double dist_N = 0.0;
    double dist_E = 0.0;
    double alt = entry.gps(2) - gps_ref(2);
    latlon_diff(gps_ref(0),
                gps_ref(1),
                entry.gps(0),
                entry.gps(1),
                &dist_N,
                &dist_E);

    // Store data
    gps.emplace_back(entry.gps);
    rpy.emplace_back(entry.rpy);
    p_G.emplace_back(dist_E, dist_N, alt);
    v_G.emplace_back(entry.v_G);
    v_B.emplace_back(entry.v_B);
    a_G.emplace_back(entry.a_G);
    a_B.emplace_back(entry.a_B);
    w_G.emplace_back(entry.w_G);
    w_B.emplace_back(entry.w_B);
    pos_accuracy.push_back(entry.pos_accuracy);
    vel_accuracy.push_back(entry.vel_accuracy);
  }

  return 0;
}

int OXTS::parseSingleTimeStamp(const std::string &line, long *s) {
  // Parse datetime string
  unsigned int year = 0;
  unsigned int month = 0;
  unsigned int day = 0;
  unsigned int hour = 0;
  unsigned int minute = 0;
  unsigned int second = 0;
  unsigned int subsecond = 0;

  int scanned = std::sscanf(line.c_str(),
                            "%4u-%2u-%2u %2u:%2u:%2u.%u",
                            &year,
                            &month,
                            &day,
                            &hour,
                            &minute,
                            &second,
                            &subsecond);
  if (scanned != 7) {
    return -1;
  }

  // Convert datetime to milliseconds since epoch (1970)
  struct tm t = {}; // IMPORTANT: MUST INITIALIZE TO ZERO ELSE BUGS...
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;
  *s = ((double) mktime(&t) * 1.0e9) + subsecond;

  return 0;
}

int OXTS::loadTimeStamps(const std::string &oxts_dir) {
  // Setup parse
  const std::string file_path = strip(oxts_dir) + "/timestamps.txt";
  std::string line;
  std::ifstream timestamps_file(file_path.c_str());

  // Get first timestamp
  long ts_first = 0;
  int retval = 0;
  std::getline(timestamps_file, line);
  retval = this->parseSingleTimeStamp(line, &ts_first);
  if (retval != 0) {
    LOG_ERROR("Failed to parse timestamp -> [%s]", line.c_str());
    return -1;
  }
  this->timestamps.push_back(ts_first);
  this->time.push_back(0.0);
  line = std::string();

  // Parse the rest of timestamps
  long ts = 0;
  while (std::getline(timestamps_file, line)) {
    // Parse timestamp
    retval = this->parseSingleTimeStamp(line, &ts);
    if (retval != 0) {
      LOG_ERROR("Failed to parse timestamp -> [%s]", line.c_str());
      return -1;
    }

    // Check to make sure current parsed timestamp is larger than first
    if (ts_first > ts) {
      LOG_ERROR("First timestamp: %ld", ts_first);
      LOG_ERROR("Current timestamp: %ld", ts);
      LOG_ERROR("Current timestamp line: %s", line.c_str());
      FATAL("Failed to parse timestamp properly!");
    }

    this->timestamps.push_back(ts);
    this->time.push_back((double) (ts - ts_first) * 1.0e-9);
    line = std::string();
  }

  return 0;
}

int OXTS::load(const std::string &oxts_dir) {
  if (this->loadOXTS(oxts_dir) != 0) {
    return -1;
  }
  if (this->loadTimeStamps(oxts_dir) != 0) {
    return -1;
  }

  return 0;
}

} // namespace gvio
