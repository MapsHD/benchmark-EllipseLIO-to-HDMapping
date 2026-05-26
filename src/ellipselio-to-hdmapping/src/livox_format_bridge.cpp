// Livox PointCloud2 format bridge.
//
// The upstream ellipselio algorithm (LidarProcess::SetPoint<LivoxPoint>)
// reads a `double timestamp` field and constructs rclcpp::Time(timestamp).
// In ROS 2 Humble that constructor takes nanoseconds, so this bridge stores
// the absolute point time as nanoseconds in the double timestamp field.
//
// The Bunker-DVI dataset bag was recorded with the older ROS 1
// livox_ros_driver, whose PointCloud2 layout is:
//     x:f32 y:f32 z:f32 intensity:f32 tag:u8 line:u8 time:f32
// where `time` is a per-point offset in seconds relative to header.stamp.
//
// This node subscribes to the recorded topic, rewrites each point into the
// upstream algorithm-native layout, and republishes. It is equivalent to the
// Bunker-DVI algorithm patch that made LivoxPoint carry `float time` and used
// `point_time += Duration::from_seconds(time)`, but keeps the algorithm source
// untouched.
//
// Input layout (auto-validated by field names):
//     x:f32 y:f32 z:f32 intensity:f32 tag:u8 line:u8 time:f32   (22 B/point)
//
// Output layout:
//     x:f32 y:f32 z:f32 intensity:f32 tag:u8 line:u8 timestamp:f64 (32 B/point,
//     padded to satisfy 8-byte alignment of the double).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

namespace {

constexpr const char* kInTimeField = "time";
constexpr const char* kOutTimeField = "timestamp";

struct InOffsets {
  int x = -1, y = -1, z = -1, intensity = -1, tag = -1, line = -1, time = -1;
  bool Valid() const {
    return x >= 0 && y >= 0 && z >= 0 && intensity >= 0 && tag >= 0 &&
           line >= 0 && time >= 0;
  }
};

InOffsets FindOffsets(const sensor_msgs::msg::PointCloud2& msg) {
  InOffsets o;
  for (const auto& f : msg.fields) {
    if (f.name == "x") o.x = f.offset;
    else if (f.name == "y") o.y = f.offset;
    else if (f.name == "z") o.z = f.offset;
    else if (f.name == "intensity") o.intensity = f.offset;
    else if (f.name == "tag") o.tag = f.offset;
    else if (f.name == "line") o.line = f.offset;
    else if (f.name == kInTimeField) o.time = f.offset;
  }
  return o;
}

}  // namespace

class LivoxFormatBridge : public rclcpp::Node {
 public:
  LivoxFormatBridge()
      : Node("livox_format_bridge"),
        input_topic_(declare_parameter<std::string>("input_topic",
                                                    "/livox/pointcloud")),
        output_topic_(declare_parameter<std::string>(
          "output_topic", "/livox/pointcloud_native")) {
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LivoxFormatBridge::OnCloud, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, rclcpp::SensorDataQoS());
    RCLCPP_INFO(get_logger(),
                "livox_format_bridge: %s -> %s (float `time` -> double "
            "absolute `timestamp` nanoseconds)",
                input_topic_.c_str(), output_topic_.c_str());
  }

 private:
  void OnCloud(const sensor_msgs::msg::PointCloud2::SharedPtr in) {
    if (in->data.empty()) return;
    const auto off = FindOffsets(*in);
    if (!off.Valid()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Input cloud missing one or more expected fields "
          "(x,y,z,intensity,tag,line,time); dropping message");
      return;
    }

    // Output layout — keep 8-byte alignment for the trailing double.
    //   0  x         f32
    //   4  y         f32
    //   8  z         f32
    //  12  intensity f32
    //  16  tag       u8
    //  17  line      u8
    //  18  <pad>     6 B
    //  24  timestamp f64
    //  32  <next point>
    constexpr uint32_t kOutPointStep = 32;
    constexpr uint32_t kOffX = 0;
    constexpr uint32_t kOffY = 4;
    constexpr uint32_t kOffZ = 8;
    constexpr uint32_t kOffI = 12;
    constexpr uint32_t kOffTag = 16;
    constexpr uint32_t kOffLine = 17;
    constexpr uint32_t kOffTimestamp = 24;

    sensor_msgs::msg::PointCloud2 out;
    out.header = in->header;
    out.height = 1;
    out.width = in->width * in->height;
    out.is_bigendian = in->is_bigendian;
    out.is_dense = in->is_dense;
    out.point_step = kOutPointStep;
    out.row_step = kOutPointStep * out.width;

    out.fields.resize(7);
    auto set_field = [](sensor_msgs::msg::PointField& f, const char* name,
                        uint32_t offset, uint8_t type) {
      f.name = name;
      f.offset = offset;
      f.datatype = type;
      f.count = 1;
    };
    set_field(out.fields[0], "x", kOffX,
              sensor_msgs::msg::PointField::FLOAT32);
    set_field(out.fields[1], "y", kOffY,
              sensor_msgs::msg::PointField::FLOAT32);
    set_field(out.fields[2], "z", kOffZ,
              sensor_msgs::msg::PointField::FLOAT32);
    set_field(out.fields[3], "intensity", kOffI,
              sensor_msgs::msg::PointField::FLOAT32);
    set_field(out.fields[4], "tag", kOffTag,
              sensor_msgs::msg::PointField::UINT8);
    set_field(out.fields[5], "line", kOffLine,
              sensor_msgs::msg::PointField::UINT8);
    set_field(out.fields[6], kOutTimeField, kOffTimestamp,
              sensor_msgs::msg::PointField::FLOAT64);

    out.data.assign(static_cast<size_t>(out.row_step), 0);

    const double base_ns = static_cast<double>(in->header.stamp.sec) * 1e9 +
                           static_cast<double>(in->header.stamp.nanosec);
    const uint32_t in_step = in->point_step;
    const uint32_t n = out.width;
    const uint8_t* in_data = in->data.data();
    uint8_t* out_data = out.data.data();
    double min_point_ts = std::numeric_limits<double>::infinity();
    double max_point_ts = -std::numeric_limits<double>::infinity();

    for (uint32_t i = 0; i < n; ++i) {
      const uint8_t* sp = in_data + static_cast<size_t>(i) * in_step;
      uint8_t* dp = out_data + static_cast<size_t>(i) * kOutPointStep;

      std::memcpy(dp + kOffX, sp + off.x, 4);
      std::memcpy(dp + kOffY, sp + off.y, 4);
      std::memcpy(dp + kOffZ, sp + off.z, 4);
      std::memcpy(dp + kOffI, sp + off.intensity, 4);
      dp[kOffTag] = sp[off.tag];
      dp[kOffLine] = sp[off.line];

      float t_off_f = 0.f;
      std::memcpy(&t_off_f, sp + off.time, 4);
      // Per-point offset units in livox_ros_driver vary by version.
      // Detect and convert to seconds, then clamp to one scan period.
      double t_off = static_cast<double>(t_off_f);
      const double abs_t = std::fabs(t_off);
      if (abs_t > 1e6) {
        t_off *= 1e-9;  // nanoseconds
      } else if (abs_t > 1.0) {
        t_off *= 1e-6;  // microseconds
      }
      if (t_off > 0.2) t_off = 0.2;
      else if (t_off < -0.2) t_off = -0.2;

      const double abs_ts_ns = base_ns + t_off * 1e9;
      const double abs_ts_sec = abs_ts_ns * 1e-9;
      min_point_ts = std::min(min_point_ts, abs_ts_sec);
      max_point_ts = std::max(max_point_ts, abs_ts_sec);
      std::memcpy(dp + kOffTimestamp, &abs_ts_ns, 8);
    }

    pub_->publish(out);

    if (++msg_count_ == 1 || (msg_count_ % 50) == 0) {
      RCLCPP_INFO(
          get_logger(),
          "[bridge] forwarded #%lu  (in_pts=%u, point_step=%u->%u, "
          "header.stamp=%u.%09u, lidar=%.6f..%.6f, timestamp_unit=ns)",
          static_cast<unsigned long>(msg_count_), n, in_step, kOutPointStep,
          in->header.stamp.sec, in->header.stamp.nanosec, min_point_ts,
          max_point_ts);
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  uint64_t msg_count_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxFormatBridge>());
  rclcpp::shutdown();
  return 0;
}
