#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <Eigen/Dense>
#include "laszip_api.h"
#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iomanip>

#include "laz_writer.hpp"


struct TrajectoryPose
{
    uint64_t timestamp_ns;
    double x_m;
    double y_m;
    double z_m;
    double qw;
    double qx;
    double qy;
    double qz;
    Eigen::Affine3d pose;
    double om_rad;  // Roll (omega)
    double fi_rad;  // Pitch (phi)
    double ka_rad;  // Yaw (kappa)
};

struct TaitBryanPose
{
    double px;
    double py;
    double pz;
    double om;
    double fi;
    double ka;
};

inline TaitBryanPose pose_tait_bryan_from_affine_matrix(Eigen::Affine3d m){
    TaitBryanPose pose;

    pose.px = m(0,3);
    pose.py = m(1,3);
    pose.pz = m(2,3);

    if (m(0,2) < 1) {
        if (m(0,2) > -1) {
            pose.fi = asin(m(0,2));
            pose.om = atan2(-m(1,2), m(2,2));
            pose.ka = atan2(-m(0,1), m(0,0));
            return pose;
        }
        else
        {
            pose.fi = -M_PI / 2.0;
            pose.om = -atan2(m(1,0), m(1,1));
            pose.ka = 0;
            return pose;
        }
    }
    else {
        pose.fi = M_PI / 2.0;
        pose.om = atan2(m(1,0), m(1,1));
        pose.ka = 0.0;
        return pose;
    }

    return pose;
}

namespace fs = std::filesystem;
std::vector<Point3Di> points_global;

std::vector<TrajectoryPose> trajectory;
std::vector<std::vector<TrajectoryPose>> chunks_trajectory;

bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames)
{
    std::ofstream outfile;
    outfile.open(file_name);
    if (!outfile.good())
    {
        std::cout << "can not save file: '" << file_name << "'" << std::endl;
        return false;
    }

    outfile << m_poses.size() << std::endl;
    for (size_t i = 0; i < m_poses.size(); i++)
    {
        outfile << filenames[i] << std::endl;
        outfile << m_poses[i](0, 0) << " " << m_poses[i](0, 1) << " " << m_poses[i](0, 2) << " " << m_poses[i](0, 3) << std::endl;
        outfile << m_poses[i](1, 0) << " " << m_poses[i](1, 1) << " " << m_poses[i](1, 2) << " " << m_poses[i](1, 3) << std::endl;
        outfile << m_poses[i](2, 0) << " " << m_poses[i](2, 1) << " " << m_poses[i](2, 2) << " " << m_poses[i](2, 3) << std::endl;
        outfile << "0 0 0 1" << std::endl;
    }
    outfile.close();

    return true;
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <input_bag> <output_directory>" << std::endl;
        return 1;
    }

    const std::string input_bag = argv[1];
    const std::string output_directory = argv[2];

    // Topic names (default to EllipseLIO publishers). Allow overrides via argv[3..4].
    std::string odom_topic = "/ellipselio_odom";
    std::string cloud_topic = "/cloud_scan";
    if (argc >= 4) odom_topic = argv[3];
    if (argc >= 5) cloud_topic = argv[4];

    std::cout << "Processing bag : " << input_bag << std::endl;
    std::cout << "Odometry topic : " << odom_topic << std::endl;
    std::cout << "Cloud topic    : " << cloud_topic << std::endl;

    // ── Read ROS 2 bag ───────────────────────────────────────────────────────
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = input_bag;

    reader.open(storage_options);

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serializer;
    rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serializer;

    std::cout << "First pass: reading odometry and clouds..." << std::endl;

    // Store clouds temporarily with their timestamps
    struct CloudEntry {
        uint64_t timestamp_ns;
        sensor_msgs::msg::PointCloud2 cloud;
    };
    std::vector<CloudEntry> clouds;

    while (reader.has_next()) {
        auto serialized_msg = reader.read_next();

        if (serialized_msg->topic_name == odom_topic) {
            nav_msgs::msg::Odometry odom_msg;
            rclcpp::SerializedMessage extracted_msg(*serialized_msg->serialized_data);
            odom_serializer.deserialize_message(&extracted_msg, &odom_msg);

            double x = odom_msg.pose.pose.position.x;
            double y = odom_msg.pose.pose.position.y;
            double z = odom_msg.pose.pose.position.z;

            double qx = odom_msg.pose.pose.orientation.x;
            double qy = odom_msg.pose.pose.orientation.y;
            double qz = odom_msg.pose.pose.orientation.z;
            double qw = odom_msg.pose.pose.orientation.w;

            TrajectoryPose pose;
            // Prefer header.stamp (EllipseLIO sets it from sensor data); fall back to bag receive time.
            uint64_t header_ns =
                static_cast<uint64_t>(odom_msg.header.stamp.sec) * 1000000000ULL +
                static_cast<uint64_t>(odom_msg.header.stamp.nanosec);
            pose.timestamp_ns = (header_ns > 0)
                ? header_ns
                : static_cast<uint64_t>(serialized_msg->time_stamp);

            pose.x_m = x;
            pose.y_m = y;
            pose.z_m = z;
            pose.qw = qw;
            pose.qx = qx;
            pose.qy = qy;
            pose.qz = qz;

            pose.pose = Eigen::Affine3d::Identity();
            Eigen::Vector3d trans(x, y, z);
            Eigen::Quaterniond q(qw, qx, qy, qz);
            pose.pose.translation() = trans;
            pose.pose.linear() = q.toRotationMatrix();

            TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(pose.pose);
            pose.om_rad = tb.om;
            pose.fi_rad = tb.fi;
            pose.ka_rad = tb.ka;

            trajectory.push_back(pose);
        }

        if (serialized_msg->topic_name == cloud_topic) {
            CloudEntry entry;
            rclcpp::SerializedMessage extracted_msg(*serialized_msg->serialized_data);
            cloud_serializer.deserialize_message(&extracted_msg, &entry.cloud);

            uint64_t header_ns =
                static_cast<uint64_t>(entry.cloud.header.stamp.sec) * 1000000000ULL +
                static_cast<uint64_t>(entry.cloud.header.stamp.nanosec);
            entry.timestamp_ns = (header_ns > 0)
                ? header_ns
                : static_cast<uint64_t>(serialized_msg->time_stamp);

            clouds.push_back(std::move(entry));
        }
    }

    // Ensure trajectory is sorted by timestamp for chunk indexing.
    std::sort(trajectory.begin(), trajectory.end(),
              [](const TrajectoryPose& a, const TrajectoryPose& b) {
                  return a.timestamp_ns < b.timestamp_ns;
              });

    std::cout << "Read " << trajectory.size() << " odometry poses and "
              << clouds.size() << " point clouds." << std::endl;

    if (!trajectory.empty() && !clouds.empty()) {
        std::cout << "DEBUG: odom  timestamp range: [" << trajectory.front().timestamp_ns
                  << " .. " << trajectory.back().timestamp_ns << "]" << std::endl;
        std::cout << "DEBUG: cloud timestamp range: [" << clouds.front().timestamp_ns
                  << " .. " << clouds.back().timestamp_ns << "]" << std::endl;
    }

    if (trajectory.empty() || clouds.empty()) {
        std::cerr << "Error: no odometry or cloud data found in bag!" << std::endl;
        return 1;
    }

    // ── Collect point cloud points ───────────────────────────────────────────
    // EllipseLIO publishes /cloud_scan already in the world (odom_ellipselio)
    // frame, so we do NOT apply any pose transformation here.
    std::cout << "Collecting world-frame point clouds..." << std::endl;

    for (const auto& ce : clouds) {
        const auto& cloud_msg = ce.cloud;
        size_t num_points = cloud_msg.width * cloud_msg.height;
        if (num_points == 0) continue;

        // Find field offsets
        int x_offset = -1, y_offset = -1, z_offset = -1, intensity_offset = -1;
        for (const auto& field : cloud_msg.fields) {
            if (field.name == "x") x_offset = field.offset;
            if (field.name == "y") y_offset = field.offset;
            if (field.name == "z") z_offset = field.offset;
            if (field.name == "intensity") intensity_offset = field.offset;
        }
        if (x_offset < 0 || y_offset < 0 || z_offset < 0) continue;

        for (size_t i = 0; i < num_points; ++i) {
            size_t byte_offset = i * cloud_msg.point_step;
            float px, py, pz;
            memcpy(&px, &cloud_msg.data[byte_offset + x_offset], sizeof(float));
            memcpy(&py, &cloud_msg.data[byte_offset + y_offset], sizeof(float));
            memcpy(&pz, &cloud_msg.data[byte_offset + z_offset], sizeof(float));

            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;

            float intensity = 0.0f;
            if (intensity_offset >= 0) {
                memcpy(&intensity, &cloud_msg.data[byte_offset + intensity_offset], sizeof(float));
            }

            Point3Di point_global;
            point_global.timestamp = ce.timestamp_ns;
            point_global.point = Eigen::Vector3d(px, py, pz);
            point_global.intensity = intensity;
            point_global.index_pose = static_cast<int>(i);
            point_global.lidarid = 0;
            point_global.index_point = static_cast<int>(i);

            points_global.push_back(point_global);
        }
    }

    std::cout << "Total global points: " << points_global.size() << std::endl;

    // ── Chunk point clouds ───────────────────────────────────────────────────
    std::vector<std::vector<Point3Di>> chunks_pc;

    int counter = 0;
    std::vector<Point3Di> chunk;

    for (size_t i = 0; i < points_global.size(); i++)
    {
        chunk.push_back(points_global[i]);

        if (chunk.size() > 2000000)
        {
            counter++;
            chunks_pc.push_back(chunk);
            chunk.clear();
            std::cout << "adding chunk [" << counter << "]" << std::endl;
        }
    }

    std::cout << "remaining points: " << chunk.size() << std::endl;

    // Always flush the residual chunk so small recordings still produce output.
    if (!chunk.empty())
    {
        chunks_pc.push_back(chunk);
    }

    std::cout << "cleaning points" << std::endl;
    points_global.clear();
    std::cout << "points cleaned" << std::endl;

    // ── Index trajectory into chunks ─────────────────────────────────────────
    std::cout << "start indexing chunks_trajectory" << std::endl;
    chunks_trajectory.resize(chunks_pc.size());

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        if(i % 1000 == 0){
            std::cout << "computing [" << i + 1 << "] of: " << trajectory.size() << std::endl;
        }
        bool assigned = false;
        for (size_t j = 0; j < chunks_pc.size(); j++)
        {
            if (chunks_pc[j].size() > 0)
            {
                if (trajectory[i].timestamp_ns >= chunks_pc[j][0].timestamp &&
                    trajectory[i].timestamp_ns <= chunks_pc[j][chunks_pc[j].size() - 1].timestamp)
                {
                    chunks_trajectory[j].push_back(trajectory[i]);
                    assigned = true;
                    break;
                }
            }
        }
        if (!assigned && i < 5) {
            std::cout << "DEBUG: trajectory[" << i << "] ts=" << trajectory[i].timestamp_ns
                      << " not matched.";
            if (!chunks_pc.empty() && !chunks_pc[0].empty()) {
                std::cout << " chunk[0] range=[" << chunks_pc[0][0].timestamp
                          << ".." << chunks_pc[0].back().timestamp << "]";
            } else {
                std::cout << " (no point-cloud chunks available)";
            }
            std::cout << std::endl;
        }
    }

    for (const auto &trj : chunks_trajectory)
    {
        std::cout << "number of trajectory elements: " << trj.size() << std::endl;
    }

    // ── Transform chunks to local coordinate system ──────────────────────────
    std::cout << "start transforming chunks_pc to local coordinate system" << std::endl;
    for (size_t i = 0; i < chunks_pc.size(); i++)
    {
        std::cout << "computing [" << i + 1 << "] of: " << chunks_pc.size() << std::endl;
        if (chunks_trajectory[i].size() == 0){
            continue;
        }

        Eigen::Vector3d trans(chunks_trajectory[i][0].x_m, chunks_trajectory[i][0].y_m, chunks_trajectory[i][0].z_m);
        Eigen::Quaterniond q(chunks_trajectory[i][0].qw, chunks_trajectory[i][0].qx, chunks_trajectory[i][0].qy, chunks_trajectory[i][0].qz);

        Eigen::Affine3d first_affine = Eigen::Affine3d::Identity();
        first_affine.translation() = trans;
        first_affine.linear() = q.toRotationMatrix();

        Eigen::Affine3d first_affine_inv = first_affine.inverse();

        for (auto &p : chunks_pc[i])
        {
            p.point = first_affine_inv * p.point;
        }
    }

    // ── Create output directory ──────────────────────────────────────────────
    if (fs::exists(output_directory)) {
        std::cout << "Directory already exists." << std::endl;
    } else {
        try {
            if (fs::create_directory(output_directory)) {
                std::cout << "Directory has been created." << std::endl;
            } else {
                std::cerr << "Failed to create directory " << std::endl;
                return 1;
            }
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Error creating directory: " << e.what() << std::endl;
            return 1;
        }
    }

    fs::path outwd = output_directory;

    // ── Compute offset ───────────────────────────────────────────────────────
    Eigen::Vector3d offset(0, 0, 0);
    int cc = 0;
    for (size_t i = 0; i < chunks_trajectory.size(); i++)
    {
        for (size_t j = 0; j < chunks_trajectory[i].size(); j++)
        {
            Eigen::Vector3d trans_curr(chunks_trajectory[i][j].x_m, chunks_trajectory[i][j].y_m, chunks_trajectory[i][j].z_m);
            offset += trans_curr;
            cc++;
        }
    }
    if (cc > 0) {
        offset /= cc;
    } else {
        std::cerr << "WARNING: No trajectory elements matched any chunk! Using offset=0." << std::endl;
        std::cerr << "This means timestamp domains don't overlap between odom and cloud." << std::endl;
    }

    // ── Save LAZ files and trajectory CSVs ───────────────────────────────────
    std::vector<Eigen::Affine3d> m_poses;
    std::vector<std::string> file_names;

    for (size_t i = 0; i < chunks_pc.size(); i++)
    {
        if (chunks_pc[i].size() == 0){
            continue;
        }
        if (chunks_trajectory[i].size() == 0)
        {
            continue;
        }

        fs::path path(outwd);
        std::string filename = ("scan_lio_" + std::to_string(i) + ".laz");
        path /= filename;
        std::cout << "saving to: " << path << " number of points: " << chunks_pc[i].size() << std::endl;
        saveLaz(path.string(), chunks_pc[i]);
        file_names.push_back(filename);

        std::string trajectory_filename = ("trajectory_lio_" + std::to_string(i) + ".csv");
        fs::path pathtrj(outwd);
        pathtrj /= trajectory_filename;
        std::cout << "saving to: " << pathtrj << std::endl;

        std::ofstream outfile;
        outfile.open(pathtrj);
        if (!outfile.good())
        {
            std::cout << "can not save file: " << pathtrj << std::endl;
            return 1;
        }

        outfile << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 timestampUnix_nanoseconds om_rad fi_rad ka_rad" << std::endl;

        Eigen::Vector3d trans(chunks_trajectory[i][0].x_m, chunks_trajectory[i][0].y_m, chunks_trajectory[i][0].z_m);
        Eigen::Quaterniond q(chunks_trajectory[i][0].qw, chunks_trajectory[i][0].qx, chunks_trajectory[i][0].qy, chunks_trajectory[i][0].qz);

        Eigen::Affine3d first_affine = Eigen::Affine3d::Identity();
        first_affine.translation() = trans;
        first_affine.linear() = q.toRotationMatrix();

        Eigen::Affine3d first_affine_inv = first_affine.inverse();
        m_poses.push_back(first_affine);

        for (size_t j = 0; j < chunks_trajectory[i].size(); j++)
        {
            Eigen::Vector3d trans_curr(chunks_trajectory[i][j].x_m, chunks_trajectory[i][j].y_m, chunks_trajectory[i][j].z_m);
            Eigen::Quaterniond q_curr(chunks_trajectory[i][j].qw, chunks_trajectory[i][j].qx, chunks_trajectory[i][j].qy, chunks_trajectory[i][j].qz);

            Eigen::Affine3d first_affine_curr = Eigen::Affine3d::Identity();
            first_affine_curr.translation() = trans_curr;
            first_affine_curr.linear() = q_curr.toRotationMatrix();

            auto pose = first_affine_inv * first_affine_curr;
            outfile
                << chunks_trajectory[i][j].timestamp_ns << " " << std::setprecision(10)
                << pose(0, 0) << " "
                << pose(0, 1) << " "
                << pose(0, 2) << " "
                << pose(0, 3) << " "
                << pose(1, 0) << " "
                << pose(1, 1) << " "
                << pose(1, 2) << " "
                << pose(1, 3) << " "
                << pose(2, 0) << " "
                << pose(2, 1) << " "
                << pose(2, 2) << " "
                << pose(2, 3) << " "
                << chunks_trajectory[i][j].timestamp_ns << " "
                << std::setprecision(20)
                << chunks_trajectory[i][j].om_rad << " "
                << chunks_trajectory[i][j].fi_rad << " "
                << chunks_trajectory[i][j].ka_rad << " "
                << std::endl;
        }
        outfile.close();
    }

    for (auto &m : m_poses)
    {
        m.translation() -= offset;
    }

    // ── Save pose files ──────────────────────────────────────────────────────
    fs::path path(outwd);
    path /= "lio_initial_poses.reg";
    save_poses(path.string(), m_poses, file_names);
    fs::path path2(outwd);
    path2 /= "poses.reg";
    save_poses(path2.string(), m_poses, file_names);

    // ── Save session.json ────────────────────────────────────────────────────
    fs::path path3(outwd);
    path3 /= "session.json";

    std::cout << "saving file: '" << path3 << "'" << std::endl;

    nlohmann::json jj;
    nlohmann::json j;
    j["offset_x"] = 0.0;
    j["offset_y"] = 0.0;
    j["offset_z"] = 0.0;
    j["folder_name"] = outwd.string();
    j["out_folder_name"] = outwd.string();
    j["poses_file_name"] = (outwd / "poses.reg").string();
    j["initial_poses_file_name"] = (outwd / "lio_initial_poses.reg").string();
    j["out_poses_file_name"] = (outwd / "poses.reg").string();
    j["lidar_odometry_version"] = "HdMap";

    jj["Session Settings"] = j;

    nlohmann::json jlaz_file_names;
    for (size_t i = 0; i < chunks_pc.size(); i++)
    {
        if (chunks_pc[i].size() == 0)
        {
            continue;
        }
        if (chunks_trajectory[i].size() == 0)
        {
            continue;
        }

        fs::path path(outwd);
        std::string filename = ("scan_lio_" + std::to_string(i) + ".laz");
        path /= filename;
        std::cout << "adding file: " << path << std::endl;
        nlohmann::json jfn{
            {"file_name", path.string()}};
        jlaz_file_names.push_back(jfn);
    }
    jj["laz_file_names"] = jlaz_file_names;

    std::ofstream fs(path3.string());
    fs << jj.dump(2);
    fs.close();

    return 0;
}
