#include <System.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

static bool read_exact(std::istream& in, char* dst, std::size_t n) {
    in.read(dst, static_cast<std::streamsize>(n));
    return static_cast<std::size_t>(in.gcount()) == n;
}

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "Usage: live_slam VOCABULARY SETTINGS OUTPUT_PREFIX UDP_PORT\n";
        return 2;
    }

    const std::string vocabulary = argv[1];
    const std::string settings = argv[2];
    const std::string output_prefix = argv[3];
    const int port = std::stoi(argv[4]);

    if (port < 1 || port > 65535) {
        std::cerr << "Invalid UDP port\n";
        return 2;
    }

    const int sock = socket(AF_INET, SOCK_DGRAM, 0);

    if (sock < 0) {
        perror("socket");
        return 1;
    }

    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_port = htons(static_cast<uint16_t>(port));
    destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    std::ofstream pose_file(output_prefix + "_tracked.txt");
    if (!pose_file) {
        std::cerr << "[LIVE_SLAM] Cannot open trajectory output file\n";
        close(sock);
        return 1;
    }
    pose_file << std::fixed;

    try {
        ORB_SLAM3::System slam(
            vocabulary, settings, ORB_SLAM3::System::MONOCULAR, true
        );

        std::cerr << "[LIVE_SLAM] Ready; awaiting frames on stdin\n";
        std::cerr << "[LIVE_SLAM] Pose output: 127.0.0.1:" << port << '\n';

        // Frame protocol: double timestamp, uint32 JPEG length, JPEG bytes.
        // Both processes run in the same WSL environment.
        for (;;) {
            double timestamp = 0.0;
            uint32_t length = 0;

            if (!read_exact(std::cin, reinterpret_cast<char*>(&timestamp),
                            sizeof(timestamp))) {
                break;
            }
            if (!read_exact(std::cin, reinterpret_cast<char*>(&length),
                            sizeof(length))) {
                std::cerr << "[LIVE_SLAM] Incomplete frame header\n";
                break;
            }
            if (length < 4 || length > 2'000'000) {
                std::cerr << "[LIVE_SLAM] Invalid JPEG length: "
                          << length << '\n';
                break;
            }

            std::vector<unsigned char> bytes(length);
            if (!read_exact(std::cin, reinterpret_cast<char*>(bytes.data()),
                            bytes.size())) {
                std::cerr << "[LIVE_SLAM] Incomplete JPEG payload\n";
                break;
            }

            cv::Mat image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
            if (image.empty()) {
                std::cerr << "[LIVE_SLAM] JPEG decode failed\n";
                continue;
            }

            if (image.cols != 640 || image.rows != 480) {
                std::cerr << "[LIVE_SLAM] Expected 640x480, got "
                          << image.cols << 'x' << image.rows << '\n';
                continue;
            }

            if (image.channels() == 4) {
                cv::cvtColor(image, image, cv::COLOR_BGRA2BGR);
            } else if (image.channels() == 1) {
                cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
            } else if (image.channels() != 3) {
                std::cerr << "[LIVE_SLAM] Unsupported image channel count\n";
                continue;
            }

            static unsigned long long frame_count = 0;
            ++frame_count;

            if (frame_count <= 5 || frame_count % 30 == 0) {
                std::cerr << "[DEBUG] Frame " << frame_count
                        << " timestamp=" << std::setprecision(12) << timestamp
                        << " size=" << image.cols << "x" << image.rows
                        << " type=" << image.type()
                        << " channels=" << image.channels()
                        << std::endl;
            }

            Sophus::SE3f Tcw = slam.TrackMonocular(image, timestamp);
            const int state = slam.GetTrackingState();

            if (frame_count <= 5 || frame_count % 30 == 0) {
                std::cerr << "[DEBUG] Frame " << frame_count
                        << " tracking_state=" << state
                        << std::endl;
            }
            std::ostringstream msg;
            msg << std::setprecision(9)
                << "{\"timestamp\":" << timestamp
                << ",\"tracking_state\":" << state;

            if (state == 2) { // Tracking::OK only
                const Sophus::SE3f Twc = Tcw.inverse();
                const auto t = Twc.translation();
                const auto q = Twc.unit_quaternion();

                pose_file << std::setprecision(6) << timestamp << " "
                          << std::setprecision(9)
                          << t.x() << " " << t.y() << " " << t.z() << " "
                          << q.x() << " " << q.y() << " "
                          << q.z() << " " << q.w() << '\n';
                pose_file.flush();
                msg << ",\"position\":[" << t.x() << ',' << t.y() << ','
                    << t.z() << "],\"quaternion_xyzw\":["
                    << q.x() << ',' << q.y() << ','
                    << q.z() << ',' << q.w() << ']';

            } else {
                msg << ",\"position\":null,\"quaternion_xyzw\":null";
            }

            const std::string packet = msg.str();
            sendto(sock, packet.data(), packet.size(), 0,
                   reinterpret_cast<sockaddr*>(&destination),
                   sizeof(destination));
        }

        slam.Shutdown();
        slam.SaveTrajectoryTUM(output_prefix + "_camera.txt");
        slam.SaveKeyFrameTrajectoryTUM(output_prefix + "_keyframes.txt");

        std::cerr << "[LIVE_SLAM] Saved trajectories with prefix "
                  << output_prefix << '\n';
    } catch (const std::exception& e) {
        std::cerr << "[LIVE_SLAM] Fatal error: " << e.what() << '\n';
        close(sock);
        return 1;
    }

    close(sock);
    return 0;
}
