#include <iostream>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sstream>
#include <fstream>
// #include <linux/can.h>
// #include <linux/can/raw.h>

#include "dbc/dbc_parser.h"
#include "v2c/v2c_transcoder.h"

std::string read_file(const std::string &dbc_path)
{
    std::ifstream dbc_content(dbc_path);
    std::ostringstream ss;
    ss << dbc_content.rdbuf();
    return ss.str();
}

void print_frames(const can::frame_packet &fp, can::v2c_transcoder &transcoder, int32_t frame_counter)
{
    using namespace std::chrono;

    std::cout << "New frame_packet (from " << frame_counter << " frames):" << std::endl;
    std::cout << "Inside print_frames" << std::endl;
    for (const auto &[ts, frame] : fp)
    {
        auto frame_data = std::bit_cast<int64_t>(frame.data);
        auto t = duration_cast<milliseconds>(ts.time_since_epoch()).count() / 1000.0;

        std::cout << std::fixed
                  << " can_frame at t: " << t << "s, can_id: " << frame.can_id << std::endl;

        auto msg = transcoder.find_message(frame.can_id);
        for (const auto &sig : msg->signals(frame_data))
            std::cout << "  " << sig.name() << ": " << (int64_t)sig.decode(frame_data) << std::endl;
    }
}
int main()
{
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    can::v2c_transcoder transcoder;
    auto start = std::chrono::system_clock::now();
    // bool parsed = can::parse_dbc(read_file("/home/devuser/AOX/ROS2/src/can-utils/example/example.dbc"), std::ref(transcoder));
    // bool parsed = can::parse_dbc(read_file("/home/devuser/AOX/ROS2/mspc_set_msgs.dbc"), std::ref(transcoder));
    bool parsed = can::parse_dbc(read_file("/home/devuser/AOX/ROS2/mspc_fdbk_msgs.dbc"), std::ref(transcoder));
    auto end = std::chrono::system_clock::now();
    std::cout << "Parsed DBC in " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    if (!parsed)
        std::cout << "Parsing not successful" << std::endl;
    else
        std::cout << "Parsing successful!!" << std::endl;
    int32_t frame_counter = 0;

    // Create a socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s == -1)
    {
        perror("Socket creation error");
        return EXIT_FAILURE;
    }

    std::strcpy(ifr.ifr_name, "can_qemu");

    // Find the interface index using ioctl
    if (ioctl(s, SIOCGIFINDEX, &ifr) == -1)
    {
        perror("ioctl error");
        close(s);
        return EXIT_FAILURE;
    }

    // Set the address parameters
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind the socket to the CAN interface
    if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) == -1)
    {
        perror("Bind error");
        close(s);
        return EXIT_FAILURE;
    }

    // Receive a CAN frame
    struct can_frame frame;
    can_frame my_can_frame;
    while (true)
    {
        ssize_t nbytes = read(s, &frame, sizeof(struct can_frame));

        if (nbytes < 0)
        {
            perror("Read error");
        }
        else if (nbytes < sizeof(struct can_frame))
        {
            std::cerr << "Incomplete CAN frame received" << std::endl;
        }
        else
        {
            // Process the received CAN frame
            std::cout << "Received CAN frame:" << std::endl;
            std::cout << "  ID: 0x" << std::hex << frame.can_id << std::dec << std::endl;
            std::cout << "  DLC: " << static_cast<int>(frame.can_dlc) << std::endl;
            std::cout << "  Data: ";
            for (int i = 0; i < frame.can_dlc; ++i)
            {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frame.data[i]) << " ";
            }
            std::cout << std::endl;
        }

        // can_frame frame = bufferToCanFrame(buffer);
        // std::cout << "Outside bufferToCanFrame");
        frame_counter++;
        std::cout << "Transcoding abt to start!" << std::endl;

        auto fp = transcoder.transcode(std::chrono::system_clock::now(), frame);

        std::cout << "Transcoding over!" << std::endl;
        auto frame_data = std::bit_cast<int64_t>(frame.data);
        // auto t = duration_cast<milliseconds>(ts.time_since_epoch()).count() / 1000.0;

        auto msg = transcoder.find_message(frame.can_id);
        for (const auto &sig : msg->signals(frame_data))
            std::cout << "  " << sig.name() << ": " << (int64_t)sig.decode(frame_data) << std::endl;

        if (fp.empty())
            continue;
        // print_frames(fp, transcoder, frame_counter);
        frame_counter = 0;
    }
    // Close the socket
    close(s);
    return EXIT_SUCCESS;
}
