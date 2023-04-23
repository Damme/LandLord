#include <queue>
#include <atomic>

#include "ros/ros.h"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include "boost/crc.hpp"

#include "spidev_lib++.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <mower_msgs/Status.h>
#include "mower_msgs/MowerControlSrv.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "mower_msgs/ImuRaw.h"
#include "mower_msgs/HighLevelControlSrv.h"
#include "mower_msgs/HighLevelStatus.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include <xbot_msgs/WheelTick.h>

#include "ll_datatypes.h"

//#include <xesc_driver/xesc_driver.h>
//#include <xesc_msgs/XescStateStamped.h>

using namespace rapidjson;

#define NOP 0x00
#define SOF 0x01
#define EOF 0xFF
#define buflen  250

#define WHEEL_DISTANCE_M 0.325 // ?

spi_config_t spi_config;

std::atomic<bool> keep_running(true);

std::queue<std::string> spi_rx_queue;
std::queue<std::string> spi_tx_queue;
boost::mutex spi_rx_queue_mutex;
boost::mutex spi_tx_queue_mutex;

ros::ServiceClient highLevelClient;

ros::Publisher status_pub;
ros::Publisher wheel_tick_pub;

ros::Publisher sensor_imu_pub;
ros::Publisher sensor_mag_pub;

ros::Time last_cmd_vel(0.0);

sensor_msgs::MagneticField sensor_mag_msg;
sensor_msgs::Imu sensor_imu_msg;
mower_msgs::HighLevelStatus last_high_level_status;

bool emergency_high_level = false;
bool emergency_low_level = false;
bool ll_clear_emergency = false;
bool allow_send = false;
float speed_l = 0, speed_r = 0, speed_mow = 0;

void velReceived(const geometry_msgs::Twist::ConstPtr &msg) {
    // TODO: change this to suit Worx wheelticks and worx speed! (0-1023)
    last_cmd_vel = ros::Time::now();
    speed_l = msg->linear.x - 0.5*WHEEL_DISTANCE_M*msg->angular.z;
    speed_r = msg->linear.x + 0.5*WHEEL_DISTANCE_M*msg->angular.z;

    if (speed_l >= 1.0) {
        speed_l = 1.0;
    } else if (speed_l <= -1.0) {
        speed_l = -1.0;
    }
    if (speed_r >= 1.0) {
        speed_r = 1.0;
    } else if (speed_r <= -1.0) {
        speed_r = -1.0;
    }
    ROS_INFO("¤¤¤ velRecieved: %f %f", speed_l, speed_r);
}

void ping_timer(const ros::TimerEvent& event) {
  ROS_INFO("::: Timertest!");
}

bool is_emergency() {
    return emergency_high_level || emergency_low_level;
}

void publishActuators() {
// emergency or timeout -> send 0 speeds
    if (is_emergency()) {
        speed_l = 0;
        speed_r = 0;
        speed_mow = 0;
    }
    if (ros::Time::now() - last_cmd_vel > ros::Duration(1.0)) {
        speed_l = 0;
        speed_r = 0;

    }
    if (ros::Time::now() - last_cmd_vel > ros::Duration(25.0)) {
        speed_l = 0;
        speed_r = 0;
        speed_mow = 0;
    }
    // TODO: Create txqueue message
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res) {
    if (req.mow_enabled && !is_emergency()) {
        speed_mow = 1;
    } else {
        speed_mow = 0;
    }
    ROS_INFO_STREAM("Setting mow enabled to " << speed_mow);
    return true;
}

bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res) {
    ROS_INFO("got setEmergencyStop");
    if (req.emergency) {
        ROS_ERROR_STREAM("Setting emergency!!");
        ll_clear_emergency = false;
    } else {
        ll_clear_emergency = true;
    }
    // Set the high level emergency instantly. Low level value will be set on next update.
    emergency_high_level = req.emergency;
    publishActuators();
    return true;
}

void highLevelStatusReceived(const mower_msgs::HighLevelStatus::ConstPtr &msg) {
    ROS_INFO("got highLevelStatusReceived");
}

void publishStatus() {
    mower_msgs::Status status_msg;
    status_msg.stamp = ros::Time::now();

    status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_INITIALIZING;
    status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_OK;
    

    status_msg.raspberry_pi_power = 1;
    status_msg.gps_power = 1;
    status_msg.esc_power = 1;
    status_msg.rain_detected = 0;
    status_msg.sound_module_available = 0;
    status_msg.sound_module_busy = 0;
    status_msg.ui_board_available = 0;

    for (uint8_t i = 0; i < 5; i++) {
        status_msg.ultrasonic_ranges[i] = 0;
    }

    // overwrite emergency with the LL value.
    /*emergency_low_level = last_ll_status.emergency_bitmask > 0;
    if (!emergency_low_level) {
        // it obviously worked, reset the request
        ll_clear_emergency = false;
    } else {
        ROS_ERROR_STREAM_THROTTLE(1, "Low Level Emergency. Bitmask was: " << (int)last_ll_status.emergency_bitmask);
    }*/

    // True, if high or low level emergency condition is present
    status_msg.emergency = is_emergency();

    status_msg.v_battery = 28;
    status_msg.v_charge = 0;
    status_msg.charge_current = 0;


    //xesc_msgs::XescStateStamped mow_status, left_status, right_status;
    //if(mow_xesc_interface) {
        //mow_xesc_interface->getStatus(mow_status);
    //} else {
        //mow_status.state.connection_state = xesc_msgs::XescState::XESC_CONNECTION_STATE_DISCONNECTED;
    //}
    //left_xesc_interface->getStatus(left_status);
    //right_xesc_interface->getStatus(right_status);

    //convertStatus(mow_status, status_msg.mow_esc_status);
    //convertStatus(left_status, status_msg.left_esc_status);
    //convertStatus(right_status, status_msg.right_esc_status);

    status_msg.mow_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    status_msg.mow_esc_status.tacho = 0;
    status_msg.mow_esc_status.current = 0;
    status_msg.mow_esc_status.temperature_motor = 0;
    status_msg.mow_esc_status.temperature_pcb = 0;

    status_msg.left_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    status_msg.left_esc_status.tacho = 0;
    status_msg.left_esc_status.current = 0;
    status_msg.left_esc_status.temperature_motor = 0;
    status_msg.left_esc_status.temperature_pcb = 0;

    status_msg.right_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    status_msg.right_esc_status.tacho = 0;
    status_msg.right_esc_status.current = 0;
    status_msg.right_esc_status.temperature_motor = 0;
    status_msg.right_esc_status.temperature_pcb = 0;

    status_pub.publish(status_msg);

    xbot_msgs::WheelTick wheel_tick_msg;
    // TODO: set this correctly!
    wheel_tick_msg.wheel_tick_factor = 0;
    wheel_tick_msg.stamp = status_msg.stamp;
    wheel_tick_msg.wheel_ticks_rl = 0;
    wheel_tick_msg.wheel_direction_rl = 0;
    wheel_tick_msg.wheel_ticks_rr = 0;
    wheel_tick_msg.wheel_direction_rr = 0;

    wheel_tick_pub.publish(wheel_tick_msg);
}

void publishActuatorsTimerTask(const ros::TimerEvent &timer_event) {
    ROS_INFO("got publishActuatorsTimerTask");
    publishActuators();
    publishStatus();
}

void spi_thread(std::atomic<bool>& keep_running, ros::NodeHandle& n, const std::string& ll_spi_dev_name) {
    ros::Time::init(); // Initialize ROS time
    
    SPI *SPIdev = NULL;

    char rxBuffer[buflen];
    char txBuffer[buflen];
    bool finishedRx, finishedTx;
    bool receiving;
    uint8_t rxByte, txByte;
    size_t txPos = 0, rxPos = 0;
    
    receiving = false;

    memset(txBuffer, 0x00, buflen);
    spi_config.mode=0;
    spi_config.speed=500000;
    spi_config.delay=0;
    spi_config.bits_per_word=8;

    ROS_INFO_STREAM("Opening spi device: " << ll_spi_dev_name);

    SPIdev = new SPI(ll_spi_dev_name.c_str(), &spi_config);
    if (SPIdev->begin()) {
        ROS_INFO_STREAM("Connecting SPI interface: " << ll_spi_dev_name);
    } else {
        keep_running.store(false);
        ROS_ERROR_STREAM("Error opening spi device - check your udev permissions.");
    }

    ros::Timer timer = n.createTimer(ros::Duration(1.0), ping_timer);

    ros::Duration loopDelay(0, 10000000); // 10ms

    while (ros::ok() && keep_running.load()) {
        std::string txMsg;
        {
            boost::unique_lock<boost::mutex> lock(spi_tx_queue_mutex);
            if (!spi_tx_queue.empty()) {
                txMsg = spi_tx_queue.front();
                spi_tx_queue.pop();
                strncpy(txBuffer, txMsg.c_str(), sizeof(txBuffer) - 1);
                txBuffer[sizeof(txBuffer) - 1] = '\0'; // Ensure null termination
                ROS_INFO(">>> %s", txBuffer);
            }
        }

        receiving = false;
        finishedRx = finishedTx = false;
        memset(rxBuffer, 0x00, buflen);

        while (!finishedRx || !finishedTx) {
            if (strlen(txBuffer) > 0 && txPos == 0) {
                txByte = SOF;
            }

            SPIdev->xfer(&txByte, 1, &rxByte, 1);

            if (receiving) {
                if (rxByte == EOF) {
                    finishedRx = true;
                    rxBuffer[rxPos] = '\0'; // Null-terminate the received string
                    //ROS_INFO("<<< %s", rxBuffer);
                    boost::unique_lock<boost::mutex> lock(spi_rx_queue_mutex);
                    spi_rx_queue.push(rxBuffer);
                    rxPos = 0;
                    continue;
                } else if (rxByte != NOP) {
                    rxBuffer[rxPos++] = rxByte;
                }
            } else if (rxByte == SOF) {
                receiving = true;
            }

            if (txBuffer[txPos] != 0x00) {
                txByte = txBuffer[txPos++];
            } else {
                if (txPos > 0) {
                    txByte = EOF;
                    memset(txBuffer, 0x00, buflen);
                    txPos = 0;
                } else {
                    // Transmit NOP when there is no more data to send
                    txByte = NOP;
                    finishedTx = true;
                }
            }
        }
        
        loopDelay.sleep();

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "worx_comms");

    rapidjson::Document::AllocatorType* allocator;
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer>* writer;

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");
    
    highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>(
            "mower_service/high_level_control");

    std::string ll_spi_dev_name;
    if (!paramNh.getParam("ll_spi_dev_name", ll_spi_dev_name)) {
        ROS_ERROR_STREAM("Error getting ll_spi_dev_name. Quitting.");
        return 1;
    }
    boost::thread spiThread(boost::bind(&spi_thread, std::ref(keep_running), std::ref(n), ll_spi_dev_name));


    status_pub = n.advertise<mower_msgs::Status>("mower/status", 1);
    wheel_tick_pub = n.advertise<xbot_msgs::WheelTick>("mower/wheel_ticks", 1);

    sensor_imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
    sensor_mag_pub = n.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
    ros::ServiceServer mow_service = n.advertiseService("mower_service/mow_enabled", setMowEnabled);
    ros::ServiceServer emergency_service = n.advertiseService("mower_service/emergency", setEmergencyStop);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber high_level_status_sub = n.subscribe("/mower_logic/current_state", 0, highLevelStatusReceived);
    ros::Timer publish_timer = n.createTimer(ros::Duration(0.02), publishActuatorsTimerTask);


    // don't change, we need to wait for arduino to boot before actually sending stuff
    ros::Duration retryDelay(5, 0);
    ros::Duration loopDelay(0, 10000000); // 10ms
    ros::AsyncSpinner spinner(1);

    int counter = 0;
    std::string rxMsg;
    Document document;

    ros::Time last_ping_time = ros::Time::now();

    spinner.start();
    while (ros::ok() && keep_running.load()) {
        ros::Time current_time = ros::Time::now();


        counter++;

        if ((current_time - last_ping_time).toSec() > 5.0) {
            last_ping_time = current_time;

            rapidjson::Document document;
            document.SetObject();

            allocator = &document.GetAllocator();
            rapidjson::Value obj(rapidjson::kObjectType);
            obj.AddMember("ping", counter, *allocator);

            document.AddMember("command", obj, *allocator);

            buffer.Clear();
            writer = new rapidjson::Writer<rapidjson::StringBuffer>(buffer);
            document.Accept(*writer);

            boost::unique_lock<boost::mutex> lock(spi_tx_queue_mutex);
            spi_tx_queue.push(buffer.GetString());
        }

        
        {
            boost::unique_lock<boost::mutex> lock(spi_rx_queue_mutex);
            if (!spi_rx_queue.empty()) {
                rxMsg = spi_rx_queue.front();
                spi_rx_queue.pop();
                //strncpy(txBuffer, txMsg.c_str(), sizeof(txBuffer) - 1);
                //txBuffer[sizeof(txBuffer) - 1] = '\0'; // Ensure null termination
                ROS_INFO("<<< %s", rxMsg.c_str());
            }
        }

        
        ParseResult result = document.Parse(rxMsg.c_str());
        if (result) {
            if (document.HasMember("Boundary")) {
                //ROS_INFO("*** Got Boundary! :D");
            }
        }
        
    

        loopDelay.sleep();
    }

    spinner.stop();
    spiThread.join();

    return 0;
}
