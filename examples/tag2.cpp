#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_client.hpp"

#include "tractor/comms/serial.hpp"

#include "echo/echo.hpp"

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static std::atomic_bool running = true;
static std::atomic<std::int32_t> gnss_auth_status = 0;
static std::atomic<std::int32_t> gnss_warning = 0;

static void signal_handler(int) { running = false; }

struct PHTGData {
    std::string date;
    std::string time;
    std::string system;
    std::string service;
    int auth_result = 0;
    int warning = 0;
};

bool export_ddop_to_xml(std::shared_ptr<isobus::DeviceDescriptorObjectPool> ddop, const std::string &filename) {
    if (!ddop) {
        std::cerr << "Error: DDOP is null\n";
        return false;
    }

    std::string xmlContent;
    if (!ddop->generate_task_data_iso_xml(xmlContent)) {
        std::cerr << "Error: Failed to generate ISOXML from DDOP\n";
        return false;
    }

    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing\n";
        return false;
    }

    outFile << xmlContent;
    outFile.close();

    std::cout << "DDOP exported successfully to " << filename << "\n";
    return true;
}

static bool validate_checksum(const std::string &sentence) {
    size_t star_pos = sentence.find('*');
    if (star_pos == std::string::npos || star_pos + 2 >= sentence.length()) {
        return false;
    }

    std::uint8_t calc_cs = 0;
    for (size_t i = 1; i < star_pos; i++) {
        calc_cs ^= static_cast<std::uint8_t>(sentence[i]);
    }

    std::string cs_str = sentence.substr(star_pos + 1, 2);
    int recv_cs = std::stoi(cs_str, nullptr, 16);

    return calc_cs == recv_cs;
}

static bool parse_phtg(const std::string &sentence, PHTGData &data) {
    if (sentence.size() < 5) {
        return false;
    }
    if (sentence.substr(0, 5) != "$PHTG") {
        return false;
    }
    if (!validate_checksum(sentence)) {
        return false;
    }

    size_t star_pos = sentence.find('*');
    if (star_pos == std::string::npos) {
        return false;
    }

    std::string body = sentence.substr(6, star_pos - 6);

    std::stringstream ss(body);
    std::string token;
    int field = 0;

    while (std::getline(ss, token, ',')) {
        switch (field) {
        case 0:
            data.date = token;
            break;
        case 1:
            data.time = token;
            break;
        case 2:
            data.system = token;
            break;
        case 3:
            data.service = token;
            break;
        case 4:
            data.auth_result = token.empty() ? 0 : std::stoi(token);
            break;
        case 5:
            data.warning = token.empty() ? 0 : std::stoi(token);
            break;
        default:
            break;
        }
        field++;
    }

    return field >= 6;
}

static void process_nmea_line(const std::string &line) {
    if (line.size() >= 5 && line.substr(0, 5) == "$PHTG") {
        PHTGData phtg;
        if (parse_phtg(line, phtg)) {
            gnss_auth_status.store(phtg.auth_result);
            gnss_warning.store(phtg.warning);
        }
    }
}

// DDOP object IDs for HASHTAG sensor
enum class HashtagDDOPObjectIDs : std::uint16_t {
    Device = 0,
    MainDeviceElement = 1,
    SensorElement = 2,

    AuthResultPD = 10,

    DeviceTotalTime = 20,
    ActualWorkState = 21,

    RawPresentation = 50,
    SurfacePresentation = 51,
    TimePresentation = 52
};

static constexpr std::uint16_t ELEMENT_NUMBER = 1;

// Proprietary DDI range: 57344..65534
static constexpr std::uint16_t DDI_AUTH_RESULT = 65432;

static bool request_value_command_callback(std::uint16_t elementNumber, std::uint16_t DDI, std::int32_t &value,
                                           void *) {
    if (elementNumber != ELEMENT_NUMBER) {
        value = 0;
        return true;
    }

    switch (DDI) {
    case DDI_AUTH_RESULT:
        value = gnss_auth_status.load();
        break;
    default:
        value = 0;
        break;
    }

    return true;
}

static bool default_process_data_requested_callback(short unsigned int elm, short unsigned int ddi,
                                                    isobus::DefaultProcessDataSettings &returnedSettings, void *) {
    returnedSettings.timeTriggerInterval_ms = 5500;
    returnedSettings.changeThreshold = 0;
    returnedSettings.enableTimeTrigger = true;
    returnedSettings.enableChangeThresholdTrigger = true;
    returnedSettings.enableMaximumWithinThresholdTrigger = false;
    returnedSettings.enableMinimumWithinThresholdTrigger = false;
    returnedSettings.enableDistanceTrigger = false;
    return true;
}

static bool value_command_callback(std::uint16_t, std::uint16_t, std::int32_t, void *) { return true; }

static bool create_ddop(std::shared_ptr<isobus::DeviceDescriptorObjectPool> pool, isobus::NAME clientName) {
    if (!pool) {
        return false;
    }

    pool->clear();

    constexpr std::array<std::uint8_t, 7> localizationData = {'H', 'A', 'S', 'H', 'T', 'G', 0xFF};

    bool ok = true;

    ok &= pool->add_device("HASHTAG", "0.4.12", "HASHTAG-SENSOR", "HTS0.0.12", localizationData,
                           std::vector<std::uint8_t>(), clientName.get_full_name());

    ok &= pool->add_device_element("WURDevice", 0, static_cast<std::uint16_t>(HashtagDDOPObjectIDs::Device),
                                   isobus::task_controller_object::DeviceElementObject::Type::Device,
                                   static_cast<std::uint16_t>(HashtagDDOPObjectIDs::MainDeviceElement));

    ok &= pool->add_device_value_presentation("mm", 0, 1.0f, 0,
                                              static_cast<std::uint16_t>(HashtagDDOPObjectIDs::SurfacePresentation));

    ok &= pool->add_device_value_presentation("minutes", 0, 1.0f, 0,
                                              static_cast<std::uint16_t>(HashtagDDOPObjectIDs::TimePresentation));

    ok &= pool->add_device_value_presentation("raw", 0, 1.0f, 0,
                                              static_cast<std::uint16_t>(HashtagDDOPObjectIDs::RawPresentation));

    ok &= pool->add_device_process_data("Hashtag DDI #1", DDI_AUTH_RESULT,
                                        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::RawPresentation),
                                        static_cast<std::uint8_t>(3), static_cast<std::uint8_t>(9),
                                        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::AuthResultPD));

    ok &= pool->add_device_process_data(
        "Actual Work State", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState),
        isobus::NULL_OBJECT_ID, static_cast<std::uint8_t>(1), static_cast<std::uint8_t>(8),
        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::ActualWorkState));

    ok &= pool->add_device_process_data(
        "Total Time", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::EffectiveTotalTime),
        isobus::NULL_OBJECT_ID, static_cast<std::uint8_t>(1), static_cast<std::uint8_t>(16),
        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::DeviceTotalTime));

    if (!ok) {
        return false;
    }

    auto sensorElement = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
        pool->get_object_by_id(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::SensorElement)));

    auto mainElement = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
        pool->get_object_by_id(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::MainDeviceElement)));

    if (sensorElement) {
        mainElement->add_reference_to_child_object(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::SensorElement));
        mainElement->add_reference_to_child_object(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::ActualWorkState));
        sensorElement->add_reference_to_child_object(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::AuthResultPD));
        sensorElement->add_reference_to_child_object(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::DeviceTotalTime));
    }

    return true;
}

int main(int argc, char **argv) {

    const char *serial_device = "/tmp/ttyV0";
    int serial_baud = 115200;

    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
            std::cout << "Usage: " << argv[0] << " [serial_device] [serial_baud]\n";
            return 0;
        } else if (i == 1) {
            serial_device = argv[i];
        } else if (i == 2) {
            serial_baud = std::atoi(argv[i]);
        }
    }

    std::signal(SIGINT, signal_handler);

    std::cout << "HASHTAG Tractor Sensor TC Client\n";
    std::cout << "Serial: " << serial_device << " @ " << serial_baud << "\n";

    auto nmea_serial = std::make_shared<tractor::comms::Serial>(serial_device, serial_baud);
    nmea_serial->on_line([](const std::string &line) { process_nmea_line(line); });
    nmea_serial->on_connection([](bool connected) {
        if (connected)
            std::cout << "Serial connected\n";
        else
            std::cout << "Serial disconnected\n";
    });
    nmea_serial->on_error([](const std::string &err) { std::cerr << "Serial error: " << err << "\n"; });

    if (!nmea_serial->start()) {
        std::cerr << "Failed to start serial\n";
        return 1;
    }

    auto canDriver = std::make_shared<isobus::SocketCANInterface>("can0");
    if (!canDriver) {
        std::cerr << "No CAN driver\n";
        return 2;
    }

    isobus::CANHardwareInterface::set_number_of_can_channels(1);
    isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);

    if ((!isobus::CANHardwareInterface::start()) || (!canDriver->get_is_valid())) {
        std::cerr << "Failed to start CAN\n";
        return 3;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    isobus::NAME name(0);
    name.set_arbitrary_address_capable(true);
    name.set_industry_group(2);
    name.set_device_class(0);
    name.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::ObjectDetectionSensor));
    name.set_identity_number(42);
    name.set_ecu_instance(0);
    name.set_function_instance(0);
    name.set_device_class_instance(0);
    name.set_manufacturer_code(1407);

    const isobus::NAMEFilter filterTC(isobus::NAME::NAMEParameters::FunctionCode,
                                      static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));
    const isobus::NAMEFilter filterTCInstance(isobus::NAME::NAMEParameters::FunctionInstance, 0);

    const std::vector<isobus::NAMEFilter> tcFilters = {filterTC, filterTCInstance};

    auto ecu = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(name, 0);
    auto partnerTC = isobus::CANNetworkManager::CANNetwork.create_partnered_control_function(0, tcFilters);

    auto tcClient = std::make_shared<isobus::TaskControllerClient>(partnerTC, ecu, nullptr);

    auto ddop = std::make_shared<isobus::DeviceDescriptorObjectPool>();
    if (!create_ddop(ddop, ecu->get_NAME())) {
        std::cerr << "Failed to create DDOP\n";
        return 4;
    }

    // tcClient->add_default_process_data_requested_callback(default_process_data_requested_callback, nullptr);

    tcClient->add_request_value_callback(request_value_command_callback, nullptr);
    tcClient->add_value_command_callback(value_command_callback, nullptr);

    tcClient->configure(ddop, 0, 0, 0,
                        true, // supports documentation (so TC can log you)
                        false, false, false, true);

    tcClient->initialize(true);
    std::cout << "TC Client started\n";

    std::int32_t lastAuth = gnss_auth_status.load();
    std::int32_t lastWarn = gnss_warning.load();

    std::string filenameee = "hashtag_fromcode.xml";
    const char *xml_export_filename = filenameee.c_str();
    export_ddop_to_xml(ddop, xml_export_filename);

    while (running) {
        auto a = gnss_auth_status.load();
        auto w = gnss_warning.load();

        if (a != lastAuth) {
            tcClient->on_value_changed_trigger(ELEMENT_NUMBER, DDI_AUTH_RESULT);
            lastAuth = a;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cout << "Shutting down...\n";
    nmea_serial->stop();
    tcClient->terminate();
    isobus::CANHardwareInterface::stop();
    return 0;
}
