#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_client.hpp"

#include "tractor/comms/serial.hpp"

#include "echo/echo.hpp"
#include "echo/format.hpp"
#include "echo/widget.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static std::atomic_bool running = true;

// -----------------------------------------------------------------------------
// Runtime values exposed to ISOBUS TC / as-applied map
// -----------------------------------------------------------------------------

static std::atomic<std::int32_t> gnss_constellation = 0;
static std::atomic<std::int32_t> gnss_auth_system = 0;
static std::atomic<std::int32_t> gnss_auth_service = 0;
static std::atomic<std::int32_t> gnss_auth_status = 0;

// Current work state: 0 = not working, 1 = working
static std::atomic<std::int32_t> current_work_state = 0;

// Data sending frequency in milliseconds
static std::atomic<std::uint32_t> send_frequency_ms = 1000;

static void signal_handler(int) { running.store(false); }

// -----------------------------------------------------------------------------
// PHTG protocol model
// -----------------------------------------------------------------------------

enum class PHTGSystem : std::int32_t { HAS = 0, OSNMA = 1, HO = 2, GPS = 3, GAL = 4, Unknown = -1 };

enum class PHTGService : std::int32_t {
    AllOk = 0,
    AuthenticationFailed = 1,
    NoInformationAvailableInSIS = 2,
    KeyOrCertificateAvailableButTimedOut = 3,
    NoKeyOrCertificateAvailable = 4,
    Unknown = -1
};

enum class PHTGStatus : std::int32_t {
    KeyWillTimeOut = 0,
    CertificateWillTimeOut = 1,
    KeysAreTimedOut = 2,
    CertificatesAreTimedOut = 3,
    Unknown = -1
};

struct PHTGData {
    std::string date;
    std::string time;

    std::string constellation_text;
    std::string auth_system_text;

    PHTGSystem constellation = PHTGSystem::Unknown;
    PHTGSystem auth_system = PHTGSystem::Unknown;

    PHTGService service = PHTGService::Unknown;
    PHTGStatus status = PHTGStatus::Unknown;
};

static std::string trim_line_endings(std::string value) {
    while (!value.empty() && (value.back() == '\r' || value.back() == '\n')) {
        value.pop_back();
    }
    return value;
}

static PHTGSystem parse_phtg_system(const std::string &system) {
    if (system == "HAS")
        return PHTGSystem::HAS;
    if (system == "OSNMA")
        return PHTGSystem::OSNMA;
    if (system == "HO")
        return PHTGSystem::HO;
    if (system == "GPS")
        return PHTGSystem::GPS;
    if (system == "GAL")
        return PHTGSystem::GAL;
    return PHTGSystem::Unknown;
}

static bool parse_int_field(const std::string &token, int &value) {
    if (token.empty())
        return false;
    try {
        size_t consumed = 0;
        int parsed = std::stoi(token, &consumed, 10);
        if (consumed != token.size())
            return false;
        value = parsed;
        return true;
    } catch (const std::exception &) {
        return false;
    }
}

static PHTGService parse_phtg_service(const std::string &token) {
    int value = -1;
    if (!parse_int_field(token, value))
        return PHTGService::Unknown;
    switch (value) {
    case 0:
        return PHTGService::AllOk;
    case 1:
        return PHTGService::AuthenticationFailed;
    case 2:
        return PHTGService::NoInformationAvailableInSIS;
    case 3:
        return PHTGService::KeyOrCertificateAvailableButTimedOut;
    case 4:
        return PHTGService::NoKeyOrCertificateAvailable;
    default:
        return PHTGService::Unknown;
    }
}

static PHTGStatus parse_phtg_status(const std::string &token) {
    int value = -1;
    if (!parse_int_field(token, value))
        return PHTGStatus::Unknown;
    switch (value) {
    case 0:
        return PHTGStatus::KeyWillTimeOut;
    case 1:
        return PHTGStatus::CertificateWillTimeOut;
    case 2:
        return PHTGStatus::KeysAreTimedOut;
    case 3:
        return PHTGStatus::CertificatesAreTimedOut;
    default:
        return PHTGStatus::Unknown;
    }
}

static bool validate_checksum(const std::string &raw_sentence) {
    const auto sentence = trim_line_endings(raw_sentence);
    const size_t star_pos = sentence.find('*');

    if (star_pos == std::string::npos || star_pos + 2 >= sentence.length())
        return false;
    if (sentence.empty() || sentence[0] != '$')
        return false;

    std::uint8_t calc_cs = 0;
    for (size_t i = 1; i < star_pos; i++) {
        calc_cs ^= static_cast<std::uint8_t>(sentence[i]);
    }

    const std::string cs_str = sentence.substr(star_pos + 1, 2);
    int recv_cs = 0;
    try {
        recv_cs = std::stoi(cs_str, nullptr, 16);
    } catch (const std::exception &) {
        return false;
    }

    return calc_cs == static_cast<std::uint8_t>(recv_cs);
}

static bool parse_phtg(const std::string &raw_sentence, PHTGData &data) {
    const auto sentence = trim_line_endings(raw_sentence);

    if (sentence.size() < 7)
        return false;
    if (sentence.substr(0, 6) != "$PHTG,")
        return false;

    if (!validate_checksum(sentence)) {
        std::cerr << "Invalid PHTG checksum: " << sentence << "\n";
        return false;
    }

    const size_t star_pos = sentence.find('*');
    if (star_pos == std::string::npos)
        return false;

    const std::string body = sentence.substr(6, star_pos - 6);

    std::stringstream ss(body);
    std::string token;
    std::vector<std::string> fields;
    while (std::getline(ss, token, ',')) {
        fields.push_back(token);
    }

    if (fields.size() < 6) {
        std::cerr << "Invalid PHTG field count: " << fields.size() << " sentence: " << sentence << "\n";
        return false;
    }

    data.date = fields[0];
    data.time = fields[1];
    data.constellation_text = fields[2];
    data.auth_system_text = fields[3];
    data.constellation = parse_phtg_system(fields[2]);
    data.auth_system = parse_phtg_system(fields[3]);
    data.service = parse_phtg_service(fields[4]);
    data.status = parse_phtg_status(fields[5]);

    if (data.constellation == PHTGSystem::Unknown)
        std::cerr << "Unknown PHTG constellation: " << fields[2] << "\n";
    if (data.auth_system == PHTGSystem::Unknown)
        std::cerr << "Unknown PHTG auth system: " << fields[3] << "\n";
    if (data.service == PHTGService::Unknown)
        std::cerr << "Unknown PHTG service: " << fields[4] << "\n";
    if (data.status == PHTGStatus::Unknown)
        std::cerr << "Unknown PHTG status: " << fields[5] << "\n";

    return true;
}

static void process_nmea_line(const std::string &line) {
    if (line.size() >= 6 && line.substr(0, 6) == "$PHTG,") {
        PHTGData phtg;
        if (parse_phtg(line, phtg)) {
            gnss_constellation.store(static_cast<std::int32_t>(phtg.constellation));
            gnss_auth_system.store(static_cast<std::int32_t>(phtg.auth_system));
            gnss_auth_service.store(static_cast<std::int32_t>(phtg.service));
            gnss_auth_status.store(static_cast<std::int32_t>(phtg.status));

            // std::cout << "PHTG received: " << phtg.date << " " << phtg.time
            //           << " constellation=" << phtg.constellation_text << " auth_system=" << phtg.auth_system_text
            //           << " service=" << static_cast<std::int32_t>(phtg.service)
            //           << " status=" << static_cast<std::int32_t>(phtg.status) << "\n";
        }
    }
}

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

// -----------------------------------------------------------------------------
// DDOP
//
// Structure modeled on the working sprayer: a real Device element + a
// functional sub-element holding the process data. Many TC servers will not
// start logging if everything hangs directly off the device root.
// -----------------------------------------------------------------------------

enum class HashtagDDOPObjectIDs : std::uint16_t {
    Device = 0,
    MainDeviceElement = 1,
    SensorElement = 2,

    AuthConstellationPD = 10,
    AuthSystemPD = 11,
    AuthServicePD = 12,
    AuthStatusPD = 13,

    DeviceTotalTime = 20,
    ActualWorkState = 21,
    RequestDefaultProcessData = 22,

    RawPresentation = 50,
    TimePresentation = 52
};

// Element number that we report values against (must match the element index
// of SensorElement, which is the 2nd element added → index 1)
static constexpr std::uint16_t SENSOR_ELEMENT_NUMBER = 1;

// Proprietary DDI range: 57344..65534
static constexpr std::uint16_t DDI_AUTH_CONSTELLATION = 65430;
static constexpr std::uint16_t DDI_AUTH_SYSTEM = 65431;
static constexpr std::uint16_t DDI_AUTH_SERVICE = 65432;
static constexpr std::uint16_t DDI_AUTH_STATUS = 65433;

static std::uint8_t member_of_default_set_property() {
    return static_cast<std::uint8_t>(
        isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet);
}

static std::uint8_t member_of_default_set_and_settable_property() {
    return static_cast<std::uint8_t>(
               isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet) |
           static_cast<std::uint8_t>(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable);
}

static std::uint8_t trigger_on_change_and_time() {
    return static_cast<std::uint8_t>(
               isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange) |
           static_cast<std::uint8_t>(
               isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::TimeInterval);
}

static std::uint8_t trigger_on_change() {
    return static_cast<std::uint8_t>(
        isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange);
}

static std::uint8_t trigger_total() {
    return static_cast<std::uint8_t>(
        isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::Total);
}

static bool request_value_command_callback(std::uint16_t /*elementNumber*/, std::uint16_t DDI, std::int32_t &value,
                                           void * /*parent*/) {
    switch (DDI) {
    case DDI_AUTH_CONSTELLATION:
        value = gnss_constellation.load();
        break;
    case DDI_AUTH_SYSTEM:
        value = gnss_auth_system.load();
        break;
    case DDI_AUTH_SERVICE:
        value = gnss_auth_service.load();
        break;
    case DDI_AUTH_STATUS:
        value = gnss_auth_status.load();
        break;
    case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState):
        value = current_work_state.load();
        break;
    case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::EffectiveTotalTime):
        value = 0;
        break;
    case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::RequestDefaultProcessData):
        value = 0;
        break;
    default:
        value = 0;
        break;
    }
    return true;
}

static bool value_command_callback(std::uint16_t, std::uint16_t, std::int32_t, void *) { return true; }

static bool create_ddop(std::shared_ptr<isobus::DeviceDescriptorObjectPool> pool, isobus::NAME clientName) {
    if (!pool)
        return false;
    pool->clear();

    std::array<std::uint8_t, 7> localizationData = {'e', 'n', 0x50, 0x00, 0x55, 0x55, 0xFF};

    bool ok = true;
    std::uint16_t elementCounter = 0;

    ok &= pool->add_device("HASHTAG", "0.0.14", "HASHTAG-SENSOR", "HTS0.0.14", localizationData,
                           std::vector<std::uint8_t>(), clientName.get_full_name());

    // --- Element 0: root device element -------------------------------------
    ok &= pool->add_device_element("WURDevice", elementCounter, 0,
                                   isobus::task_controller_object::DeviceElementObject::Type::Device,
                                   static_cast<std::uint16_t>(HashtagDDOPObjectIDs::MainDeviceElement));
    elementCounter++;

    // --- Presentations ------------------------------------------------------
    ok &= pool->add_device_value_presentation("raw", 0, 1.0f, 0,
                                              static_cast<std::uint16_t>(HashtagDDOPObjectIDs::RawPresentation));

    ok &= pool->add_device_value_presentation("minutes", 0, 1.0f, 0,
                                              static_cast<std::uint16_t>(HashtagDDOPObjectIDs::TimePresentation));

    // --- Device-level process data -----------------------------------------
    ok &= pool->add_device_process_data(
        "Request Default PD", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::RequestDefaultProcessData),
        isobus::NULL_OBJECT_ID, 0, trigger_total(),
        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::RequestDefaultProcessData));

    ok &= pool->add_device_process_data("Total Time",
                                        static_cast<std::uint16_t>(isobus::DataDescriptionIndex::EffectiveTotalTime),
                                        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::TimePresentation),
                                        member_of_default_set_and_settable_property(), trigger_total(),
                                        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::DeviceTotalTime));

    // --- Element 1: sensor function element ---------------------------------
    // This is the element TC will log values against. Hanging process data
    // off a Function child (rather than the Device root) matches what the
    // working sprayer does and is what most TC servers expect for recording.
    ok &= pool->add_device_element("GNSS Auth Sensor", elementCounter,
                                   static_cast<std::uint16_t>(HashtagDDOPObjectIDs::MainDeviceElement),
                                   isobus::task_controller_object::DeviceElementObject::Type::Function,
                                   static_cast<std::uint16_t>(HashtagDDOPObjectIDs::SensorElement));
    elementCounter++;

    // --- PHTG values (logged) -----------------------------------------------
    ok &= pool->add_device_process_data("PHTG Constellation", DDI_AUTH_CONSTELLATION,
                                        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::RawPresentation),
                                        member_of_default_set_property(), trigger_on_change_and_time(),
                                        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::AuthConstellationPD));

    ok &= pool->add_device_process_data("PHTG Auth System", DDI_AUTH_SYSTEM,
                                        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::RawPresentation),
                                        member_of_default_set_property(), trigger_on_change_and_time(),
                                        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::AuthSystemPD));

    ok &= pool->add_device_process_data("PHTG Auth Service", DDI_AUTH_SERVICE,
                                        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::RawPresentation),
                                        member_of_default_set_property(), trigger_on_change_and_time(),
                                        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::AuthServicePD));

    ok &= pool->add_device_process_data("PHTG Auth Status", DDI_AUTH_STATUS,
                                        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::RawPresentation),
                                        member_of_default_set_property(), trigger_on_change_and_time(),
                                        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::AuthStatusPD));

    ok &= pool->add_device_process_data(
        "Actual Work State", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState),
        isobus::NULL_OBJECT_ID, member_of_default_set_property(), trigger_on_change_and_time(),
        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::ActualWorkState));

    if (!ok) {
        std::cerr << "Failed to add one or more DDOP objects\n";
        return false;
    }

    // --- Wire up parent → child references ---------------------------------
    auto mainElement = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
        pool->get_object_by_id(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::MainDeviceElement)));

    auto sensorElement = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
        pool->get_object_by_id(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::SensorElement)));

    if (!mainElement || !sensorElement) {
        std::cerr << "DDOP error: required elements missing\n";
        return false;
    }

    // Device-level
    mainElement->add_reference_to_child_object(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::DeviceTotalTime));
    mainElement->add_reference_to_child_object(
        static_cast<std::uint16_t>(HashtagDDOPObjectIDs::RequestDefaultProcessData));

    // Sensor-level (this is what TC will record)
    sensorElement->add_reference_to_child_object(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::ActualWorkState));
    sensorElement->add_reference_to_child_object(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::AuthConstellationPD));
    sensorElement->add_reference_to_child_object(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::AuthSystemPD));
    sensorElement->add_reference_to_child_object(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::AuthServicePD));
    sensorElement->add_reference_to_child_object(static_cast<std::uint16_t>(HashtagDDOPObjectIDs::AuthStatusPD));

    return true;
}

int main(int argc, char **argv) {

    gnss_constellation.store(0);
    gnss_auth_system.store(0);
    gnss_auth_service.store(0);
    gnss_auth_status.store(0);
    current_work_state.store(1);

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
    std::signal(SIGTERM, signal_handler);

    std::cout << "HASHTAG Tractor Sensor TC Client\n";
    std::cout << "Serial: " << serial_device << " @ " << serial_baud << "\n";

    auto nmea_serial = std::make_shared<tractor::comms::Serial>(serial_device, serial_baud);

    nmea_serial->on_line([](const std::string &line) { process_nmea_line(line); });

    nmea_serial->on_connection(
        [](bool connected) { std::cout << (connected ? "Serial connected\n" : "Serial disconnected\n"); });

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

    // --- NAME ---------------------------------------------------------------
    // Aligned with the working sprayer: industry group 2, an ag device class,
    // and a function code TCs are comfortable recording from.
    isobus::NAME name(0);
    name.set_arbitrary_address_capable(true);
    name.set_industry_group(2);
    name.set_device_class(6); // matches working sprayer
    name.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::RateControl));
    name.set_identity_number(42);
    name.set_ecu_instance(0);
    name.set_function_instance(0);
    name.set_device_class_instance(0);
    name.set_manufacturer_code(1407);

    // --- TC partner filters (4-filter set, like the working sprayer) -------
    const isobus::NAMEFilter filterTC(isobus::NAME::NAMEParameters::FunctionCode,
                                      static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));
    const isobus::NAMEFilter filterTCInstance(isobus::NAME::NAMEParameters::FunctionInstance, 0);
    const isobus::NAMEFilter filterTCIndustryGroup(
        isobus::NAME::NAMEParameters::IndustryGroup,
        static_cast<std::uint8_t>(isobus::NAME::IndustryGroup::AgriculturalAndForestryEquipment));
    const isobus::NAMEFilter filterTCDeviceClass(isobus::NAME::NAMEParameters::DeviceClass,
                                                 static_cast<std::uint8_t>(isobus::NAME::DeviceClass::NonSpecific));

    const std::vector<isobus::NAMEFilter> tcFilters = {filterTC, filterTCInstance, filterTCIndustryGroup,
                                                       filterTCDeviceClass};

    auto ecu = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(name, 0);
    auto partnerTC = isobus::CANNetworkManager::CANNetwork.create_partnered_control_function(0, tcFilters);

    auto tcClient = std::make_shared<isobus::TaskControllerClient>(partnerTC, ecu, nullptr);
    auto ddop = std::make_shared<isobus::DeviceDescriptorObjectPool>();

    bool tcClientStarted = false;
    std::int32_t lastConstellation = gnss_constellation.load();
    std::int32_t lastAuthSystem = gnss_auth_system.load();
    std::int32_t lastAuthService = gnss_auth_service.load();
    std::int32_t lastAuthStatus = gnss_auth_status.load();
    std::int32_t lastWorkState = current_work_state.load();

    echo::box("Tractor Hashtag Sensor TC Client", echo::BoxStyle::Double);
    std::cout << "Waiting for TC server...\n\n";

    auto last_toggle_time = std::chrono::steady_clock::now();
    const auto toggle_interval = std::chrono::seconds(5);

    while (running.load()) {
        if (!tcClientStarted) {
            if (create_ddop(ddop, ecu->get_NAME())) {
                tcClient->configure(ddop,
                                    1,     // numberBoomsSupported
                                    1,     // numberSectionsSupported
                                    1,     // numberChannelsSupportedForPositionBasedControl
                                    true,  // reportToTCSupportsDocumentation
                                    false, // reportToTCSupportsTCGEOWithoutPositionBasedControl
                                    false, // reportToTCSupportsTCGEOWithPositionBasedControl
                                    false, // reportToTCSupportsPeerControlAssignment
                                    true   // reportToTCSupportsImplementSectionControl
                );

                tcClient->add_request_value_callback(request_value_command_callback, nullptr);
                tcClient->add_value_command_callback(value_command_callback, nullptr);

                tcClient->initialize(true);
                tcClientStarted = true;

                export_ddop_to_xml(ddop, "tag_fromcode.xml");
                std::cout << "TC Client initialized successfully\n";
            } else {
                std::cerr << "Failed to create DDOP\n";
                break;
            }
        }

        auto now = std::chrono::steady_clock::now();
        // if (now - last_toggle_time >= toggle_interval) {
        //     current_work_state.store(1 - current_work_state.load());
        //     last_toggle_time = now;
        // }

        auto constellation = gnss_constellation.load();
        auto authSystem = gnss_auth_system.load();
        auto authService = gnss_auth_service.load();
        auto authStatus = gnss_auth_status.load();
        auto workState = current_work_state.load();

        if (constellation != lastConstellation || (now - last_toggle_time >= toggle_interval)) {
            echo("Constellation: ",
                 echo::format::String(std::to_string(constellation).c_str()).bg(0, 255, 0).black().bold());
            tcClient->on_value_changed_trigger(SENSOR_ELEMENT_NUMBER, DDI_AUTH_CONSTELLATION);
            lastConstellation = constellation;
        }
        if (authSystem != lastAuthSystem || (now - last_toggle_time >= toggle_interval)) {
            echo("Auth System: ",
                 echo::format::String(std::to_string(authSystem).c_str()).bg(0, 255, 0).black().bold());
            tcClient->on_value_changed_trigger(SENSOR_ELEMENT_NUMBER, DDI_AUTH_SYSTEM);
            lastAuthSystem = authSystem;
        }
        if (authService != lastAuthService || (now - last_toggle_time >= toggle_interval)) {
            echo("Auth Service: ",
                 echo::format::String(std::to_string(authService).c_str()).bg(0, 255, 0).black().bold());
            tcClient->on_value_changed_trigger(SENSOR_ELEMENT_NUMBER, DDI_AUTH_SERVICE);
            lastAuthService = authService;
        }
        if (authStatus != lastAuthStatus || (now - last_toggle_time >= toggle_interval)) {
            echo("Auth Status: ",
                 echo::format::String(std::to_string(authStatus).c_str()).bg(0, 255, 0).black().bold());
            tcClient->on_value_changed_trigger(SENSOR_ELEMENT_NUMBER, DDI_AUTH_STATUS);
            lastAuthStatus = authStatus;
        }
        if (workState != lastWorkState || (now - last_toggle_time >= toggle_interval)) {
            echo("Work State: ", echo::format::String(workState ? "ON" : "OFF").bg(0, 255, 0).black().bold());
            tcClient->on_value_changed_trigger(
                SENSOR_ELEMENT_NUMBER, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState));
            lastWorkState = workState;
        }

        // std::cout << "\r  Work: [" << (workState ? " ON " : "OFF ") << "]  Const: " << constellation
        //           << "  Sys: " << authSystem << "  Svc: " << authService << "  Sts: " << authStatus << "  "
        //           << std::flush;
        //
        // echo::format::String pretty_string;
        // if (workState) {
        //     pretty_string = echo::format::String(" [ ON  ] ").bg(0, 255, 0).black().bold();
        // } else {
        //     pretty_string = echo::format::String(" [ OFF ] ").bg(255, 0, 0).black().bold();
        // }
        // echo("WORK STATE ", pretty_string).inplace();

        std::this_thread::sleep_for(std::chrono::milliseconds(send_frequency_ms.load()));
    }

    std::cout << "\nShutting down...\n";

    nmea_serial->stop();
    if (tcClientStarted) {
        tcClient->terminate();
    }
    isobus::CANHardwareInterface::stop();

    return 0;
}
