#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_client.hpp"

#include <atomic>
#include <cassert>
#include <csignal>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static std::atomic_bool running = true;
std::int32_t auth_status = 0;

void signal_handler(int) { running = false; }

class SectionControlImplementSimulator {
  public:
    static constexpr std::uint16_t MAX_NUMBER_SECTIONS_SUPPORTED = 256;
    static constexpr std::uint8_t NUMBER_SECTIONS_PER_CONDENSED_MESSAGE = 16;
    static constexpr std::int32_t BOOM_WIDTH = 9144; // 30ft in mm (same as seeder example)

    enum class ImplementDDOPObjectIDs : std::uint16_t {
        Device = 0,
        MainDeviceElement,
        DeviceActualWorkState,
        RequestDefaultProcessData,
        DeviceTotalTime,
        Connector,
        ConnectorXOffset,
        ConnectorYOffset,
        ConnectorType,
        SprayBoom,
        ActualWorkState,
        ActualWorkingWidth,
        AreaTotal,
        SetpointWorkState,
        SectionControlState,
        BoomXOffset,
        BoomYOffset,
        BoomZOffset,
        Section1,
        SectionMax = Section1 + (MAX_NUMBER_SECTIONS_SUPPORTED - 1),
        Section1XOffset,
        SectionXOffsetMax = Section1XOffset + (MAX_NUMBER_SECTIONS_SUPPORTED - 1),
        Section1YOffset,
        SectionYOffsetMax = Section1YOffset + (MAX_NUMBER_SECTIONS_SUPPORTED - 1),
        Section1Width,
        SectionWidthMax = Section1Width + (MAX_NUMBER_SECTIONS_SUPPORTED - 1),
        ActualCondensedWorkingState1To16,
        SetpointCondensedWorkingState1To16,
        LiquidProduct,
        TankCapacity,
        TankVolume,
        LifetimeApplicationVolumeTotal,
        PrescriptionControlState,
        ActualCulturalPractice,
        TargetRate,
        TimePresentation,
        ShortWidthPresentation,
        HashtagParameter
    };

    explicit SectionControlImplementSimulator(std::uint8_t numberOfSections)
        : sectionSetpointStates(numberOfSections, false), sectionSwitchStates(numberOfSections, false) {}

    std::uint8_t get_number_of_sections() const { return static_cast<std::uint8_t>(sectionSetpointStates.size()); }

    bool get_section_actual_state(std::uint8_t index) const {
        if (isAutoMode) {
            return sectionSetpointStates.at(index);
        } else {
            return sectionSwitchStates.at(index);
        }
    }

    std::uint8_t get_actual_number_of_sections_on() const {
        std::uint8_t count = 0;
        for (std::uint8_t i = 0; i < get_number_of_sections(); i++) {
            if (get_section_actual_state(i)) {
                count++;
            }
        }
        return count;
    }

    bool get_section_setpoint_state(std::uint8_t index) const { return sectionSetpointStates.at(index); }

    void set_section_switch_state(std::uint8_t index, bool value) { sectionSwitchStates.at(index) = value; }

    bool get_section_switch_state(std::uint8_t index) const { return sectionSwitchStates.at(index); }

    std::uint32_t get_actual_rate() const {
        bool anySectionOn = get_actual_number_of_sections_on() > 0;
        return targetRate * (anySectionOn ? 1 : 0);
    }

    std::uint32_t get_target_rate() const { return targetRate; }

    bool get_setpoint_work_state() const { return setpointWorkState; }

    void set_is_mode_auto(bool isAuto) { isAutoMode = isAuto; }

    bool get_is_mode_auto() const { return isAutoMode; }

    std::uint32_t get_prescription_control_state() const { return static_cast<std::uint32_t>(get_is_mode_auto()); }

    std::uint32_t get_section_control_state() const { return static_cast<std::uint32_t>(get_is_mode_auto()); }

    bool create_ddop(std::shared_ptr<isobus::DeviceDescriptorObjectPool> poolToPopulate,
                     isobus::NAME clientName) const {
        bool retVal = true;
        std::uint16_t elementCounter = 0;
        assert(0 != get_number_of_sections());
        const std::int32_t SECTION_WIDTH = (BOOM_WIDTH / get_number_of_sections());
        poolToPopulate->clear();

        constexpr std::array<std::uint8_t, 7> localizationData = {'e', 'n', 0x50, 0x00, 0x55, 0x55, 0xFF};

        retVal &= poolToPopulate->add_device("HASHTAG", "1.42.0", "WAZZZAAAAAA", "SP1.11", localizationData,
                                             std::vector<std::uint8_t>(), clientName.get_full_name());
        retVal &= poolToPopulate->add_device_element(
            "Sprayer", elementCounter, 0, isobus::task_controller_object::DeviceElementObject::Type::Device,
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::MainDeviceElement));
        retVal &= poolToPopulate->add_device_process_data(
            "Actual Work State", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState),
            isobus::NULL_OBJECT_ID,
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::DeviceActualWorkState));
        retVal &= poolToPopulate->add_device_process_data(
            "Hashtag", static_cast<std::uint16_t>(65432),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ShortWidthPresentation),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::HashtagParameter));
        retVal &= poolToPopulate->add_device_process_data(
            "Request Default PD", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::RequestDefaultProcessData),
            isobus::NULL_OBJECT_ID, 0,
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::Total),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::RequestDefaultProcessData));
        retVal &= poolToPopulate->add_device_process_data(
            "Total Time", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::EffectiveTotalTime),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::TimePresentation),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet) |
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::Total),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::DeviceTotalTime));
        elementCounter++; // Finished with main device element, increment to start adding child elements

        // Add child element Connector, and properties
        retVal &= poolToPopulate->add_device_element(
            "Connector", elementCounter, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::MainDeviceElement),
            isobus::task_controller_object::DeviceElementObject::Type::Connector,
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::Connector));
        retVal &= poolToPopulate->add_device_process_data(
            "Connector X", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ShortWidthPresentation),
            static_cast<std::uint8_t>(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable),
            0, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ConnectorXOffset));
        retVal &= poolToPopulate->add_device_process_data(
            "Connector Y", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetY),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ShortWidthPresentation),
            static_cast<std::uint8_t>(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable),
            0, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ConnectorYOffset));
        retVal &= poolToPopulate->add_device_property(
            "Type", 9, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ConnectorType), isobus::NULL_OBJECT_ID,
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ConnectorType));
        elementCounter++;


        // Add presentations
        retVal &= poolToPopulate->add_device_value_presentation(
            "mm", 0, 1.0f, 0, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ShortWidthPresentation));
        retVal &= poolToPopulate->add_device_value_presentation(
            "minutes", 0, 1.0f, 1, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::TimePresentation));

        if (retVal) {
            auto sprayer = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
                poolToPopulate->get_object_by_id(
                    static_cast<std::uint16_t>(ImplementDDOPObjectIDs::MainDeviceElement)));
            auto connector = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
                poolToPopulate->get_object_by_id(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::Connector)));
            auto boom = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
                poolToPopulate->get_object_by_id(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::SprayBoom)));
            auto product = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
                poolToPopulate->get_object_by_id(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::LiquidProduct)));

            sprayer->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::DeviceActualWorkState));
            sprayer->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::SetpointWorkState));
            sprayer->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::DeviceTotalTime));
            sprayer->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::RequestDefaultProcessData));
            sprayer->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::HashtagParameter));

            connector->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ConnectorXOffset));
            connector->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ConnectorYOffset));
            connector->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ConnectorType));

            boom->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::BoomXOffset));
            boom->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::BoomYOffset));
            boom->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::BoomZOffset));
            boom->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ActualWorkingWidth));
            boom->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::SectionControlState));
            boom->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::AreaTotal));
            boom->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ActualCondensedWorkingState1To16));
            boom->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::SetpointCondensedWorkingState1To16));

            product->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::TankCapacity));
            product->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::TankVolume));
            product->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::LifetimeApplicationVolumeTotal));
            product->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::PrescriptionControlState));
            product->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ActualCulturalPractice));
            product->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::TargetRate));
            product->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ActualRate));
        }
        return retVal;
    } // End of create_ddop

    static bool request_value_command_callback(std::uint16_t, std::uint16_t DDI, std::int32_t &value,
                                               void *parentPointer) {
        if (nullptr != parentPointer) {
            // std::cout << "DDI: " << DDI << "\n";
            auto sim = reinterpret_cast<SectionControlImplementSimulator *>(parentPointer);
            switch (DDI) {
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::MaximumVolumeContent):
                value = 4000000;
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualVolumeContent):
                value = 3000000;
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState):
                value = sim->get_section_control_state();
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::PrescriptionControlState):
                value = sim->get_prescription_control_state();
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16): {
                value = 0;
                for (std::uint_fast8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++) {
                    if (i < sim->get_number_of_sections()) {
                        bool sectionState = sim->get_section_actual_state(i);
                        value |= static_cast<std::uint8_t>(sectionState) << (2 * i);
                    } else {
                        value |= (static_cast<std::uint32_t>(0x03) << (2 * i));
                    }
                }
            } break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualVolumePerAreaApplicationRate):
                value = sim->get_actual_rate();
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState):
                value = sim->get_actual_number_of_sections_on() > 0 ? 1 : 0;
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetY):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::RequestDefaultProcessData):
                value = 0;
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkingWidth):
                value = BOOM_WIDTH;
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16): {
                value = 0;
                for (std::uint_fast8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++) {
                    if (i < sim->get_number_of_sections()) {
                        std::uint8_t sectionState = sim->get_section_setpoint_state(i);
                        value |= sectionState << (2 * i);
                    } else {
                        value |= (static_cast<std::uint32_t>(0x03) << (2 * i));
                    }
                }
            } break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointVolumePerAreaApplicationRate):
                value = sim->get_target_rate();
                break;
            case static_cast<std::uint16_t>(65432): // Custom DDI for testing
            {
                value = static_cast<std::uint32_t>(auth_status);
                std::cout << "Hashtag auth status: " << value << "\n";
                break;
            }
            default:
                value = 0;
                break;
            }
        }
        return true;
    }

    static bool value_command_callback(std::uint16_t, std::uint16_t DDI, std::int32_t processVariableValue,
                                       void *parentPointer) {
        if (nullptr != parentPointer) {
            auto sim = reinterpret_cast<SectionControlImplementSimulator *>(parentPointer);
            switch (DDI) {
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16): {
                for (std::uint_fast8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++) {
                    if (i < sim->get_number_of_sections()) {
                        bool sectionState = (0x01 == (processVariableValue >> (2 * i) & 0x03));
                        sim->sectionSetpointStates.at(i) = sectionState;
                    } else {
                        break;
                    }
                }
            } break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointVolumePerAreaApplicationRate):
                sim->targetRate = processVariableValue;
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState):
                sim->setpointWorkState = (0x01 == processVariableValue);
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::PrescriptionControlState):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState):
                sim->set_is_mode_auto(processVariableValue != 0);
                break;
            default:
                break;
            }
        }
        return true;
    }

    std::vector<bool> sectionSetpointStates;
    std::vector<bool> sectionSwitchStates;
    std::uint32_t targetRate = 100000;
    bool setpointWorkState = true;
    bool isAutoMode = true;
};

int main(int argc, char **argv) {
    std::signal(SIGINT, signal_handler);

    int counter = 0;

    std::cout << "Sprayer TC Client Example (Clean Version)\n";

    auto canDriver = std::make_shared<isobus::SocketCANInterface>("vcan0");
    if (nullptr == canDriver) {
        std::cout << "Unable to find a CAN driver." << std::endl;
        return -1;
    }

    isobus::CANHardwareInterface::set_number_of_can_channels(1);
    isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);

    if ((!isobus::CANHardwareInterface::start()) || (!canDriver->get_is_valid())) {
        std::cout << "Failed to start hardware interface." << std::endl;
        return -2;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    isobus::NAME TestDeviceNAME(0);
    TestDeviceNAME.set_arbitrary_address_capable(true);
    TestDeviceNAME.set_industry_group(2);
    TestDeviceNAME.set_device_class(6);
    TestDeviceNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::RateControl));
    TestDeviceNAME.set_identity_number(2);
    TestDeviceNAME.set_ecu_instance(0);
    TestDeviceNAME.set_function_instance(0);
    TestDeviceNAME.set_device_class_instance(0);
    TestDeviceNAME.set_manufacturer_code(1407);

    const isobus::NAMEFilter filterTaskController(isobus::NAME::NAMEParameters::FunctionCode,
                                                  static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));
    const isobus::NAMEFilter filterTaskControllerInstance(isobus::NAME::NAMEParameters::FunctionInstance, 0);
    const isobus::NAMEFilter filterTaskControllerIndustryGroup(
        isobus::NAME::NAMEParameters::IndustryGroup,
        static_cast<std::uint8_t>(isobus::NAME::IndustryGroup::AgriculturalAndForestryEquipment));
    const isobus::NAMEFilter filterTaskControllerDeviceClass(
        isobus::NAME::NAMEParameters::DeviceClass, static_cast<std::uint8_t>(isobus::NAME::DeviceClass::NonSpecific));
    const std::vector<isobus::NAMEFilter> tcNameFilters = {filterTaskController, filterTaskControllerInstance,
                                                           filterTaskControllerIndustryGroup,
                                                           filterTaskControllerDeviceClass};

    auto TestInternalECU = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(TestDeviceNAME, 0);
    auto TestPartnerTC = isobus::CANNetworkManager::CANNetwork.create_partnered_control_function(0, tcNameFilters);

    auto TestTCClient = std::make_shared<isobus::TaskControllerClient>(TestPartnerTC, TestInternalECU, nullptr);

    auto myDDOP = std::make_shared<isobus::DeviceDescriptorObjectPool>();
    bool tcClientStarted = false;

    constexpr std::uint8_t NUMBER_OF_SECTIONS = 6;
    SectionControlImplementSimulator rateController(NUMBER_OF_SECTIONS);

    std::cout << "Sections: " << static_cast<int>(NUMBER_OF_SECTIONS) << "\n";
    std::cout << "Waiting for TC server...\n\n";

    while (running) {
        counter++;
        if (!tcClientStarted) {
            if (rateController.create_ddop(myDDOP, TestInternalECU->get_NAME())) {
                TestTCClient->configure(myDDOP, 1, NUMBER_OF_SECTIONS, 1, true, false, true, false, true);
                TestTCClient->add_request_value_callback(
                    SectionControlImplementSimulator::request_value_command_callback, &rateController);
                TestTCClient->add_value_command_callback(SectionControlImplementSimulator::value_command_callback,
                                                         &rateController);
                TestTCClient->initialize(true);
                tcClientStarted = true;
                std::cout << "TC Client initialized successfully\n";
            } else {
                std::cout << "Failed to create DDOP\n";
                break;
            }
        }

        if (counter % 10 == 0) {
            if (auth_status != 0) {
                auth_status = 0;
            } else {
                auth_status = 1;
            }
        }
        TestTCClient->on_value_changed_trigger(0, 65432); // Trigger for on-change update at DeviceElement A, for DDI B (65432)

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << "\nShutting down...\n";
    TestTCClient->terminate();
    isobus::CANHardwareInterface::stop();
    return (!tcClientStarted);
}
