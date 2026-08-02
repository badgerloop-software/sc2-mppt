#include "can_mppt.h"

#include "can_ids.h"
#include "mppt.h"

// ------------- PUBLIC FUNCTIONS -------------

CanMppt::CanMppt(CAN_TypeDef* canPort, CAN_PINS pins, int frequency)
    : CANManager(canPort, pins, frequency) {}

void CanMppt::readHandler(CAN_message_t msg) {
    uint8_t* data = msg.buf;
    switch (msg.id) {
        case SC2_CAN_MPPT_CAP_DISCHARGE_ID:
            setCapDischarge(*data);
            break;
        case SC2_CAN_MPPT_OV_RESET_ID:
            clearOVFaultReset(*data);
            break;
        case SC2_CAN_BMS_PACK_ID:
            // bytes 0-1: current * 0.1 A, byte 4: SOC * 0.5 %
            packCurrent = ((data[0] << 8) + data[1]) * 0.1f;
            packSOC = (float)(data[4]) / 2.0f;
            break;
        case SC2_CAN_BMS_CHARGE_LIMIT_ID:
            packChargeCurrentLimit = (float)(*(uint16_t*)data) * CONST_CURR_SAFETY_MULT;
            break;
        default:
            break;
    }
}

void CanMppt::sendMpptData() {
    sendMessage(SC2_CAN_MPPT_BOOST_ID, (void*)&boostEnabled, sizeof(boostEnabled));
    sendMessage(SC2_CAN_MPPT_MODE_ID, (void*)&chargeMode, sizeof(ChargeMode));

    for (int i = 0; i < NUM_ARRAYS; i++) {
        sendMessage(SC2_CAN_MPPT_STRING_ID(i, SC2_CAN_MPPT_FIELD_V), (void*)&(arrayData[i].voltage),
                    sizeof(float));
        sendMessage(SC2_CAN_MPPT_STRING_ID(i, SC2_CAN_MPPT_FIELD_I), (void*)&(arrayData[i].current),
                    sizeof(float));
        sendMessage(SC2_CAN_MPPT_STRING_ID(i, SC2_CAN_MPPT_FIELD_TEMP), (void*)&(arrayData[i].temp),
                    sizeof(float));
        sendMessage(SC2_CAN_MPPT_STRING_ID(i, SC2_CAN_MPPT_FIELD_DUTY),
                    (void*)&(arrayData[i].dutyCycle), sizeof(float));
        if (chargeMode == ChargeMode::MPPT) {
            sendMessage(SC2_CAN_MPPT_STRING_ID(i, SC2_CAN_MPPT_FIELD_TARGET),
                        (void*)&(targetVoltage[i]), sizeof(float));
        } else {
            // no target in const current mode
            float none = -1.0f;
            sendMessage(SC2_CAN_MPPT_STRING_ID(i, SC2_CAN_MPPT_FIELD_TARGET), (void*)&none,
                        sizeof(float));
        }
    }
}
