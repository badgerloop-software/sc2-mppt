#ifndef __CAN_MANAGER_H__
#define __CAN_MANAGER_H__

#include "STM32_CAN.h"

#define DEFAULT_CAN_FREQ 250000     // Match Elcon charger

class CANManager {
    private:
        STM32_CAN canBus;                  // object to interface with CAN

    public:
        /* Constructor initializing bus and all manager functions
         *
         * canPort: choose from CAN1, CAN2, CAN3 (NOTE: see STM32_CAN library for more details)
         * pins: choose from DEF, ALT1, ALT2
         * frequency: Baud rate of can bus
         */
        CANManager(CAN_TypeDef* canPort, CAN_PINS pins, int frequency = DEFAULT_CAN_FREQ);

        /* Reads input message and does any logic handling needed
         * Intended to be implemented by class extension per board
         */
        virtual void readHandler(CAN_message_t msg) = 0;

        /* Send a message over CAN
         *
         * messageID: CAN ID to use to identify the signal
         * data: Payload array
         * length: Size of data in bytes
         * timeout: in milliseconds
         */ 
        bool sendMessage(int messageID, void* data, int length, int timeout = 10);

        /* Processes CAN (read) messages stored in messageQueue for a set duration. 
         * THIS IS THE FUNCTION TO CALL FOR PROCESSING CAN READ MESSAGES
         * 
         * duration: time in milliseconds
         */
        void runQueue(int duration);
};


#endif