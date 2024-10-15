#include "canmanager.h"

CANManager::CANManager(CAN_TypeDef* canPort, CAN_PINS pins, int frequency) : canBus(canPort, pins) {                                                            
    // Begin canBus and set its frequency
    this->canBus.begin();
    this->canBus.setBaudRate(frequency);
}

bool CANManager::sendMessage(int messageID, void* data, int length, int timeout) {
    bool retValue = false; // return value of write function. False by default

    // Create a CAN message that is going to be written
    CAN_message_t CAN_message;                
    CAN_message.id = messageID;
    CAN_message.len = length;

    // Copy the data to the buffer
    for (int i = 0; i < length; i++) {
        CAN_message.buf[i] = ((uint8_t*)data)[i];
    }

    // Storing the time right before we try sending
    unsigned long start = millis();

    // Keep trying to write the message until it is successful or timeout
    while (!(retValue = this->canBus.write(CAN_message)) && millis() - start < timeout){
        this->runQueue(1);
    }

    return retValue;  //returns true if message is written successfully
}

void CANManager::runQueue(int duration) {
    CAN_message_t msg; 

    // Storing the time when the function is called
    unsigned long start = millis();

    // reads by dequeuing messages from Rx Ring buffer until the duration is reached
    while (millis() - start < duration){
        if (this->canBus.read(msg)) {
            this->readHandler(msg); //calls readHandler function to process the message              
        }
    }   
}