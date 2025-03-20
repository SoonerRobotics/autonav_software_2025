#include <CONBus.h>
#include <CANBusDriver.h>
#include <catch.hpp>

TEST_CASE("Initialize CANBusDriver test") {
    CONBus::CONBus conbus;

    CONBus::CANBusDriver conbus_can(conbus, 42);
}

TEST_CASE("CANBusDriver Read test") {
    CONBus::CONBus conbus;

    CONBus::CANBusDriver conbus_can(conbus, 42);

    uint8_t example_register = 24;
    conbus.addRegister(14, &example_register);

    // Mock can data asking for register 14 on device 42
    uint32_t can_id = 1042;
    uint8_t can_data[8];
    can_data[0] = 14;

    conbus_can.readCanMessage(can_id, can_data);

    REQUIRE(conbus_can.isReplyReady() == true);

    uint32_t out_can_id = 0;
    uint8_t out_can_len = 0;
    uint8_t out_can_data[8];

    conbus_can.peekReply(out_can_id, out_can_len, out_can_data);

    REQUIRE(conbus_can.isReplyReady() == true);
    REQUIRE(out_can_id == 1142);
    REQUIRE(out_can_len == 4);
    REQUIRE(out_can_data[0] == 14);
    REQUIRE(out_can_data[1] == 1);
    REQUIRE(out_can_data[3] == 24);

    conbus_can.popReply();
    REQUIRE(conbus_can.isReplyReady() == false);
}

TEST_CASE("CANBusDriver Read All test") {
    CONBus::CONBus conbus;

    CONBus::CANBusDriver conbus_can(conbus, 42);

    uint8_t example_register = 24;
    uint8_t example_register_2 = 25;
    conbus.addRegister(14, &example_register);
    conbus.addRegister(20, &example_register_2);

    // Mock can data asking for all register on device 42
    uint32_t can_id = 1042;
    uint8_t can_data[8];
    can_data[0] = 0xFF;

    conbus_can.readCanMessage(can_id, can_data);

    REQUIRE(conbus_can.isReplyReady() == true);

    uint32_t out_can_id = 0;
    uint8_t out_can_len = 0;
    uint8_t out_can_data[8];

    conbus_can.peekReply(out_can_id, out_can_len, out_can_data);

    REQUIRE(conbus_can.isReplyReady() == true);
    REQUIRE(out_can_id == 1142);
    REQUIRE(out_can_len == 4);
    REQUIRE(out_can_data[0] == 14);
    REQUIRE(out_can_data[1] == 1);
    REQUIRE(out_can_data[3] == 24);

    conbus_can.popReply();

    REQUIRE(conbus_can.isReplyReady() == true);

    conbus_can.peekReply(out_can_id, out_can_len, out_can_data);

    REQUIRE(conbus_can.isReplyReady() == true);
    REQUIRE(out_can_id == 1142);
    REQUIRE(out_can_len == 4);
    REQUIRE(out_can_data[0] == 20);
    REQUIRE(out_can_data[1] == 1);
    REQUIRE(out_can_data[3] == 25);

    conbus_can.popReply();

    REQUIRE(conbus_can.isReplyReady() == false);
}

TEST_CASE("CANBusDriver Write test") {
    CONBus::CONBus conbus;

    CONBus::CANBusDriver conbus_can(conbus, 42);

    uint8_t example_register = 24;
    conbus.addRegister(14, &example_register);

    // Mock can data writing register 14 on device 42
    uint32_t can_id = 1242;
    uint8_t can_data[8];
    can_data[0] = 14;
    can_data[1] = 1;
    can_data[3] = 33;

    conbus_can.readCanMessage(can_id, can_data);

    REQUIRE(conbus_can.isReplyReady() == true);

    uint32_t out_can_id = 0;
    uint8_t out_can_len = 0;
    uint8_t out_can_data[8];

    conbus_can.peekReply(out_can_id, out_can_len, out_can_data);

    REQUIRE(conbus_can.isReplyReady() == true);
    REQUIRE(out_can_id == 1342);
    REQUIRE(out_can_len == 4);
    REQUIRE(out_can_data[0] == 14);
    REQUIRE(out_can_data[1] == 1);
    REQUIRE(out_can_data[3] == 33);
    REQUIRE(example_register == 33);

    conbus_can.popReply();

    REQUIRE(conbus_can.isReplyReady() == false);
}

TEST_CASE("CANBusDriver Read All test with failure") {
    CONBus::CONBus conbus;

    CONBus::CANBusDriver conbus_can(conbus, 42);

    uint8_t example_register = 24;
    uint8_t example_register_2 = 25;
    conbus.addRegister(14, &example_register);
    conbus.addRegister(20, &example_register_2);

    // Mock can data asking for all register on device 42
    uint32_t can_id = 1042;
    uint8_t can_data[8];
    can_data[0] = 0xFF;

    conbus_can.readCanMessage(can_id, can_data);

    REQUIRE(conbus_can.isReplyReady() == true);

    uint32_t out_can_id = 0;
    uint8_t out_can_len = 0;
    uint8_t out_can_data[8];

    conbus_can.peekReply(out_can_id, out_can_len, out_can_data);

    REQUIRE(conbus_can.isReplyReady() == true);
    REQUIRE(out_can_id == 1142);
    REQUIRE(out_can_len == 4);
    REQUIRE(out_can_data[0] == 14);
    REQUIRE(out_can_data[1] == 1);
    REQUIRE(out_can_data[3] == 24);

    // Simulate failure by not popping
    // conbus_can.popReply();

    conbus_can.peekReply(out_can_id, out_can_len, out_can_data);

    REQUIRE(conbus_can.isReplyReady() == true);
    REQUIRE(out_can_id == 1142);
    REQUIRE(out_can_len == 4);
    REQUIRE(out_can_data[0] == 14);
    REQUIRE(out_can_data[1] == 1);
    REQUIRE(out_can_data[3] == 24);

    conbus_can.popReply();

    REQUIRE(conbus_can.isReplyReady() == true);

    conbus_can.peekReply(out_can_id, out_can_len, out_can_data);

    REQUIRE(conbus_can.isReplyReady() == true);
    REQUIRE(out_can_id == 1142);
    REQUIRE(out_can_len == 4);
    REQUIRE(out_can_data[0] == 20);
    REQUIRE(out_can_data[1] == 1);
    REQUIRE(out_can_data[3] == 25);

    conbus_can.popReply();

    REQUIRE(conbus_can.isReplyReady() == false);
}

TEST_CASE("CANBusDriver Write test with failure") {
    CONBus::CONBus conbus;

    CONBus::CANBusDriver conbus_can(conbus, 42);

    uint8_t example_register = 24;
    conbus.addRegister(14, &example_register);

    // Mock can data writing register 14 on device 42
    uint32_t can_id = 1242;
    uint8_t can_data[8];
    can_data[0] = 14;
    can_data[1] = 1;
    can_data[3] = 33;

    conbus_can.readCanMessage(can_id, can_data);

    REQUIRE(conbus_can.isReplyReady() == true);

    uint32_t out_can_id = 0;
    uint8_t out_can_len = 0;
    uint8_t out_can_data[8];

    conbus_can.peekReply(out_can_id, out_can_len, out_can_data);

    REQUIRE(conbus_can.isReplyReady() == true);
    REQUIRE(out_can_id == 1342);
    REQUIRE(out_can_len == 4);
    REQUIRE(out_can_data[0] == 14);
    REQUIRE(out_can_data[1] == 1);
    REQUIRE(out_can_data[3] == 33);
    REQUIRE(example_register == 33);

    // Simulate failure by not popping
    // conbus_can.popReply();
    
    conbus_can.peekReply(out_can_id, out_can_len, out_can_data);

    REQUIRE(conbus_can.isReplyReady() == true);
    REQUIRE(out_can_id == 1342);
    REQUIRE(out_can_len == 4);
    REQUIRE(out_can_data[0] == 14);
    REQUIRE(out_can_data[1] == 1);
    REQUIRE(out_can_data[3] == 33);
    REQUIRE(example_register == 33);

    conbus_can.popReply();

    REQUIRE(conbus_can.isReplyReady() == false);
}