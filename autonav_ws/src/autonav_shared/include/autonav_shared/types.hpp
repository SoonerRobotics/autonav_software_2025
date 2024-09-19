namespace AutoNav
{
    enum SystemState
    {
        DISABLED = 0,
        MANUAL = 1,
        AUTONOMOUS = 2,
        SHUTDOWN = 3
    };

    enum DeviceState
    {
        OFF = 0,
        WARMING = 1,
        READY = 2,
        OPERATING = 3,
        ERROR = 4
    };
}