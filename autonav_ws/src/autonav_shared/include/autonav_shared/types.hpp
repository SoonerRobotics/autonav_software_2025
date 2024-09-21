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

    namespace Logging
    {
        enum LogLevel
        {
            DEBUG = 0,
            INFO = 1,
            WARN = 2,
            ERROR = 3,
            FATAL = 4
        };

        const std::map<LogLevel, std::string> LOG_LEVEL_NAMES = {
            {LogLevel::DEBUG, "DEBUG"},
            {LogLevel::INFO, "INFO"},
            {LogLevel::WARN, "WARN"},
            {LogLevel::ERROR, "ERROR"},
            {LogLevel::FATAL, "FATAL"}};
    }
}