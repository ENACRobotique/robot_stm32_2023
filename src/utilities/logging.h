#ifndef LOGGING_HEADER
#define LOGGING_HEADER

#include <stdarg.h>

#define LOGGING_BUFFER 200

/// @brief Static logging class.
/// 
/// Output formatted data to usb serial port
/// with timestamp and severity.
/// Interface similar to printf.
class Logging {
public:
    static void init(unsigned long speed);

    static void trace(const char* message, ...);
    static void debug(const char* message, ...);
    static void info(const char* message, ...);
    static void warning(const char* message, ...);
    static void error(const char* message, ...);
    static void critical(const char* message, ...);

private:
    static void _log(const char *severity, const char *message, va_list args);

    static char _buffer[LOGGING_BUFFER];
};

#endif // LOGGING_HEADER