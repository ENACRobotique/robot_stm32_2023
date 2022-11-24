#include "logging.h"
#include <Arduino.h>
#include <stdio.h>


char Logging::_buffer[LOGGING_BUFFER] = {0};

void Logging::init(unsigned long speed) {
    Serial.begin(speed);
}

void Logging::trace(const char* message, ...) {
    va_list argptr;

	va_start(argptr, message);
	_log("trace", message, argptr);
	va_end(argptr);
}

void Logging::debug(const char* message, ...) {
    va_list argptr;

	va_start(argptr, message);
	_log("debug", message, argptr);
	va_end(argptr);
}

void Logging::info(const char* message, ...) {
    va_list argptr;

	va_start(argptr, message);
	_log("info", message, argptr);
	va_end(argptr);
}

void Logging::warning(const char* message, ...) {
    va_list argptr;

	va_start(argptr, message);
	_log("warning", message, argptr);
	va_end(argptr);
}

void Logging::error(const char* message, ...) {
    va_list argptr;

	va_start(argptr, message);
	_log("error", message, argptr);
	va_end(argptr);
}

void Logging::critical(const char* message, ...) {
    va_list argptr;

	va_start(argptr, message);
	_log("critical", message, argptr);
	va_end(argptr);
}

void Logging::_log(const char *severity, const char *message, va_list args) {
    uint32_t time = millis();

    uint32_t hours = time / 3600000;
    uint32_t minutes = (time / 60000) % 60;
    uint32_t seconds = (time / 1000) % 60;

    int length = snprintf(_buffer, LOGGING_BUFFER, "%02d:%02d:%02d [%s] ", hours, minutes, seconds, severity);
    vsnprintf(_buffer + length, LOGGING_BUFFER - length, message, args);

    Serial.println(_buffer);
}