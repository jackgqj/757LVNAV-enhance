#pragma once

#include <string>

namespace lvnv {

// Implemented in plugin.cpp.  A separate header lets the route/CDU readers log
// into the same file without duplicating the logging implementation.
void LogLine(const std::string& line);
void Logf(const char* fmt, ...);

} // namespace lvnv
