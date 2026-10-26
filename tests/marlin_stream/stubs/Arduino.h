#pragma once
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <type_traits>
using uint = unsigned;
#include <chrono>
extern uint32_t testMillis;
inline uint32_t millis() {
#ifdef TEST_MARLIN_WORKER
  static const auto start = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
#else
  return testMillis;
#endif
}
// Only Arduino String's value semantics; hardware, timing and IO are explicit
// fakes in host_environment.h. Production sources are compiled unchanged.
class String {
  std::string value;

 public:
  String() = default;
  String(const char* s) : value(s ? s : "") {}
  String(const std::string& s) : value(s) {}
  template <typename T,
            typename std::enable_if<std::is_integral<T>::value, int>::type = 0>
  String(T n) : value(std::to_string(n)) {}
  const char* c_str() const { return value.c_str(); }
  size_t length() const { return value.size(); }
  void reserve(size_t n) { value.reserve(n); }
  char& operator[](size_t i) { return value[i]; }
  char operator[](size_t i) const { return value[i]; }
  String& operator+=(const String& s) {
    value += s.value;
    return *this;
  }
  String& operator+=(char c) {
    value += c;
    return *this;
  }
  friend String operator+(const String& a, const String& b) {
    return a.value + b.value;
  }
  bool operator==(const String& b) const { return value == b.value; }
  int indexOf(const String& c, size_t from = 0) const {
    return indexOf(c.c_str(), from);
  }
  int indexOf(char c, size_t from = 0) const {
    auto p = value.find(c, from);
    return p == std::string::npos ? -1 : int(p);
  }
  int indexOf(const char* c, size_t from = 0) const {
    auto p = value.find(c, from);
    return p == std::string::npos ? -1 : int(p);
  }
  String substring(size_t from, size_t to = std::string::npos) const {
    if (from >= value.size()) return "";
    return value.substr(from, std::min(to, value.size()) - from);
  }
  bool startsWith(const char* p) const {
    return value.compare(0, strlen(p), p) == 0;
  }
  bool endsWith(const char* p) const {
    return value.size() >= strlen(p) &&
           value.compare(value.size() - strlen(p), strlen(p), p) == 0;
  }
  void trim() {
    auto begin = value.find_first_not_of(" \r\n\t");
    value = begin == std::string::npos
                ? ""
                : value.substr(begin,
                               value.find_last_not_of(" \r\n\t") - begin + 1);
  }
  void toLowerCase() {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return std::tolower(c); });
  }
  void replace(const char* from, const char* to) {
    for (size_t pos = 0; (pos = value.find(from, pos)) != std::string::npos;
         pos += strlen(to))
      value.replace(pos, strlen(from), to);
  }
  long toInt() const { return std::strtol(value.c_str(), nullptr, 10); }
};
