#pragma once
// A very small test framework plus the helpers every scenario uses.
//
//   TEST(group, name) { sim::Sim s; boot(s); ... CHECK_EQ(s.state(), "Idle"); }
//
// KNOWN_ISSUE() marks a test that asserts the behaviour we *want* and is
// currently expected to fail. Those are reported but do not fail the run; if
// one starts passing, the runner says so, so the marker can be dropped.
#include "sim.h"

#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

namespace check {

struct Test {
  const char *group, *name;
  void (*fn)();
  const char *known_issue;  // why it is expected to fail, or nullptr
};
std::vector<Test> &registry();
struct Register {
  Register(const char *g, const char *n, void (*fn)(), const char *issue = nullptr) {
    registry().push_back({g, n, fn, issue});
  }
};

void fail(const char *file, int line, const std::string &what);

inline std::string show(const std::string &v) { return "\"" + v + "\""; }
inline std::string show(const char *v) { return show(std::string(v)); }
inline std::string show(bool v) { return v ? "true" : "false"; }
template <class T> std::string show(const T &v) {
  std::ostringstream o;
  o << +v;
  return o.str();
}

}  // namespace check

#define CHECK_CAT2(a, b) a##b
#define CHECK_CAT(a, b) CHECK_CAT2(a, b)
#define TEST(group, name)                                                         \
  static void group##_##name();                                                   \
  static check::Register CHECK_CAT(reg_, __LINE__)(#group, #name, group##_##name); \
  static void group##_##name()
#define KNOWN_ISSUE(group, name, why)                                                  \
  static void group##_##name();                                                        \
  static check::Register CHECK_CAT(reg_, __LINE__)(#group, #name, group##_##name, why); \
  static void group##_##name()

#define CHECK(cond) \
  do { if (!(cond)) check::fail(__FILE__, __LINE__, #cond); } while (0)
#define CHECK_EQ(a, b)                                                            \
  do {                                                                            \
    auto _a = (a); auto _b = (b);                                                 \
    if (!(_a == _b))                                                              \
      check::fail(__FILE__, __LINE__, std::string(#a " == " #b ": got ") +        \
                  check::show(_a) + ", want " + check::show(_b));                 \
  } while (0)
#define CHECK_NEAR(a, b, tol)                                                     \
  do {                                                                            \
    double _a = (a), _b = (b);                                                    \
    if (std::fabs(_a - _b) > (tol))                                               \
      check::fail(__FILE__, __LINE__, std::string(#a " ~= " #b ": got ") +        \
                  check::show(_a) + ", want " + check::show(_b) + " +- " #tol);   \
  } while (0)
#define CHECK_HAS(haystack, needle)                                               \
  do {                                                                            \
    std::string _h = (haystack), _n = (needle);                                   \
    if (_h.find(_n) == std::string::npos)                                         \
      check::fail(__FILE__, __LINE__, std::string(#haystack " contains ") +       \
                  check::show(_n) + ": got " + check::show(_h));                  \
  } while (0)

// ------------------------------------------------------------------ helpers --
namespace garage {

// Power on and let the buttons' 50 ms debounce and the first LCD frame settle.
inline void boot(sim::Sim &s) {
  s.power_on();
  s.run_ms(300);
}

// A car parked at the stop, with the door shut. Call before boot().
inline void car_parked(sim::Sim &s) { s.car_parked(); }

// A car standing where the sonar reads about `reading_cm`.
inline void car_at_reading(sim::Sim &s, double reading_cm) {
  s.car.present = true;
  s.car.pos_cm = s.car.target_cm = s.pos_for_reading(reading_cm);
}

// Drive in and stop just past the steady tone, as a driver following the
// park assist would.
inline void drive_in(sim::Sim &s, double speed_cm_s = 40) {
  s.drive_to(s.pos_for_reading(PARK_NEAR_DISTANCE_DEFAULT) + 5, speed_cm_s);
}

inline bool wait_state(sim::Sim &s, const std::string &state, double timeout_ms) {
  return s.run_until([&] { return s.state() == state; }, timeout_ms);
}

// Every line the sketch printed since index `from`, joined.
inline std::string log_since(const sim::Sim &s, size_t from = 0) {
  std::string out;
  for (size_t i = from; i < s.lines.size(); i++) out += s.lines[i].text + "\n";
  return out;
}

inline int count_lines(const sim::Sim &s, const std::string &needle, size_t from = 0) {
  int n = 0;
  for (size_t i = from; i < s.lines.size(); i++)
    if (s.lines[i].text.find(needle) != std::string::npos) n++;
  return n;
}

// Rising edges on a pin in [from_us, to_us).
inline std::vector<uint64_t> rises(const sim::Sim &s, int pin, uint64_t from_us = 0,
                                   uint64_t to_us = UINT64_MAX) {
  std::vector<uint64_t> out;
  for (auto &e : s.edges)
    if (e.pin == pin && e.level && e.at_us >= from_us && e.at_us < to_us) out.push_back(e.at_us);
  return out;
}

// How long a pin was HIGH in total within [from_us, to_us).
inline double high_ms(const sim::Sim &s, int pin, uint64_t from_us, uint64_t to_us) {
  uint8_t level = 0;
  uint64_t since = from_us, total = 0;
  for (auto &e : s.edges) {
    if (e.pin != pin) continue;
    if (e.at_us < from_us) { level = e.level; continue; }
    if (e.at_us >= to_us) break;
    if (level) total += e.at_us - since;
    level = e.level;
    since = e.at_us;
  }
  if (level) total += to_us - std::max(since, from_us);
  return total / 1000.0;
}

}  // namespace garage
