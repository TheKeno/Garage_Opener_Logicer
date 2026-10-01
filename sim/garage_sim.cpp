// garage_sim - drive the R6 sketch interactively in a terminal.
//
// The real sketch runs against the simulated garage in sim.cpp, in real time
// (or faster). You play the car, the driver and the ESP8266; the screen shows
// what the board shows - the LCD, the park-assist LEDs and buzzer, the relay,
// the status pins - plus the door, the car and the light the sketch sees.
#include "sim.h"

#include "LightPulseSensor.h"

#include <poll.h>
#include <signal.h>
#include <termios.h>
#include <unistd.h>

#include <cctype>
#include <chrono>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

static const char *USAGE = R"(usage: garage_sim [options]

  --speed X        start at X times real time (default 1)
  --parked         start with the car parked at the stop
  --daylight N     light through the open door, in ADC counts (default 0: night)
  --bounce MS      contact bounce on the buttons and microswitch (default 0)
  --door-s S       door travel time in seconds (default 14)
  --garage-cm CM   door to back wall (default 550)
  --sonar-at CM    where the sonar hangs, measured in from the door (default 320)
  --ceiling CM     the sonar's height above the floor (default 165)

Keys are listed at the bottom of the screen.
)";

static termios saved_tty;
static bool    tty_raw = false;

static void restore_tty() {
  if (!tty_raw) return;
  tcsetattr(STDIN_FILENO, TCSANOW, &saved_tty);
  printf("\x1b[?25h\x1b[0m\n");  // cursor back on
  fflush(stdout);
  tty_raw = false;
}

static void on_signal(int) {
  restore_tty();
  _exit(130);
}

static void raw_tty() {
  tcgetattr(STDIN_FILENO, &saved_tty);
  termios t = saved_tty;
  t.c_lflag &= ~(ICANON | ECHO);
  t.c_cc[VMIN] = 0;
  t.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &t);
  tty_raw = true;
  atexit(restore_tty);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  printf("\x1b[?25l\x1b[2J");  // hide cursor, clear
}

// ------------------------------------------------------------------ drawing --

static std::string frame;

static void out(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void out(const char *fmt, ...) {
  char buf[1024];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  frame += buf;
}
static void eol() { frame += "\x1b[K\n"; }

static const char *lamp(bool on, const char *colour) {
  static char buf[4][32];
  static int i = 0;
  i = (i + 1) % 4;
  snprintf(buf[i], sizeof buf[i], on ? "\x1b[%sm●\x1b[0m" : "\x1b[2m○\x1b[0m", colour);
  return buf[i];
}

static const char *state_colour(const std::string &st) {
  if (st == "Parking") return "33";
  if (st == "Open door" || st == "Close door") return "36";
  if (st == "Wait second") return "35";
  if (st == "Config") return "34";
  return "32";
}

struct Ui {
  // 'w' drives in at a normal pace and slows to a crawl a little before the
  // park assist starts beeping, so the last stretch is still yours to judge.
  double slow_from = -1;
  double speed = 1;
  bool   paused = false;
  bool   typing = false;
  std::string input;
  std::string note;
};

static void draw(sim::Sim &s, Ui &ui) {
  frame = "\x1b[H";
  const int W = 56;  // width of the door/car lane

  out("\x1b[1m Garage_Opener_Logicer_R6 simulator\x1b[0m    t=%8.1f s   speed %gx  %s",
      s.secs(), ui.speed, ui.paused ? "\x1b[7m PAUSED \x1b[0m" : (s.powered() ? "" : "\x1b[7m OFF \x1b[0m"));
  eol();
  eol();

  // LCD
  std::string st = s.state();
  out("  \x1b[44;97m┌────────────────┐\x1b[0m"); eol();
  out("  \x1b[44;97m│%s│\x1b[0m    state  \x1b[1;%sm%s\x1b[0m", s.lcd(0).c_str(), state_colour(st), st.c_str()); eol();
  out("  \x1b[44;97m│%s│\x1b[0m", s.lcd(1).c_str());
  out("    relay %s   status pins: door %s car %s",
      lamp(s.pin(doorPin), "93"), lamp(s.pin(doorStatus), "92"), lamp(s.pin(carStatus), "92"));
  eol();
  out("  \x1b[44;97m└────────────────┘\x1b[0m");
  out("    park   red %s  green %s  buzzer %s",
      lamp(s.pin(ledRedPin), "91"), lamp(s.pin(ledGreenPin), "92"),
      s.pin(buzzerPin) ? "\x1b[1;93m♪ BEEP\x1b[0m" : "\x1b[2m♪\x1b[0m");
  eol();
  eol();

  // Door
  int filled = (int)std::lround(s.door.pos * 20);
  const char *motion = s.door.motion > 0 ? "opening" : s.door.motion < 0 ? "closing" : s.door.shut() ? "shut" : s.door.open() ? "open" : "stopped";
  out("  Door   [");
  for (int i = 0; i < 20; i++) out(i < filled ? "░" : "█");
  out("] %3d%% %-8s microswitch %s", (int)std::lround(s.door.pos * 100), motion,
      s.door.shut() ? "closed" : "open");
  eol();

  // Side view: back wall on the left, the garage, the door, the driveway.
  // The sonar hangs from the ceiling and looks straight down; the car drives
  // in leftwards.
  double x0 = -s.car.length_cm() - 250, span = s.garage.depth_cm - x0;
  auto col = [&](double x) { return (int)std::lround((s.garage.depth_cm - x) / span * (W - 1)); };
  int door_col = col(0), wall_col = col(s.garage.depth_cm), sensor_col = col(s.garage.sensor_x_cm);
  std::string ceiling, lane;
  for (int i = 0; i < W; i++) {
    ceiling += i == sensor_col ? "\x1b[1;95m\u25bc\x1b[0m" : (i >= wall_col && i <= door_col) ? "\u2500" : " ";
    const char *c = i > door_col ? "\u00b7" : " ";
    if (i == door_col) c = s.door.pos < 0.95 ? "\u2588" : "\u2506";
    if (i == wall_col) c = "\u2590";
    if (s.car.present) {
      int front = col(s.car.pos_cm), back = col(s.car.pos_cm - s.car.length_cm());
      if (i >= front && i <= back && i != door_col)
        c = (i == front && s.car.headlights) ? "\x1b[93m\u25c0\x1b[0m" : "\x1b[96m\u2593\x1b[0m";
    }
    lane += c;
  }
  out("  Garage %s", ceiling.c_str());
  eol();
  out("         %s", lane.c_str());
  eol();
  int echo = s.echo_cm();
  std::string sonar = echo < 0 ? "--" : std::to_string(echo) + " cm";
  if (s.car.present)
    out("         bumper %+5.0f cm from the door   sonar %s   headlights %s   %s", s.car.pos_cm,
        sonar.c_str(), s.car.headlights ? "\x1b[93mON\x1b[0m" : "off",
        s.car.pos_cm != s.car.target_cm ? (s.car.target_cm > s.car.pos_cm ? "driving in" : "reversing") : "stopped");
  else
    out("         no car   sonar %s", sonar.c_str());
  eol();

  // Light
  int light = s.light_now();
  long avg = s.powered() ? (long)s.global<LightPulseSensor>("lightPulseSensor")->average : 0;
  int bar = light * 30 / 1023;
  out("  Light  A0 %4d  ", light);
  for (int i = 0; i < 30; i++) out(i < bar ? "▇" : "\x1b[2m▁\x1b[0m");
  out("  sketch's ambient avg %ld   %s", avg, s.light.daylight > 0 ? "day" : "night");
  eol();
  eol();

  // Serial log
  out("  \x1b[2mserial ─────────────────────────────────────────\x1b[0m");
  eol();
  const size_t N = 10;
  size_t from = s.lines.size() > N ? s.lines.size() - N : 0;
  for (size_t i = 0; i < N; i++) {
    if (from + i < s.lines.size()) {
      std::string text = s.lines[from + i].text.substr(0, 110);
      out("  \x1b[2m[%8.1f]\x1b[0m %s", s.lines[from + i].at_us / 1e6, text.c_str());
    }
    eol();
  }
  if (ui.typing) out("  \x1b[1m> %s\x1b[7m \x1b[0m", ui.input.c_str());
  else out("  \x1b[2m%s\x1b[0m", ui.note.c_str());
  eol();
  eol();

  out("  \x1b[1mcar\x1b[0m    c pull up outside/drive off  i park inside  w drive in  x stop  s reverse out  r remote  h headlights  f flash  F double flash"); eol();
  out("  \x1b[1mpanel\x1b[0m  b GUI button  B hold 2.5 s (save)  [ ] knob  { } spin  e ESP door input"); eol();
  out("  \x1b[1mother\x1b[0m  : serial command  S STATE  n day/night  +/- speed  p pause  P power cycle  q quit"); eol();
  frame += "\x1b[J";
  fwrite(frame.data(), 1, frame.size(), stdout);
  fflush(stdout);
}

// ------------------------------------------------------------------- input --

static bool handle_key(sim::Sim &s, Ui &ui, int c) {
  if (ui.typing) {
    if (c == '\n' || c == '\r') {
      s.send(ui.input + "\n");
      ui.note = "sent: " + ui.input;
      ui.input.clear();
      ui.typing = false;
    } else if (c == 27) {
      ui.input.clear();
      ui.typing = false;
    } else if (c == 127 || c == 8) {
      if (!ui.input.empty()) ui.input.pop_back();
    } else if (c >= 32 && c < 127) {
      ui.input += (char)c;
    }
    return true;
  }

  switch (c) {
    case 'q': return false;
    case 'c':
      if (s.car.present) { s.car_gone(); ui.note = "car drove off"; }
      else { s.car_outside(); ui.note = "car pulled up outside the door - r opens it, w drives in"; }
      break;
    case 'i':
      s.car_parked();
      ui.slow_from = -1;
      ui.note = "car parked inside";
      break;
    case 'w':
      s.drive_to(s.garage.depth_cm, 150);
      ui.slow_from = s.pos_for_reading(PARK_FAR_DISTANCE_DEFAULT) - 60;
      ui.note = "driving in - x to stop on the steady tone";
      break;
    case 's': s.drive_out(150); ui.slow_from = -1; ui.note = "reversing out"; break;
    case 'x': s.car.target_cm = s.car.pos_cm; ui.slow_from = -1; ui.note = "car stopped"; break;
    case 'r': s.remote(); ui.note = "pressed the car's remote"; break;
    case 'h': s.car.headlights = !s.car.headlights; s.car.present = true; break;
    case 'f': s.flash(1); ui.note = "flash"; break;
    case 'F': s.flash(2); ui.note = "flash flash"; break;
    case 'b': s.tap(guiBtn1, 150); ui.note = "GUI button"; break;
    case 'B': s.tap(guiBtn1, CONFIG_SAVE_HOLD + 500); ui.note = "GUI button held"; break;
    case ']': s.turn(+1, 1, 150); break;
    case '[': s.turn(-1, 1, 150); break;
    case '}': s.turn(+1, 10, 30); break;
    case '{': s.turn(-1, 10, 30); break;
    case 'e': s.tap(externalDoorPin, 300); ui.note = "ESP door input"; break;
    case ':': ui.typing = true; break;
    case 'S': s.send("STATE\n"); ui.note = "sent: STATE"; break;
    case 'n':
      s.light.daylight = s.light.daylight > 0 ? 0 : 350;
      ui.note = s.light.daylight > 0 ? "daytime" : "night";
      break;
    case '+': case '=': ui.speed = std::min(64.0, ui.speed * 2); break;
    case '-': ui.speed = std::max(0.125, ui.speed / 2); break;
    case 'p': ui.paused = !ui.paused; break;
    case 'P': s.power_cycle(); ui.note = "power cycled"; break;
    case 27: {  // swallow arrow keys and other escape sequences
      unsigned char next;
      while (read(STDIN_FILENO, &next, 1) == 1 && !isalpha(next)) {}
      break;
    }
  }
  return true;
}

int main(int argc, char **argv) {
  sim::Sim s;
  Ui ui;
  ui.note = "c: a car pulls up outside, r: its remote, w: drive in - or i to start parked inside";

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto val = [&]() -> double {
      if (i + 1 >= argc) { fprintf(stderr, "%s needs a value\n", a.c_str()); exit(2); }
      return atof(argv[++i]);
    };
    if (a == "-h" || a == "--help") { fputs(USAGE, stdout); return 0; }
    else if (a == "--speed") ui.speed = val();
    else if (a == "--parked") s.car_parked();
    else if (a == "--daylight") s.light.daylight = val();
    else if (a == "--bounce") s.bounce_ms = val();
    else if (a == "--door-s") s.door.travel_s = val();
    else if (a == "--garage-cm") s.garage.depth_cm = val();
    else if (a == "--sonar-at") s.garage.sensor_x_cm = val();
    else if (a == "--ceiling") s.garage.sensor_h_cm = val();
    else { fprintf(stderr, "unknown option %s (try --help)\n", a.c_str()); return 2; }
  }
  if (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO)) {
    fprintf(stderr, "garage_sim is interactive; run it in a terminal\n");
    return 2;
  }

  raw_tty();
  s.power_on();

  using clock = std::chrono::steady_clock;
  auto last = clock::now();
  for (;;) {
    pollfd p{STDIN_FILENO, POLLIN, 0};
    poll(&p, 1, 33);
    unsigned char c;
    bool quit = false;
    while (read(STDIN_FILENO, &c, 1) == 1)
      if (!handle_key(s, ui, c)) { quit = true; break; }
    if (quit) break;

    auto now = clock::now();
    double dt = std::chrono::duration<double>(now - last).count();
    last = now;
    if (!ui.paused) {
      // Never fall more than a quarter second of wall time behind.
      uint64_t target = s.now_us + (uint64_t)(std::min(dt, 0.25) * ui.speed * 1e6);
      while (s.now_us < target) {
        if (ui.slow_from >= 0 && s.car.pos_cm >= ui.slow_from) {
          s.car.speed = 20;
          ui.slow_from = -1;
        }
        s.loop_once();
      }
    }
    draw(s, ui);
  }
  restore_tty();
  return 0;
}
