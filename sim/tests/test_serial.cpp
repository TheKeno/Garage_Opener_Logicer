// The UART control protocol the ESP8266 (or a terminal) speaks:
// OPEN / CLOSE / STATE / SET <name> <value> / SAVE.
#include "check.h"

#include "LightPulseSensor.h"

using namespace garage;

TEST(serial, empty_garage_reads_the_floor) {
  sim::Sim s;
  boot(s);
  CHECK_HAS(s.command("STATE"), "car=no range=16");  // 165 cm, read ~1% short
}

TEST(serial, state_reports_everything_on_one_line) {
  sim::Sim s;
  car_at_reading(s, 42);
  boot(s);
  std::string r = s.command("STATE");
  CHECK_HAS(r, "OK state=idle door=closed car=yes range=4");  // 41 or 42 at 343 m/s
  CHECK_HAS(r, "Upper=300 Lower=200 Dist=45 ParkFar=50 ParkNear=15");
  CHECK_HAS(r, "gui=0");
}

TEST(serial, a_dead_sonar_reads_as_dashes_and_no_car) {
  sim::Sim s;
  car_parked(s);
  s.garage.sonar_ok = false;
  boot(s);
  std::string r = s.command("STATE");
  CHECK_HAS(r, "car=no range=--");
}

TEST(serial, open_when_shut_pulses_the_relay_and_runs_the_cycle) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  CHECK_EQ(s.command("OPEN", 1000), std::string("OK opening"));
  CHECK_EQ(s.door.toggles, 1);
  CHECK_EQ(s.door.motion, +1);
  CHECK_EQ(s.state(), std::string("Open door"));
  CHECK_EQ(s.command("OPEN"), std::string("ERR busy state=opening"));
  CHECK_EQ(s.command("CLOSE"), std::string("ERR busy state=opening"));
  CHECK_EQ(s.door.toggles, 1);
}

TEST(serial, open_and_close_answer_without_acting_when_already_there) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  CHECK_EQ(s.command("close"), std::string("OK already closed"));
  s.remote();                       // open it behind the sketch's back
  s.run_ms(s.door.travel_s * 1000 + 500);
  CHECK_EQ(s.command("open"), std::string("OK already open"));
  CHECK_EQ(s.command("CLOSE", 1000), std::string("OK closing"));
  CHECK_EQ(s.door.motion, -1);
  CHECK_EQ(s.door.toggles, 2);
}

TEST(serial, set_changes_live_values_and_clamps) {
  sim::Sim s;
  boot(s);
  CHECK_EQ(s.command("SET Upper 400"), std::string("OK Upper=400"));
  CHECK_EQ(s.global<LightPulseSensor>("lightPulseSensor")->upper_threshold, 400);
  CHECK_EQ(s.command("set lower 150"), std::string("OK Lower=150"));
  CHECK_EQ(s.global<LightPulseSensor>("lightPulseSensor")->lower_threshold, 150);
  CHECK_EQ(s.command("SET Dist 999"), std::string("OK Dist=60"));
  CHECK_EQ(s.command("SET Dist 1"), std::string("OK Dist=5"));
  CHECK_EQ(s.command("SET ParkFar 0"), std::string("OK ParkFar=15"));
  CHECK_EQ(s.command("SET ParkNear 70"), std::string("OK ParkNear=50"));
  CHECK_EQ(s.command("SET bogus 5"), std::string("ERR no setting called bogus"));
  CHECK_EQ(s.command("SET Upper"), std::string("ERR usage: SET <name> <value>"));
  CHECK_EQ(s.eeprom_writes, 0u);
}

TEST(serial, save_persists_across_a_power_cycle) {
  sim::Sim s;
  boot(s);
  s.command("SET Upper 400");
  s.command("SET Dist 30");
  CHECK_EQ(s.command("SAVE"), std::string("OK saved"));
  CHECK_EQ(s.eeprom[0], 1);         // marker written
  s.power_cycle();
  s.run_ms(300);
  CHECK_HAS(s.command("STATE"), "Upper=400 Lower=200 Dist=30");
}

TEST(serial, unsaved_set_is_lost_on_reset) {
  sim::Sim s;
  boot(s);
  s.command("SET Upper 400");
  s.power_cycle();
  s.run_ms(300);
  CHECK_HAS(s.command("STATE"), "Upper=300");
  CHECK_EQ(s.eeprom_writes, 0u);
}

TEST(serial, save_writes_only_bytes_that_changed) {
  sim::Sim s;
  boot(s);
  s.command("SAVE");
  CHECK_EQ(s.eeprom_writes, 11u);   // marker + five int16 thresholds
  s.command("SAVE");
  // EEPROM.put() skips unchanged bytes; the marker uses write() and is
  // rewritten every time - one byte of wear per SAVE.
  CHECK_EQ(s.eeprom_writes, 12u);
  s.command("SET ParkNear 16");
  s.command("SAVE");
  CHECK_EQ(s.eeprom_writes, 14u);
}

TEST(serial, garbage_containing_open_is_not_a_command) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  CHECK_EQ(s.command("HELLO"), std::string("ERR unknown, try OPEN CLOSE STATE SET SAVE"));
  // ESP8266 ROM boot log, read at the wrong baud.
  CHECK_EQ(s.command("\xff\x1b\x80OPENISH\x03 rst:0x1 ap\xf0"),
           std::string("ERR unknown, try OPEN CLOSE STATE SET SAVE"));
  CHECK_EQ(s.door.toggles, 0);
  CHECK_EQ(s.state(), std::string("Idle"));
}

TEST(serial, overlong_line_is_rejected_whole) {
  sim::Sim s;
  boot(s);
  CHECK_EQ(s.command(std::string(60, 'x')), std::string("ERR line too long"));
  CHECK_HAS(s.command("STATE"), "OK state=idle");
  // 31 characters fits the buffer; 32 does not.
  CHECK_EQ(s.command("SET Upper 300" + std::string(18, ' ')), std::string("OK Upper=300"));
  CHECK_EQ(s.command("SET Upper 300" + std::string(19, ' ')), std::string("ERR line too long"));
}

TEST(serial, crlf_line_endings_work) {
  sim::Sim s;
  boot(s);
  size_t from = s.lines.size();
  s.send("STATE\r\n");
  s.run_ms(50);
  CHECK_EQ(count_lines(s, "OK state=idle", from), 1);
  CHECK_EQ(count_lines(s, "ERR", from), 0);
}

TEST(serial, one_command_per_loop_pass) {
  sim::Sim s;
  boot(s);
  size_t from = s.lines.size();
  s.send("STATE\nSTATE\n");
  s.advance(2000);                  // both lines land while the sketch is busy
  s.loop_once();
  CHECK_EQ(count_lines(s, "OK state=", from), 1);
  s.loop_once();
  CHECK_EQ(count_lines(s, "OK state=", from), 2);
}

TEST(serial, commands_sent_during_the_relay_pulse_are_answered_after_it) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  size_t from = s.lines.size();
  s.send("OPEN\nSTATE\n");          // STATE lands while send_door_signal() blocks
  s.run_ms(800);
  CHECK_EQ(count_lines(s, "OK opening", from), 1);
  CHECK_EQ(count_lines(s, "OK state=opening", from), 1);
}

// The relay pulse blocks loop() for 510 ms. The AVR's RX ring holds 63 bytes;
// anything beyond is lost - the sketch must come out of that in sync.
TEST(serial, rx_overflow_during_the_relay_pulse_recovers) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.send("OPEN\n");
  s.at(100, [&] {                   // mid-pulse: 140 bytes into a 63-byte ring
    for (int i = 0; i < 10; i++) s.send("SET Upper 301\n");
  });
  s.run_ms(1500);
  CHECK(s.rx_dropped > 0);
  // The torn last line merges with the next one and earns a single ERR; after
  // that the protocol is back in step.
  CHECK_HAS(s.command("STATE"), "ERR");
  CHECK_HAS(s.command("STATE"), "OK state=opening");
}

TEST(serial, state_reply_does_not_stall_the_loop) {
  sim::Sim s;
  car_at_reading(s, 42);
  boot(s);
  s.run_ms(1100);                   // let the fps counter fill in
  std::string r = s.command("STATE");
  CHECK(r.size() < 140);
  s.longest_loop_us = 0;
  s.command("STATE");
  // At 115200 baud the part beyond the 64-byte TX buffer drains in ~5 ms.
  CHECK(s.longest_loop_us < 12000);
}
