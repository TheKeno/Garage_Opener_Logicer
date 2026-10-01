// The on-device config screen: GUI button + rotary encoder + LCD.
#include "check.h"

#include "AccelDial.h"
#include "LightPulseSensor.h"

using namespace garage;

static void enter_config(sim::Sim &s) {
  s.tap(guiBtn1);
  s.run_ms(300);
}

// The cursor moves on release, and the LCD catches up on its next 250 ms
// refresh.
static void next_item(sim::Sim &s) {
  s.tap(guiBtn1);
  s.run_ms(150 + LCD_UPDATE_INTERVAL + 100);
}

static void spin(sim::Sim &s, int dir, int clicks, double interval_ms) {
  s.turn(dir, clicks, interval_ms);
  s.run_ms(clicks * interval_ms + 300);
}

static int16_t eeprom_threshold(const sim::Sim &s, int index) {
  int16_t v;
  memcpy(&v, &s.eeprom[1 + 2 * index], 2);
  return v;
}

TEST(config, button_enters_config_on_the_first_item) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  enter_config(s);
  CHECK_EQ(s.state(), std::string("Config"));
  CHECK_EQ(s.lcd(1), std::string("Upper: 300      "));
}

TEST(config, slow_clicks_step_by_one_both_ways) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  enter_config(s);
  spin(s, +1, 5, 300);
  CHECK_EQ(s.lcd(1), std::string("Upper: 305      "));
  spin(s, -1, 3, 300);
  CHECK_EQ(s.lcd(1), std::string("Upper: 302      "));
  CHECK_EQ(s.global<LightPulseSensor>("lightPulseSensor")->upper_threshold, 302);
}

TEST(config, a_fast_spin_accelerates) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  enter_config(s);
  spin(s, +1, 12, 20);
  CHECK_HAS(s.command("STATE"), "Upper=");
  int v = s.global<LightPulseSensor>("lightPulseSensor")->upper_threshold;
  CHECK(v > 300 + 12 * 2);          // well past one count per click
}

TEST(config, value_stops_at_the_bounds) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  enter_config(s);
  s.command("SET Upper 3");
  spin(s, -1, 10, 300);
  CHECK_EQ(s.lcd(1), std::string("Upper: 0        "));
  spin(s, +1, 2, 300);
  CHECK_EQ(s.lcd(1), std::string("Upper: 2        "));
}

TEST(config, short_presses_cycle_through_all_five_and_wrap) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  enter_config(s);
  const char *want[] = {"Lower: 200", "Dist: 45", "ParkFar: 50", "ParkNear: 15", "Upper: 300"};
  for (const char *w : want) {
    next_item(s);
    CHECK_HAS(s.lcd(1), w);
  }
}

TEST(config, dial_edits_only_the_item_under_the_cursor) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  enter_config(s);
  next_item(s);                     // Lower
  spin(s, +1, 4, 300);
  CHECK_HAS(s.command("STATE"), "Upper=300 Lower=204");
}

TEST(config, narrow_range_clamps_even_when_spun_fast) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  enter_config(s);
  next_item(s);
  next_item(s);                     // Dist, 5..60
  spin(s, +1, 30, 30);
  CHECK_HAS(s.lcd(1), "Dist: 60");
  spin(s, -1, 80, 30);
  CHECK_HAS(s.lcd(1), "Dist: 5 ");
  spin(s, +1, 3, 300);
  CHECK_HAS(s.lcd(1), "Dist: 8 ");
}

TEST(config, long_press_saves_and_leaves) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  enter_config(s);
  next_item(s);
  next_item(s);                     // Dist
  spin(s, -1, 3, 300);
  CHECK_EQ(s.eeprom_writes, 0u);    // nothing written while editing
  s.tap(guiBtn1, CONFIG_SAVE_HOLD + 200);
  CHECK(wait_state(s, "Idle", CONFIG_SAVE_HOLD + 500));
  CHECK_EQ(s.eeprom[0], 1);
  CHECK_EQ(eeprom_threshold(s, 2), 42);
  s.run_ms(500);                    // the release must not do anything else
  CHECK_EQ(s.state(), std::string("Idle"));
}

TEST(config, set_over_serial_moves_the_dial_too) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  enter_config(s);
  CHECK_EQ(s.command("SET Upper 700"), std::string("OK Upper=700"));
  CHECK_EQ(s.lcd(1), std::string("Upper: 700      "));
  spin(s, +1, 1, 300);              // the next click starts from 700, not 300
  CHECK_EQ(s.lcd(1), std::string("Upper: 701      "));
}

TEST(config, knob_turned_outside_config_is_not_banked) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  spin(s, +1, 5, 300);              // in idle: nobody listening
  enter_config(s);
  spin(s, +1, 1, 300);
  CHECK_EQ(s.lcd(1), std::string("Upper: 301      "));
}

TEST(config, light_flashes_do_nothing_while_configuring) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  enter_config(s);
  s.flash(2);
  s.run_ms(3000);
  CHECK_EQ(s.state(), std::string("Config"));
  CHECK_EQ(s.door.toggles, 0);
}

TEST(config, no_clicks_lost_at_a_brisk_turn) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  enter_config(s);
  auto *dial = s.global<AccelDial>("configDial");
  long before = dial->detents();
  spin(s, +1, 60, 50);              // 20 clicks/s
  CHECK_EQ(dial->detents() - before, 60);
  CHECK_EQ(dial->missedClicks(), 0ul);
}

TEST(config, no_clicks_lost_in_an_empty_garage) {
  sim::Sim s;
  boot(s);                          // the 1 Hz car check now waits ~10 ms for the floor echo
  enter_config(s);
  auto *dial = s.global<AccelDial>("configDial");
  long before = dial->detents();
  spin(s, +1, 100, 50);
  CHECK_EQ(dial->detents() - before, 100);
}
