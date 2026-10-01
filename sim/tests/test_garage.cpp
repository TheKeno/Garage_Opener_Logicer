// Whole-garage scenarios: the car, the door, the lights and the park assist.
#include "check.h"

using namespace garage;

// ------------------------------------------------------------------- boot --

TEST(boot, comes_up_idle_with_everything_off) {
  sim::Sim s;
  boot(s);
  CHECK_EQ(s.lines.at(0).text, std::string("# Garage opener R6 ready"));
  CHECK_EQ(s.state(), std::string("Idle"));
  CHECK_HAS(s.lcd(1), "D:16");      // the floor, 165 cm down
  CHECK_EQ(s.pin(doorPin), LOW);
  CHECK_EQ(s.pin(ledRedPin), LOW);
  CHECK_EQ(s.pin(ledGreenPin), LOW);
  CHECK_EQ(s.pin(buzzerPin), LOW);
  CHECK_EQ(s.door.toggles, 0);
}

TEST(boot, blank_eeprom_gives_the_compiled_defaults) {
  sim::Sim s;
  boot(s);
  CHECK_HAS(s.command("STATE"), "Upper=300 Lower=200 Dist=45 ParkFar=50 ParkNear=15");
}

TEST(boot, corrupt_eeprom_is_clamped_into_range) {
  sim::Sim s;
  s.eeprom[0] = 1;                  // "valid" marker over leftover junk
  for (int i = 1; i <= 10; i++) s.eeprom[i] = (i % 2) ? 0xFF : 0x7F;  // 32767 each
  boot(s);
  CHECK_HAS(s.command("STATE"), "Upper=1023 Lower=1023 Dist=60 ParkFar=100 ParkNear=50");
}

TEST(boot, status_pins_follow_door_and_car) {
  sim::Sim s;
  car_at_reading(s, 30);
  boot(s);
  s.run_ms(1100);
  CHECK_EQ(s.pin(doorStatus), HIGH);
  CHECK_EQ(s.pin(carStatus), HIGH);
  s.car_gone();
  s.remote();
  s.run_ms(2100);
  CHECK_EQ(s.pin(doorStatus), LOW);
  CHECK_EQ(s.pin(carStatus), LOW);
}

TEST(boot, lcd_alternates_distance_and_fps_every_five_seconds) {
  sim::Sim s;
  car_at_reading(s, 30);
  boot(s);
  s.run_until([&] { return s.fw_millis() % 10000 > 500 && s.fw_millis() % 10000 < 4000; }, 11000);
  CHECK_HAS(s.lcd(1), "D:");
  CHECK_HAS(s.lcd(1), "L:");
  s.run_ms(5000);
  CHECK_HAS(s.lcd(1), "F:");
  CHECK_HAS(s.lcd(1), "A:");
}

// ------------------------------------------------------- coming home --

TEST(arrive, remote_into_an_empty_garage_starts_park_assist) {
  sim::Sim s;
  boot(s);
  s.car_outside();
  s.remote();
  CHECK(wait_state(s, "Parking", 200));
  CHECK_EQ(s.pin(ledRedPin), HIGH);
  CHECK_EQ(s.pin(buzzerPin), LOW);
  CHECK_EQ(s.door.toggles, 1);      // the sketch did not touch the door itself
}

TEST(arrive, park_assist_beeps_faster_then_holds_and_finishes) {
  sim::Sim s;
  boot(s);
  s.car_outside();
  s.remote();
  s.run_ms(s.door.travel_s * 1000 + 500);
  s.drive_to(s.pos_for_reading(PARK_NEAR_DISTANCE_DEFAULT) + 5, 10);  // slowly: plenty of beeps

  // Silent until the windshield comes within ParkFar of the sensor. (The
  // sketch's 340 m/s and integer truncation read ~1% short, hence the margin.)
  uint64_t far_at = 0, near_at = 0;
  s.run_until([&] { if (!far_at && s.sonar_cm() <= 52) far_at = s.now_us; return far_at != 0; }, 120000);
  CHECK_EQ(rises(s, buzzerPin).size(), 0u);
  CHECK_EQ(s.state(), std::string("Parking"));

  s.run_until([&] { return s.sonar_cm() <= 14; }, 10000);
  near_at = s.now_us;
  auto beeps = rises(s, buzzerPin, far_at, near_at);
  CHECK(beeps.size() >= 4);
  if (beeps.size() >= 4) {
    double first = (beeps[1] - beeps[0]) / 1000.0, last = (beeps.back() - beeps[beeps.size() - 2]) / 1000.0;
    CHECK(first > last + 200);      // clicks speed up as the car closes in
  }

  // At the goal: green, steady tone, then done after PARK_HOLD_TIME.
  s.run_ms(300);
  CHECK_EQ(s.pin(ledGreenPin), HIGH);
  CHECK_EQ(s.pin(ledRedPin), LOW);
  CHECK_EQ(s.pin(buzzerPin), HIGH);
  CHECK(wait_state(s, "Idle", PARK_HOLD_TIME + 500));
  CHECK_NEAR((s.now_us - near_at) / 1000.0, PARK_HOLD_TIME, 350);
  CHECK_EQ(s.pin(ledGreenPin), LOW);
  CHECK_EQ(s.pin(buzzerPin), LOW);
}

TEST(arrive, nobody_arrives_and_park_assist_times_out) {
  sim::Sim s;
  boot(s);
  s.remote();                       // door opened, no car
  CHECK(wait_state(s, "Parking", 200));
  uint64_t t0 = s.now_us;
  CHECK(wait_state(s, "Idle", PARKING_TIMEOUT + 1000));
  CHECK_NEAR((s.now_us - t0) / 1000.0, PARKING_TIMEOUT, 200);
  CHECK_EQ(rises(s, buzzerPin).size(), 0u);
  CHECK_EQ(s.pin(ledRedPin), LOW);
}

TEST(arrive, stopping_short_of_the_goal_does_not_finish) {
  sim::Sim s;
  boot(s);
  s.car_outside();
  s.remote();
  s.run_ms(s.door.travel_s * 1000 + 500);
  s.drive_to(s.pos_for_reading(25), 40);
  s.run_ms(25000);
  CHECK_EQ(s.state(), std::string("Parking"));
  CHECK_EQ(s.pin(ledRedPin), HIGH);
  CHECK(rises(s, buzzerPin).size() > 3);  // still clicking
}

TEST(arrive, remote_with_a_car_already_inside_is_not_an_arrival) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.remote();
  s.run_ms(2000);
  CHECK_EQ(s.state(), std::string("Idle"));
  CHECK_EQ(count_lines(s, "PARKING"), 0);
}

TEST(arrive, one_flash_after_parking_closes_the_door) {
  sim::Sim s;
  boot(s);
  s.car_outside();
  s.remote();
  s.run_ms(s.door.travel_s * 1000 + 500);
  drive_in(s);
  CHECK(wait_state(s, "Idle", 60000));

  s.run_ms(3000);
  s.flash(1);
  CHECK(wait_state(s, "Wait second", 500));
  CHECK(wait_state(s, "Close door", LIGHT_TIMEOUT + 300));
  CHECK_EQ(s.door.motion, -1);
  CHECK(wait_state(s, "Idle", DOOR_DELAY + 500));
  CHECK(s.door.shut());
}

// --------------------------------------------------------------- leaving --

TEST(leave, two_flashes_open_the_door) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.run_ms(2000);
  s.flash(2);
  CHECK(wait_state(s, "Open door", 1500));
  CHECK_EQ(s.door.motion, +1);
  CHECK_EQ(s.door.toggles, 1);
}

TEST(leave, flashes_too_far_apart_do_nothing) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.run_ms(2000);
  s.flash(2, 150, LIGHT_TIMEOUT + 200);
  s.run_ms(2 * LIGHT_TIMEOUT + 1500);
  CHECK_EQ(count_lines(s, "State switch: OPEN"), 0);
  CHECK_EQ(count_lines(s, "WAIT_FOR_SECOND_SIGNAL"), 2);
  CHECK_EQ(s.door.toggles, 0);
  CHECK_EQ(s.state(), std::string("Idle"));
}

// A light on for longer than LIGHT_PULSE_TIMEOUT is not a flash - including
// the moment it goes off, which the detector once counted as one.
TEST(leave, a_long_light_is_not_a_flash) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.run_ms(2000);
  for (double on_ms : {LIGHT_PULSE_TIMEOUT + 300.0, 3000.0, 10000.0}) {
    s.flash(1, on_ms, 0);           // headlights left on, then switched off
    s.run_ms(on_ms + 3000);
  }
  CHECK_EQ(count_lines(s, "WAIT_FOR_SECOND_SIGNAL"), 0);
  // ...and real flashes straight afterwards still work.
  s.flash(2);
  CHECK(wait_state(s, "Open door", 1500));
}

TEST(leave, three_flashes_still_just_open) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.run_ms(2000);
  s.flash(3);
  s.run_ms(DOOR_DELAY + 500);
  CHECK_EQ(s.door.toggles, 1);
}

TEST(leave, flashes_are_ignored_with_no_car_inside) {
  sim::Sim s;
  boot(s);
  s.remote();                       // door open, car in the driveway, lights on the sensor
  s.run_ms(s.door.travel_s * 1000 + 500);
  s.car_outside();
  s.run_ms(PARKING_TIMEOUT);        // let the remote-triggered park assist give up
  CHECK_EQ(s.state(), std::string("Idle"));
  size_t from = s.lines.size();
  s.flash(2);
  s.run_ms(3000);
  CHECK_EQ(count_lines(s, "State switch", from), 0);
}

TEST(leave, door_cycle_then_drive_out) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.flash(2);
  CHECK(wait_state(s, "Open door", 1500));
  s.run_ms(s.door.travel_s * 1000 + 500);
  s.drive_out();                    // reverse out before PARKING starts
  CHECK(wait_state(s, "Parking", DOOR_DELAY));
  CHECK(wait_state(s, "Idle", PARKING_TIMEOUT + 1000));
  CHECK_EQ(rises(s, buzzerPin).size(), 0u);
}

// Opening the door for a car that is still parked used to hand over to park
// assist after DOOR_DELAY, which beeped "parked" at it.
TEST(leave, no_parked_beep_at_a_car_that_has_not_left_yet) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.flash(2);
  CHECK(wait_state(s, "Open door", 1500));
  s.run_ms(DOOR_DELAY + PARK_HOLD_TIME + 1000);  // driver still buckling up
  CHECK_EQ(high_ms(s, buzzerPin, 0, s.now_us), 0.0);
  CHECK_EQ(count_lines(s, "PARKING"), 0);
  CHECK_EQ(s.state(), std::string("Idle"));
  CHECK_EQ(s.pin(ledRedPin), LOW);
}

TEST(leave, door_opened_for_a_car_that_then_leaves_late) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.flash(2);
  CHECK(wait_state(s, "Open door", 1500));
  CHECK(wait_state(s, "Idle", DOOR_DELAY + 500));
  s.run_ms(5000);
  s.drive_out();
  s.run_ms(15000);
  // Gone, door left open - and nothing woken up on the way out.
  CHECK_EQ(count_lines(s, "PARKING"), 0);
  CHECK_EQ(rises(s, buzzerPin).size(), 0u);
  CHECK(s.door.open());
}

// ----------------------------------------------------------------- light --

TEST(light, courtesy_lamp_and_daylight_through_the_door_are_not_flashes) {
  sim::Sim s;
  s.light.daylight = 400;           // bright day outside
  car_parked(s);
  boot(s);
  s.run_ms(5000);
  s.remote();                       // door rolls up, lamp on, daylight floods in
  s.run_ms(30000);
  s.remote();                       // and back down
  s.run_ms(30000);
  CHECK_EQ(count_lines(s, "WAIT_FOR_SECOND_SIGNAL"), 0);
  CHECK_EQ(s.state(), std::string("Idle"));
}

// Arriving in daylight with the headlights on, then switching them off, once
// looked like a single flash - which with the door open means "close it".
TEST(light, switching_headlights_off_after_parking_leaves_the_door_alone) {
  for (double day : {0.0, 300.0}) {
    sim::Sim s;
    s.light.daylight = day;
    boot(s);
    s.run_ms(60000);
    s.car_outside();
    s.car.headlights = true;
    s.remote();
    s.run_ms(s.door.travel_s * 1000 + 500);
    drive_in(s);
    CHECK(wait_state(s, "Idle", 60000));
    s.run_ms(5000);
    s.car.headlights = false;
    s.run_ms(LIGHT_TIMEOUT + 3000);
    CHECK_EQ(count_lines(s, "WAIT_FOR_SECOND_SIGNAL"), 0);
    CHECK(s.door.open());
  }
}

TEST(light, lamp_switching_off_is_not_a_flash) {
  sim::Sim s;
  s.light.lamp_s = 20;
  car_parked(s);
  boot(s);
  s.remote();
  s.run_ms(s.door.travel_s * 1000 + s.light.lamp_s * 1000 + 5000);
  CHECK_EQ(count_lines(s, "WAIT_FOR_SECOND_SIGNAL"), 0);
}

TEST(light, flashes_work_with_the_courtesy_lamp_on) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.remote();
  s.run_ms(30000);                  // door open, lamp still lit
  s.flash(2);
  s.run_ms(1500);
  // Door is open, so two flashes is "open" with nothing to open: back to idle.
  CHECK_EQ(count_lines(s, "WAIT_FOR_SECOND_SIGNAL"), 1);
  CHECK_EQ(count_lines(s, "State switch: IDLE"), 1);
  CHECK_EQ(s.door.toggles, 1);
}

TEST(light, flashes_during_the_door_cycle_are_not_saved_up) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.flash(2);
  CHECK(wait_state(s, "Open door", 1500));
  s.run_ms(3000);
  s.flash(1);                       // ignored: the door is moving
  CHECK(wait_state(s, "Idle", DOOR_DELAY));
  s.run_ms(3000);
  CHECK_EQ(count_lines(s, "WAIT_FOR_SECOND_SIGNAL"), 1);
}

// ------------------------------------------------------------------ door --

TEST(door, relay_pulse_is_500_ms_and_toggles_the_opener_once) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.command("OPEN", 1000);
  auto up = rises(s, doorPin);
  CHECK_EQ(up.size(), 1u);
  CHECK_NEAR(high_ms(s, doorPin, 0, s.now_us), 500, 1);
  CHECK_EQ(s.pin(doorPin), LOW);
  CHECK_EQ(s.door.toggles, 1);
}

TEST(door, external_button_toggles_the_door_from_idle) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.tap(externalDoorPin);
  CHECK(wait_state(s, "Open door", 300));
  CHECK_EQ(s.door.motion, +1);
  s.run_ms(DOOR_DELAY + 300);
  CHECK(wait_state(s, "Idle", 500)); // car still under the sensor: no park assist
  s.tap(externalDoorPin);
  CHECK(wait_state(s, "Close door", 300));
  CHECK_EQ(s.door.motion, -1);
}

TEST(door, external_button_is_ignored_mid_cycle) {
  sim::Sim s;
  car_parked(s);
  boot(s);
  s.command("OPEN", 1000);
  s.run_ms(2000);
  s.tap(externalDoorPin);
  s.run_ms(2000);
  CHECK_EQ(s.door.toggles, 1);
  CHECK_EQ(count_lines(s, "External button"), 0);
}

TEST(door, bouncy_contacts_give_one_event_each) {
  sim::Sim s;
  s.bounce_ms = 8;
  boot(s);
  s.car_outside();
  s.remote();                       // microswitch chatters as the door lifts
  s.run_ms(2000);
  CHECK_EQ(count_lines(s, "State switch: PARKING"), 1);
  s.run_ms(PARKING_TIMEOUT);
  s.tap(externalDoorPin, 200);      // and so does the ESP's input
  s.run_ms(1000);
  CHECK_EQ(count_lines(s, "External button"), 1);
}

// ------------------------------------------------------------- the clock --

TEST(clock, a_full_arrival_across_the_millis_rollover) {
  // 49.7 days of uptime: millis() wraps 5 s into the scenario.
  sim::Sim s(((1ULL << 32) - 5000) * 1000);
  boot(s);
  s.car_outside();
  s.remote();
  CHECK(wait_state(s, "Parking", 200));
  s.run_ms(s.door.travel_s * 1000 + 500);
  CHECK(s.fw_millis() < 60000);     // wrapped
  drive_in(s);
  CHECK(wait_state(s, "Idle", 60000));
  s.run_ms(2000);
  s.flash(1);
  CHECK(wait_state(s, "Close door", LIGHT_TIMEOUT + 800));
  CHECK(wait_state(s, "Idle", DOOR_DELAY + 500));
  CHECK(s.door.shut());
}

TEST(clock, door_cycle_straddling_the_rollover_still_lasts_door_delay) {
  sim::Sim s(((1ULL << 32) - 8000) * 1000);
  boot(s);                          // empty garage, so the cycle ends in PARKING
  s.command("OPEN", 1000);
  uint64_t t0 = s.now_us;
  CHECK(wait_state(s, "Parking", DOOR_DELAY + 1000));
  CHECK_NEAR((s.now_us - t0) / 1000.0, DOOR_DELAY - 500, 50);  // the 500 ms relay pulse comes first
}
