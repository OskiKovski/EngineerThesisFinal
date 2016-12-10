//
// Created by Oskar Kowalski on 08.12.2016.
//

#include <Arduino.h>
#include <TinyGPS.h>
#include <unity.h>

#ifdef UNIT_TEST

void testDistanceCalculation(void) {
  TinyGPS gps;
  TEST_ASSERT_EQUAL(gps.distance_between(52.248967,21.009390,52.247277,21.013376), 330.1);
}

void testBearingDetermination(void) {
  TinyGPS gps;
  TEST_ASSERT_EQUAL(gps.course_to(52.248967,21.009390,52.247277,21.013376), 124);
}

void setup() {
  UNITY_BEGIN();
}

void loop() {
  RUN_TEST(testDistanceCalculation);
  RUN_TEST(testBearingDetermination);
  UNITY_END();
}
#endif