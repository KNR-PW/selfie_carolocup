/**
 *Copyright ( c ) 2020, KNR Selfie
 *This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef CUSTOM_MSGS_TASK_ENUM_H
#define CUSTOM_MSGS_TASK_ENUM_H

// published on topic:
// /state/task
namespace selfie {
typedef enum EnumTask
{
  TASK_SHIFTING = 0, // Scheduler setting new action

  // STARTING PROCEDURE
  WAITING_FOR_BUTTON,        // Waiting for button press
  GATE_CLOSED,               // Starting gate is closed
  STARTING_DRIVE,            // Gate opened, car starts dirve
  STARTING_DISTANCE_REACHED, // Car drove given starting distance

  // FREE DRIVE
  AUTONOMOUS_DRIVE,           // Autonomous ride
  STARTING_LINE_DETECTED,     // Start line detected - parking zone incomming
  INTERSECTION_STOP_DETECTED, // Priority intersection detected
  EVENT_SENT,                 // Event is close - need special interaction

  // PARKING SPOT DETECTION
  SEARCHING_PARKING_SPOT, // Searching parking spot
  PLACE_INITIALLY_FOUND,  // Potential parking spot detected far away
  PLACE_PROPER_FOUND,     // Proper parking spot detected and sent
  PLACE_NOT_FOUND,        // Parking spot not found. Parking area finished

  // PARK
  APPROACHING_TO_PARKING_SPOT, // Car is approaching to parking spot
  ENTRY_PARKING_SPOT,          // Car drives into parking place
  PARKING_IDLE,                // Car parked
  EXIT_PARKING_SPOT,           // Car drives out of parking place
  PARKING_COMPLETED,           // Car ready to further ride

  // INTERSECTION
  APPROACHING_TO_INTERSECTION,   // Approaching to intersection
  BLIND_APPROACHING,             // Approaching to intersection when we can't see line
  STOP_TIME_ON_INTERSECTION,     // Car stopped because of time delay
  STOP_OBSTACLE_ON_INTERSECTION, // Car stopped because of another car on
                                 // intersection
  PASSING_INTERSECTION           // Intersection clear - car continues ride
} EnumTask;
} // namespace selfie

#endif // CUSTOM_MSGS_TASK_ENUM_H
