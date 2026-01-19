# Loop Production Firmware

This repository contains the production firmware for the Loop project.

## Structure
- `LCM_FullMotorControl_V21`: Motor control firmware
- `MCM_V16`: Main control module firmware
- `TWM_GEM_V3_IMU`: IMU specific code

### This branch is for development of firmware within standard C++ architecture

This modular structure separates "data" from "logic" and "configuration" from "code," which makes the system easier to read, debug, and expand.

Here is the high-level breakdown of how each file type works in your new C++ architecture.

1. src/config.h (The "Settings" File)
This is the single source of truth for all hardware and system settings.

What goes here: Pin numbers (GPIO_NUM_35), fixed constants (wheel diameter, tank size), timeouts (HEARTBEAT_TIMEOUT_MS), and CAN IDs.

How it works: Every other file includes this header. When you need to change a pin or a speed setting, you change it here once, and it automatically updates everywhere in the project.

Why: It prevents "magic numbers" buried deep in your code (e.g., digitalWrite(4, HIGH) becoming digitalWrite(Config::RELAY_WATER_INLET, HIGH)).

2. src/types.h (The "Dictionary")
This file defines the shapes of your data, but not the data itself.

What goes here: struct definitions (like MotorState, ValveController) and enum lists (like MixerStatus, CanID).

How it works: This file is included by globals.h and helpers.h so that they know what a "MotorState" looks like (i.e., that it has rpm, direction, etc.).

Why: Putting these in a separate file prevents circular dependency errors (where File A needs File B, but File B needs File A). It ensures everyone agrees on the data structure before any code is compiled.

3. src/globals.h & .cpp (The "Shared Memory")
This is the most critical part for converting from Arduino .ino files. It manages variables that need to be seen by multiple tasks (like the state object).

globals.h (The Promise): It uses the extern keyword to tell other files: "Trust me, a variable named state exists somewhere. Here is its type." It does not create the variable.

globals.cpp (The Reality): This file actually reserves the memory for those variables.

Why: If you defined int myVar = 0; in a header file, the compiler would try to create a new copy of myVar for every single file that included that header, causing "Multiple Definition" errors. This split ensures the variable is created exactly once but used everywhere.

4. src/helpers.h & .cpp (The "Tools")
These files contain pure logic and hardware drivers that support the main tasks.

What goes here:

Math: calculate_pressure(), to_kg_per_ha().

Hardware Wrappers: setupRMT(), setBitbangFrequency().

Algorithms: PID control loops, polynomial correction logic.

How it works: A task (like motorTask) calls these functions to do specific jobs.

Why: It keeps your main task loops clean. Instead of 50 lines of bit-shifting code inside your main loop, you just see send_step_move_ack(motor_id). It also isolates hardware-specific code (like ESP32 RMT drivers), making it easier to port to STM32 later.

5. src/tasks.h & .cpp (The "Workers")
This is where the actual active behavior of your firmware lives.

What goes here: The FreeRTOS task functions (e.g., can_rx_task, sensor_task). These functions usually contain while(true) loops that run forever.

How it works:

The .h file declares the task names so the main .ino file can start them.

The .cpp file contains the logic: checking queues, reading sensors, and updating the global state.

Why: It separates concurrent activities. Your CAN bus listener doesn't need to know how the valve controller works; it just updates the global state, and the valve task reacts to that change independently.

Visualizing the Flow
Config & Types define the "Rules" and "Language."

Globals create the "Whiteboard" that everyone looks at.

Tasks are the "Employees" doing the work (reading sensors, running motors).

Helpers are the "Power Tools" the employees use to do their jobs efficiently.

Main (.ino) is the "Manager" who hires the employees (starts the tasks) and then goes to sleep.