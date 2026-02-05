# Copyright 2026 nanostation
# Licensed under the Apache License, Version 2.0

# Robot state machine representation
# Defines high-level operational state of Nora

# Header with timestamp
std_msgs/Header header

# State enumeration (string for readability, uint8 for efficiency)
uint8 INITIALIZING = 0
uint8 IDLE = 1
uint8 EXPLORING = 2
uint8 INTERACTING = 3
uint8 PROCESSING = 4
uint8 ANALYZING = 5
uint8 RETURNING = 6
uint8 DOCKED = 7
uint8 ERROR = 8
uint8 SHUTDOWN = 9

uint8 state

# State name (human-readable)
string state_name

# Battery level (0.0 to 1.0)
float32 battery_level

# Temperature (Celsius)
float32 temperature

# Power mode (low_power=0, balanced=1, high_performance=2)
uint8 POWER_LOW = 0
uint8 POWER_BALANCED = 1
uint8 POWER_HIGH = 2

uint8 power_mode

# WiFi connectivity
bool wifi_connected

# Cluster availability
bool cluster_available

# Time in current state (seconds)
float32 time_in_state

# Additional status flags
bool emergency_stop_active
bool manual_control_active
