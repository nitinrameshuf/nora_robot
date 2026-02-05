# Copyright 2026 nanostation
# Licensed under the Apache License, Version 2.0

# Command to Raspberry Pi Pico for NeoPixel expression display
# Sent via serial USB to Pico for LED animation control

# Header with timestamp
std_msgs/Header header

# Expression name (matches Pico's programmed expressions)
string expression

# Valid expressions:
# - neutral: Calm, ambient breathing
# - happy: Bright, warm colors, upbeat animation
# - sad: Dim, cool colors, slow pulse
# - curious: Searching pattern, moderate brightness
# - thinking: Pulsing pattern, contemplative
# - alert: Bright, attention-grabbing
# - listening: Animated ears/attention indicator
# - speaking: Mouth/expression synced animation
# - sleeping: Very dim, slow breathing
# - tired: Low brightness, slow movements
# - error: Red flash/warning pattern
# - searching: Scanning/seeking pattern
# - excited: Fast, bright, energetic animation

# Brightness override (0.0 to 1.0, -1 = use default)
float32 brightness

# Animation speed multiplier (0.5 to 2.0, -1 = use default)
# 0.5 = half speed (slower)
# 1.0 = normal speed
# 2.0 = double speed (faster)
float32 speed

# Priority (0 = normal, 1 = high, 2 = critical)
# High priority interrupts current expression
# Critical priority (e.g., error) overrides everything
uint8 priority

# Duration (seconds, 0 = indefinite)
float32 duration
