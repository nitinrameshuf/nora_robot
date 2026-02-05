# Copyright 2026 nanostation
# Licensed under the Apache License, Version 2.0

# Robot's internal emotional state
# Used by emotion engine to express personality

# Header with timestamp
std_msgs/Header header

# Primary emotion (string representation)
string emotion

# Emotion intensity (0.0 to 1.0)
# 0.0 = barely perceptible, 1.0 = maximum expression
float32 intensity

# Valence: negative to positive emotional value
# -1.0 = very negative (sad, angry)
#  0.0 = neutral
# +1.0 = very positive (happy, excited)
float32 valence

# Arousal: calm to excited state
# 0.0 = very calm, relaxed
# 1.0 = very aroused, excited, alert
float32 arousal

# Contributing factors (for debugging/logging)
float32 battery_contribution    # Low battery → tired
float32 temperature_contribution # High temp → anxious
float32 novelty_contribution    # High novelty → curious
float32 social_contribution     # Human interaction → engaged
float32 loneliness_contribution # Time alone → lonely

# Emotion categories (common emotions)
# For reference, not enforced:
# - neutral, happy, sad, curious, tired, anxious
# - alert, thinking, confused, excited, lonely
# - playful, focused, overwhelmed
