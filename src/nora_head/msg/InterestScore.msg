# Copyright 2026 nanostation
# Licensed under the Apache License, Version 2.0

# Interest score for detected objects
# Used by interest scorer to guide autonomous exploration

# Header with timestamp
std_msgs/Header header

# Object tracking ID (from vision detector)
int32 tracking_id

# Object class name (e.g., "person", "plant", "cat")
string class_name

# Bounding box center (image coordinates)
float32 bbox_center_x
float32 bbox_center_y

# Distance estimate (meters, -1 if unknown)
float32 distance

# Overall interest score (0.0 to 1.0)
# Higher score = more interesting, should investigate
float32 score

# Score components (for transparency/debugging)
float32 novelty_score      # 0.0-1.0: How novel/new is this object?
float32 size_score         # 0.0-1.0: Larger objects more salient
float32 proximity_score    # 0.0-1.0: Closer objects more accessible
float32 category_score     # 0.0-1.0: Inherent interest (person > chair)
float32 confidence_score   # 0.0-1.0: Detection confidence from YOLO

# History tracking
int32 times_seen           # How many times encountered before
float32 last_seen_seconds  # Seconds since last encounter (-1 if first time)

# Recommended action
bool should_investigate    # True if robot should move toward this object
bool should_ignore         # True if object should be deprioritized
