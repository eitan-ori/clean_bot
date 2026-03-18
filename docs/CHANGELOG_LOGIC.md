# Logic and Flow Improvements

All logic/flow changes made during the cleanup (`cleanup/streamline-project` branch).

---

## Major Flow Changes

### 1. Unified `/clean` command (removed random cleaning)
- **Before:** Two cleaning modes — `/clean` (random walk) and `/coverage` (map-based). Random mode drove aimlessly with bump-and-turn logic.
- **After:** Single `/clean` command that always uses map-based adaptive coverage. The robot uses the SLAM map to plan efficient boustrophedon paths.
- **Files changed:** `full_mission.py`, `telegram_bridge.py`

### 2. Nav2 return-home navigation
- **Before:** `return_home_sequence()` was `time.sleep(1.0)` — a placeholder that did nothing.
- **After:** Uses Nav2 `NavigateToPose` action client to navigate back to `(home_x, home_y)` coordinates. Includes async goal acceptance and result callbacks with error handling.
- **File:** `full_mission.py`

### 3. Auto-send map on mission completion
- **Before:** After cleaning completed, the user had to manually send `/map` to get the final map.
- **After:** The Telegram bridge automatically sends the final map image to all users who have interacted with the bot when the mission state transitions to `COMPLETE`.
- **Implementation:** 
  - `state_callback()` detects `COMPLETE` transition and sets `_pending_completion` flag
  - `check_auth()` tracks chat IDs in `_notify_chat_ids` set
  - A repeating job (`_check_mission_complete`) polls every 3 seconds and sends the map when the flag is set
- **File:** `telegram_bridge.py`

### 4. Cleaning hardware safety on state transitions
- **Before:** Sending `go_home` or `reset` during coverage left the cleaning relay and servo running.
- **After:** Both `handle_go_home()` and `handle_reset()` now deactivate cleaning hardware if the current state is `COVERAGE`.
- **File:** `full_mission.py`

---

## Mission State Machine

### Updated State Flow
```
WAITING_FOR_SCAN → EXPLORING → WAITING_FOR_CLEAN → COVERAGE → RETURNING → COMPLETE
       ↑                                                                      │
       └──────────────────────── reset ───────────────────────────────────────┘
```

### State Transitions
| From | Command/Event | To | Notes |
|------|--------------|-----|-------|
| `WAITING_FOR_SCAN` | `start_scan` | `EXPLORING` | |
| `EXPLORING` | `stop_scan` or auto-complete | `WAITING_FOR_CLEAN` | |
| `WAITING_FOR_CLEAN` | `start_clean` | `COVERAGE` | Activates cleaning hardware |
| `WAITING_FOR_CLEAN` | `start_scan` | `EXPLORING` | Can re-scan |
| `WAITING_FOR_SCAN` | `start_clean` | `COVERAGE` | Warning logged, proceeds anyway |
| `COVERAGE` | auto-complete | `RETURNING` | Deactivates cleaning hardware |
| `COVERAGE` | `stop_clean` | `WAITING_FOR_CLEAN` | Deactivates cleaning hardware |
| `RETURNING` | Nav2 goal complete | `COMPLETE` | Auto-sends map via Telegram |
| Any | `go_home` | `RETURNING` | Stops everything, deactivates cleaning |
| Any | `reset` | `WAITING_FOR_SCAN` | Stops everything, deactivates cleaning |
| Any | `pause` | `PAUSED` | Stores previous state |
| `PAUSED` | `resume` | (previous state) | |

---

## Safety Improvements

### Emergency stop re-enabled
- **Before:** The entire safety filter in `emergency_stop.py` was bypassed by an early `return` statement.
- **After:** Safety logic is active by default. Added `enabled` ROS parameter (default: `True`) for controlled toggling without code changes.
- **Behavior:**
  - Distance ≤ 10cm: Emergency stop (forward motion blocked, reverse allowed)
  - Distance ≤ 30cm: Speed reduced proportionally (down to 30%)
  - Distance > 30cm: Full speed passthrough
  - Sensor timeout (>0.5s): Speed halved as precaution

### Auto-start timer leak fixed
- **Before:** `_auto_start_once` timer fired every 3 seconds indefinitely despite its name.
- **After:** Timer reference stored and cancelled after first invocation.

---

## Telegram Bot Changes

### Command mapping
| Telegram Command | ROS2 Mission Command | Description |
|-----------------|---------------------|-------------|
| `/scan` | `start_scan` | Start frontier exploration |
| `/stopscan` | `stop_scan` | Stop exploration |
| `/clean` | `start_clean` | Start map-based coverage (was random) |
| `/stopclean` | `stop_clean` | Stop cleaning |
| `/home` | `go_home` | Return to start position |
| `/reset` | `reset` | Reset to initial state |
| `/pause` | `pause` | Pause current operation |
| `/resume` | `resume` | Resume from pause |
| `/status` | (read-only) | Get current state |
| `/map` | (read-only) | Get map image |

### Removed commands
- `/coverage` — merged into `/clean` (which now always does map-based coverage)

---

## Feature: No-Go Zones (Banned Areas)

### Overview
Users can draw rectangular no-go zones on the live map. The robot will not enter these areas during scanning or cleaning.

### Frontend
- **Draw mode:** Toggle button ("🚫 Draw No-Go Zone") activates crosshair cursor. Click-and-drag draws a rectangle.
- **Zone rendering:** Red semi-transparent rectangles with dashed borders overlaid on the map canvas.
- **Zone list:** Below the map, showing all zones with delete buttons.
- **Clear all:** Button to remove all zones at once.
- **Touch support:** Works on mobile with touchstart/touchmove/touchend.
- **Map legend:** Added "No-Go" swatch.
- **i18n:** English and Hebrew translations.

### Backend
- **Storage:** No-go zones saved to `no_go_zones.json` (persists across restarts).
- **API:** `GET/POST /api/no_go_zones`, `DELETE /api/no_go_zones/<id>`, `POST /api/no_go_zones/clear`.
- **ROS publishing:** Zones published on `/no_go_zones` topic (String/JSON) every 10 seconds.
- **Map data:** Zones included in `map_update` WebSocket events for frontend rendering.

### Coverage Planner (adaptive_coverage.py)
- Subscribes to `/no_go_zones` topic.
- Before generating coverage path, no-go zone rectangles are converted to grid cells and marked as occupied in the obstacle mask (before inflation). This ensures the robot's path never enters the zone.

### Frontier Explorer (frontier_explorer.py)
- Subscribes to `/no_go_zones` topic.
- `select_best_frontier()` skips any frontier whose centroid falls inside a no-go zone.

### Files Changed
- `clean_bot_mission/clean_bot_mission/webapp/app.py` — Zone storage, API, ROS publisher
- `clean_bot_mission/clean_bot_mission/webapp/templates/index.html` — Drawing UI, rendering, CSS, i18n
- `clean_bot_mission/clean_bot_mission/adaptive_coverage.py` — Zone subscription, obstacle injection
- `clean_bot_mission/clean_bot_mission/frontier_explorer.py` — Zone subscription, frontier filtering
- `clean_bot_mission/test/test_webapp.py` — 13 new tests for zone API and HTML
