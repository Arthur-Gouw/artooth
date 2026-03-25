import time
import serial
import threading
import signal
import sys
import pygame

# ===========================================================================
# TUNABLE PARAMETERS — Algorithmic Navigation
# ===========================================================================

# -- Sensor Configuration --
REVERSE_SENSOR_ORDER = False 
LOST_DETECTION_DELAY = 0.5      # Seconds of no-line before declaring LOST

# -- Algorithm Weights (W, NW, N, NE, E) --
TURN_STRENGTHS = [-7, -4.5, 0.0, 4.5, 7]
MOVE_STRENGTHS = [3.8, 4.2, 5.0, 4.2, 3.8]

# -- Speed & Physics Limits --
MIN_SPEED       = 10       # lowest usable motor speed
MAX_SPEED       = 150      # highest motor speed
DEFAULT_SPEED   = 35       # Base multiplier setting (Algorithm outputs 0-5. 5 * 10 = 50 motor speed)

ACCEL = 0.10  # Speed gained per tick
DECEL = 0.07  # Speed lost per tick
MAX_TURN_STRENGTH = 9.0 

# -- Recovery & Junction Parameters --
RECOVERY_REVERSE_SPEED = 20
RECOVERY_PIVOT_SPEED   = 80     # Front wheel speed for sidepivot recovery
RECOVERY_PIVOT_REAR_PERCENT = 15  # Rear wheels as % of front during pivot recovery
ENDPOINT_TARGET_SPEED  = 2.0   # Out of 5.0

# -- Pseudo-distance threshold (speed_units × seconds, no encoder) --
# Represents the estimated "distance travelled" while reversing before
# escalating to the pivot state. Tune to match your physical track width.
REVERSE_PSEUDO_DIST_MAX = 1.8   # How far to reverse before switching to pivot
ENDPOINT_PSEUDO_DIST_MAX = 5
LOST_PSEUDO_DIST_MAX = 1.5

# -- UART --
SERIAL_PORT     = '/dev/ttyAMA2'
BAUD_RATE       = 115200
PING_INTERVAL   = 5

# ===========================================================================
# Globals & State Machine
# ===========================================================================
running   = True
auto_mode = False

ir_status = 0
ir_lock   = threading.Lock()

current_speed = DEFAULT_SPEED   # Acts as the master scalar for the 0-5 algorithmic speeds

# FSM States
STATE_FOLLOWING    = 0
STATE_ENDPOINT     = 1
STATE_LOST_REVERSE = 2
STATE_LOST_PIVOT   = 3
STATE_STOPPED      = 4

auto_state     = STATE_FOLLOWING
internal_speed = 0.0  # Tracks the 0.0 to 5.0 algorithmic speed
last_turn_var  = 0.0  # Memory for which way we were turning before getting lost
turn_var       = 0.0

# -- Pseudo-distance accumulator --
# Incremented each tick by (estimated_speed_fraction × dt).
# Resets on each state entry so it measures distance within the current state.
pseudo_dist      = 0.0
last_step_time   = 0.0   # Wall-clock time of the previous line_follow_step call

# -- Lost detection debounce --
lost_start_time  = 0.0   # Time when active_count first dropped to 0

STATE_NAMES = {
    STATE_FOLLOWING:    "FOLLOW",
    STATE_ENDPOINT:     "ENDPOINT",
    STATE_LOST_REVERSE: "LOST_REV",
    STATE_LOST_PIVOT:   "LOST_PIVOT",
    STATE_STOPPED:      "STOPPED",
}

# ---------------------------------------------------------------------------
# UART & Movement Functions (Kept identical for hardware compatibility)
# ---------------------------------------------------------------------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def sendSerialCommand(command_name, params):
    param_str   = ",".join(map(str, params))
    command_str = f"{command_name}:{param_str}\n"
    ser.write(command_str.encode())

def moveForward(speed):  sendSerialCommand('mv_fwd',       [speed])
def moveReverse(speed):  sendSerialCommand('mv_rev',       [speed])
def moveLeft(speed):     sendSerialCommand('mv_left',      [speed])
def moveRight(speed):    sendSerialCommand('mv_right',     [speed])
def moveTurnLeft(speed): sendSerialCommand('mv_turnleft',  [speed])
def moveTurnRight(speed):sendSerialCommand('mv_turnright', [speed])
def stopAll():           sendSerialCommand('stop',         [0])
def moveCurve(left_speed, right_speed): sendSerialCommand('mv_curve', [left_speed, right_speed])

def moveSidePivot(front_speed, rear_percent, direction):
    """
    Wraps the ESP32 mv_sidepivot command.
      front_speed  : PWM speed for the front wheels (int)
      rear_percent : rear wheel contribution as % of front (0-100)
      direction    : 1 = pivot right, -1 = pivot left
    """
    sendSerialCommand('mv_sidepivot', [front_speed, rear_percent, direction])

# ---------------------------------------------------------------------------
# IR Sensor Helpers
#   Sensor output: 1 = black line detected, 0 = white / no line
#
#         [N]  North (center front)
#        /   \
#     [NW]   [NE]
#     /         \
#   [W]         [E]
#
#   bit0 = GPIO5  = W   (West / far left)
#   bit1 = GPIO6  = NW  (Northwest)
#   bit2 = GPIO7  = N   (North / center front)
#   bit3 = GPIO15 = NE  (Northeast)
#   bit4 = GPIO45 = E   (East / far right)
# ---------------------------------------------------------------------------
IDX_W, IDX_NW, IDX_N, IDX_NE, IDX_E = 0, 1, 2, 3, 4

def get_ir_bits():
    with ir_lock:
        v = ir_status
    if REVERSE_SENSOR_ORDER:
        return [(v >> (4 - i)) & 1 for i in range(5)]
    return [(v >> i) & 1 for i in range(5)]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _enter_state(new_state):
    """Transition to a new FSM state, resetting the pseudo-distance accumulator."""
    global auto_state, pseudo_dist
    auto_state  = new_state
    pseudo_dist = 0.0

def _accumulate_pseudo_dist(dt, speed_fraction):
    """
    Integrate estimated speed over time to approximate distance travelled.
    speed_fraction is 0.0–1.0 (1.0 = full speed).  dt is in seconds.
    Returns the updated pseudo_dist value.
    """
    global pseudo_dist
    pseudo_dist += abs(speed_fraction) * dt
    return pseudo_dist

# ---------------------------------------------------------------------------
# ALGORITHMIC LINE FOLLOWING FSM
# ---------------------------------------------------------------------------
def line_follow_step():
    global auto_state, internal_speed, last_turn_var, current_speed, turn_var
    global last_step_time, pseudo_dist

    # --- Per-tick timing (used for pseudo-distance integration) ---
    now = time.monotonic()
    dt  = now - last_step_time if last_step_time > 0 else 0.0
    last_step_time = now

    bits         = get_ir_bits()
    active_count = sum(bits)
    pattern      = (bits[IDX_E] << 4) | (bits[IDX_NE] << 3) | (bits[IDX_N] << 2) | (bits[IDX_NW] << 1) | bits[IDX_W]

    # Speed multiplier (maps 0.0-5.0 scale to actual motor speed)
    multiplier = current_speed / 5.0

    # -----------------------------------------------------------------------
    # 1. STATE: STOPPED / BRAKING
    # -----------------------------------------------------------------------
    if auto_state == STATE_STOPPED:
        if internal_speed > 0:
            internal_speed = max(0.0, internal_speed - DECEL * 5)
            spd = int(round(internal_speed * multiplier))
            moveCurve(spd, spd)
        else:
            stopAll()
        return

    # -----------------------------------------------------------------------
    # 2. STATE: LOST — REVERSING TO FIND LINE
    #    Accumulates pseudo-distance from RECOVERY_REVERSE_SPEED.
    #    Exits to PIVOT once the distance threshold is exceeded.
    #    If the line reappears before the threshold, also exits to PIVOT so
    #    the pivot state can properly centre before resuming.
    # -----------------------------------------------------------------------
    if auto_state == STATE_LOST_REVERSE:
        rev_frac = RECOVERY_REVERSE_SPEED / MAX_SPEED
        _accumulate_pseudo_dist(dt, rev_frac)

        if pseudo_dist >= REVERSE_PSEUDO_DIST_MAX:
            print(f">>> LOST_REV → PIVOT (dist={pseudo_dist:.2f})")
            _enter_state(STATE_LOST_PIVOT)
        elif active_count > 0:
            print(f">>> LOST_REV → PIVOT (line reappeared early, dist={pseudo_dist:.2f})")
            _enter_state(STATE_LOST_PIVOT)
        else:
            moveCurve(-RECOVERY_REVERSE_SPEED, -RECOVERY_REVERSE_SPEED)
        return

    # -----------------------------------------------------------------------
    # 3. STATE: LOST — PIVOTING TO RECENTER
    #    Uses sidepivot (front wheels oppose, rear at 15%) based on the sign
    #    of last_turn_var.  Exits to FOLLOWING purely when the sensor reading
    #    shows the line is no longer at an extreme — |turn_var| < 5.0 means
    #    the line is somewhere between the outermost sensors (W and E).
    # -----------------------------------------------------------------------
    if auto_state == STATE_LOST_PIVOT:
        if active_count > 0:
            curr_turn_var = sum(b * t for b, t in zip(bits, TURN_STRENGTHS)) / active_count
            if abs(curr_turn_var) < 5.0:
                print(f">>> LOST_PIVOT → FOLLOW (turn_var={curr_turn_var:.2f})")
                _enter_state(STATE_FOLLOWING)
                return

        # Pivot direction from sign of last recorded turn only — strength ignored.
        # last_turn_var > 0 → line was to the right → pivot right (direction=1)
        # last_turn_var ≤ 0 → line was to the left  → pivot left  (direction=-1)
        direction = 1 if last_turn_var > 0 else -1
        moveSidePivot(RECOVERY_PIVOT_SPEED, RECOVERY_PIVOT_REAR_PERCENT, direction)
        return

    # -----------------------------------------------------------------------
    # 4. STATE: ENDPOINT HANDLING
    # -----------------------------------------------------------------------
    if auto_state == STATE_ENDPOINT:
        target_max_speed = ENDPOINT_TARGET_SPEED
        turn_var = 0.0

        # Accumulate distance driven while in the ENDPOINT state
        speed_frac = internal_speed / 5.0
        _accumulate_pseudo_dist(dt, speed_frac)

        # Trigger hard stop once the distance threshold is covered
        if pseudo_dist >= ENDPOINT_PSEUDO_DIST_MAX:
            _enter_state(STATE_STOPPED)
            print(f">>> ENDPOINT DIST MET ({pseudo_dist:.2f}) -> HARD STOP")
            return

        if 0 < active_count < 5 and pattern not in [0b11011, 0b10001]:
            _enter_state(STATE_FOLLOWING)
            print(">>> ENDPOINT CLEARED")
            return

        if active_count == 0:
            _enter_state(STATE_LOST_REVERSE)
            return

    # -----------------------------------------------------------------------
    # 5. STATE: NORMAL FOLLOWING
    # -----------------------------------------------------------------------
    if auto_state == STATE_FOLLOWING:
        if active_count == 0:
            # Accumulate distance while no line is detected
            speed_frac = internal_speed / 5.0
            _accumulate_pseudo_dist(dt, speed_frac)
            if pseudo_dist >= LOST_PSEUDO_DIST_MAX:
                _enter_state(STATE_LOST_REVERSE)
                internal_speed = 0.0
                print(f">>> LOST LINE -> RECOVERY after dist {pseudo_dist:.2f}")
            else:
                # Still within the distance threshold — coast with last known steering
                internal_speed = max(0.0, internal_speed - DECEL)
                if last_turn_var > 0:
                    moveCurve(int(internal_speed * multiplier), -int(internal_speed * multiplier))
                else:
                    moveCurve(-int(internal_speed * multiplier), int(internal_speed * multiplier))
            return
        else:
            # Line visible — reset the pseudo_dist accumulator so it only tracks consecutive lost distance
            pseudo_dist = 0.0

        if active_count == 5:
            _enter_state(STATE_ENDPOINT)
            print(">>> ALL SENSORS ON: ENTERING ENDPOINT")
            target_max_speed = ENDPOINT_TARGET_SPEED
            turn_var = 0.0
        else:
            turn_var = sum(b * t for b, t in zip(bits, TURN_STRENGTHS)) / active_count
            last_turn_var = turn_var

            if turn_var == 0.0:
                target_max_speed = 5.0
            else:
                target_max_speed = sum(b * m for b, m in zip(bits, MOVE_STRENGTHS)) / active_count

    # -----------------------------------------------------------------------
    # Shared Physics & Steering  (Following & Endpoint)
    # -----------------------------------------------------------------------

    # Smooth acceleration / deceleration
    if internal_speed < target_max_speed:
        internal_speed = min(target_max_speed, internal_speed + ACCEL)
    elif internal_speed > target_max_speed:
        internal_speed = max(target_max_speed, internal_speed - DECEL)

    # Differential steering
    turn_ratio = abs(turn_var) / MAX_TURN_STRENGTH * 2
    delta_v    = internal_speed * turn_ratio

    if turn_var > 0:       # Turning Right
        left_algo  = (internal_speed + delta_v) * 1.3
        right_algo = internal_speed - delta_v
    elif turn_var < 0:     # Turning Left
        left_algo  = internal_speed - delta_v
        right_algo = (internal_speed + delta_v) * 1.3
    else:                  # Straight
        left_algo = right_algo = internal_speed

    left_motor  = max(-MAX_SPEED, min(MAX_SPEED, int(round(left_algo  * multiplier))))
    right_motor = max(-MAX_SPEED, min(MAX_SPEED, int(round(right_algo * multiplier))))

    moveCurve(left_motor, right_motor)


# ---------------------------------------------------------------------------
# UART Thread & Pygame Boilerplate
# ---------------------------------------------------------------------------
def uart_thread():
    global running, ir_status
    last_ping = time.monotonic()
    while running:
        line = ser.readline()
        text = line.decode(errors='ignore').strip()
        if text:
            if text.startswith("IR_STATUS:"):
                try:
                    _, value_str = text.split(":", 1)
                    with ir_lock:
                        ir_status = int(value_str) & 0x1F
                except ValueError:
                    pass

        now = time.monotonic()
        if now - last_ping >= PING_INTERVAL:
            ser.write(b"hello from pi\n")
            last_ping = now

def handle_sigint(sig, frame):
    global running
    running = False
    stopAll()
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, handle_sigint)

    t_uart = threading.Thread(target=uart_thread, daemon=True)
    t_uart.start()

    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Algorithmic Line Follower")
    font = pygame.font.SysFont(None, 28)

    pressed = {'w': False, 's': False, 'a': False, 'd': False, 'q': False, 'e': False}

    def update_movement():
        if auto_mode: return
        if   pressed['w']: moveForward(current_speed)
        elif pressed['s']: moveReverse(current_speed)
        elif pressed['q']: moveTurnLeft(current_speed)
        elif pressed['e']: moveTurnRight(current_speed)
        elif pressed['a']: moveLeft(current_speed)
        elif pressed['d']: moveRight(current_speed)
        else:              stopAll()

    clock = pygame.time.Clock()

    while running:
        screen.fill((30, 30, 30))
        mode_color = (0, 220, 100) if auto_mode else (220, 180, 0)
        mode_label = "AUTO (Algorithmic)" if auto_mode else "MANUAL (WASD+QE)"

        bits = get_ir_bits()
        active = sum(bits)
        turn_display = sum(b * t for b, t in zip(bits, TURN_STRENGTHS)) / active if active > 0 else 0.0

        screen.blit(font.render(f"MODE: {mode_label}",                             True, mode_color),      (20, 20))
        screen.blit(font.render(f"Max Speed Scalar: {current_speed}",              True, (200,200,200)),   (20, 60))
        screen.blit(font.render(f"Internal Algo Spd: {internal_speed:.2f}",        True, (200,200,200)),   (20, 90))
        screen.blit(font.render(f"Turn Strength: {(turn_var / MAX_TURN_STRENGTH * 2) if turn_var != 0 else 0:.2f}", True, (200,200,200)), (20, 120))
        screen.blit(font.render(f"W={bits[0]} NW={bits[1]} N={bits[2]} NE={bits[3]} E={bits[4]}", True, (100,180,255)), (20, 160))
        screen.blit(font.render(f"Calc Turn Var: {turn_display:.2f}",              True, (100,180,255)),   (20, 190))
        screen.blit(font.render(f"State: {STATE_NAMES.get(auto_state, '?')}",      True, (180,220,180)),   (20, 230))
        screen.blit(font.render(f"Pseudo-dist: {pseudo_dist:.2f}",                 True, (180,180,100)),   (20, 260))

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_m:
                    auto_mode = not auto_mode
                    pressed   = {k: False for k in pressed}
                    stopAll()
                    if auto_mode:
                        _enter_state(STATE_FOLLOWING)
                        internal_speed  = 0.0
                        last_step_time  = 0.0
                elif event.key == pygame.K_w:      pressed['w'] = True;  update_movement()
                elif event.key == pygame.K_s:      pressed['s'] = True;  update_movement()
                elif event.key == pygame.K_a:      pressed['a'] = True;  update_movement()
                elif event.key == pygame.K_d:      pressed['d'] = True;  update_movement()
                elif event.key == pygame.K_q:      pressed['q'] = True;  update_movement()
                elif event.key == pygame.K_e:      pressed['e'] = True;  update_movement()
                elif event.key == pygame.K_SPACE:  pressed = {k: False for k in pressed}; stopAll()
                elif event.key == pygame.K_ESCAPE: running = False
                elif event.key == pygame.K_1:
                    current_speed = max(MIN_SPEED, current_speed - 5); update_movement()
                elif event.key == pygame.K_2:
                    current_speed = min(MAX_SPEED, current_speed + 5); update_movement()
            elif event.type == pygame.KEYUP:
                if   event.key == pygame.K_w: pressed['w'] = False; update_movement()
                elif event.key == pygame.K_s: pressed['s'] = False; update_movement()
                elif event.key == pygame.K_a: pressed['a'] = False; update_movement()
                elif event.key == pygame.K_d: pressed['d'] = False; update_movement()
                elif event.key == pygame.K_q: pressed['q'] = False; update_movement()
                elif event.key == pygame.K_e: pressed['e'] = False; update_movement()

        if auto_mode:
            line_follow_step()

        clock.tick(60)

    stopAll()
    pygame.quit()
