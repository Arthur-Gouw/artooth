import time
import serial
import threading
import signal
import sys
from dataclasses import dataclass
import pygame

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------
running   = True
auto_mode = False

ir_status = 0
ir_lock   = threading.Lock()

PING_INTERVAL = 5  # seconds

# ---------------------------------------------------------------------------
# Speed config
# ---------------------------------------------------------------------------
@dataclass
class SetConfig:
    min_speed:     int   = 5
    max_speed:     int   = 100
    default_speed: int   = 50
    gentle_factor: float = 0.6

setConfig     = SetConfig()
current_speed = setConfig.default_speed

# ---------------------------------------------------------------------------
# UART
# ---------------------------------------------------------------------------
ser = serial.Serial('/dev/ttyAMA2', 115200, timeout=1)

def sendSerialCommand(command_name, params):
    param_str   = ",".join(map(str, params))
    command_str = f"{command_name}:{param_str}\n"
    ser.write(command_str.encode())
    print(f"Sent: {command_str.strip()}")

# ---------------------------------------------------------------------------
# Movement functions
# ---------------------------------------------------------------------------
def moveForward(speed):
    sendSerialCommand('mv_fwd', [speed])

def moveReverse(speed):
    sendSerialCommand('mv_rev', [speed])

def moveLeft(speed):
    # STRAFE left (mecanum lateral) — manual A key only, NOT used in auto
    sendSerialCommand('mv_left', [speed])

def moveRight(speed):
    # STRAFE right (mecanum lateral) — manual D key only, NOT used in auto
    sendSerialCommand('mv_right', [speed])

def moveTurnLeft(speed):
    # TANK ROTATE CCW — both sides opposite — Q key + auto mode
    sendSerialCommand('mv_turnleft', [speed])

def moveTurnRight(speed):
    # TANK ROTATE CW — both sides opposite — E key + auto mode
    sendSerialCommand('mv_turnright', [speed])

def stopAll():
    sendSerialCommand('stop', [0])

# ---------------------------------------------------------------------------
# IR sensor helpers
#   Sensor output: 1 = black line detected, 0 = white / no line
#
#   bit0 = GPIO5  = RIGHT sensor
#   bit1 = GPIO6  = MIDDLE sensor
#   bit2 = GPIO7  = LEFT sensor
# ---------------------------------------------------------------------------
def get_ir_bits():
    with ir_lock:
        v = ir_status
    # Sensor output: 1 = black line detected, 0 = white / no line
    # No inversion needed
    return [(v >> i) & 1 for i in range(7)]

# ---------------------------------------------------------------------------
# FSM pattern sets
# 1 = sees black line, 0 = sees white / no line
# ---------------------------------------------------------------------------
STRAIGHT_PATTERNS     = {0b010, 0b101, 0b111}
GENTLE_RIGHT_PATTERNS = {0b011}   # mid+right see line → drifted left
HARD_RIGHT_PATTERNS   = {0b001}   # only right sees line → drifted far left
GENTLE_LEFT_PATTERNS  = {0b110}   # mid+left see line → drifted right
HARD_LEFT_PATTERNS    = {0b100}   # only left sees line → drifted far right
STOP_PATTERNS         = {0b000}   # no sensor sees line → lost

# ---------------------------------------------------------------------------
# Line following FSM
# ALL corrections use moveTurnLeft / moveTurnRight (tank rotation)
# NEVER uses moveLeft / moveRight (those are mecanum strafe)
# ---------------------------------------------------------------------------
def line_follow_step():
    bits  = get_ir_bits()
    right = bits[0]   # GPIO5
    mid   = bits[1]   # GPIO6
    left  = bits[2]   # GPIO7

    pattern = (left << 2) | (mid << 1) | right

    speed = max(setConfig.min_speed,
                min(setConfig.max_speed, current_speed))

    gentle_speed = max(setConfig.min_speed,
                       int(speed * setConfig.gentle_factor))

    print(f"IR L={left} M={mid} R={right} | pattern={pattern:03b} | spd={speed}")

    # ---- FSM ----

    if pattern in STRAIGHT_PATTERNS:
        # Line centred → go forward
        moveForward(speed)

    elif pattern in GENTLE_RIGHT_PATTERNS:
        # Mid + right see line → drifted slightly left
        # Tank rotate CW (left) at gentle speed
        moveTurnLeft(gentle_speed)

    elif pattern in HARD_RIGHT_PATTERNS:
        # Only right sees line → drifted far left
        # Tank rotate CW (left) at full speed
        moveTurnLeft(speed)

    elif pattern in GENTLE_LEFT_PATTERNS:
        # Mid + left see line → drifted slightly right
        # Tank rotate CCW (right) at gentle speed
        moveTurnRight(gentle_speed)

    elif pattern in HARD_LEFT_PATTERNS:
        # Only left sees line → drifted far right
        # Tank rotate CCW (right) at full speed
        moveTurnRight(speed)

    elif pattern in STOP_PATTERNS:
        # No sensor sees line → lost → stop
        stopAll()

    else:
        stopAll()

    # ---- FSM END ----

# ---------------------------------------------------------------------------
# UART receive + ping thread
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
                    value = int(value_str) & 0x7F
                    with ir_lock:
                        ir_status = value
                    print(f"IR raw={value:07b}")
                except ValueError:
                    print(f"Bad IR_STATUS: {text}")
            else:
                print(f"UART: {text}")

        now = time.monotonic()
        if now - last_ping >= PING_INTERVAL:
            ser.write(b"hello from raspberry pi\n")
            last_ping = now

# ---------------------------------------------------------------------------
# Ctrl+C handler
# ---------------------------------------------------------------------------
def handle_sigint(sig, frame):
    global running
    running = False
    stopAll()
    sys.exit(0)

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    signal.signal(signal.SIGINT, handle_sigint)

    t_uart = threading.Thread(target=uart_thread, daemon=True)
    t_uart.start()

    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Mecanum Line Follower")
    font = pygame.font.SysFont(None, 28)

    pressed = {'w': False, 's': False, 'a': False,
               'd': False, 'q': False, 'e': False}

    print("M=toggle auto | W/S/A/D=manual | Q/E=turn | Space=stop | 1/2=speed | Esc=quit")
    print(f"Speed: {current_speed}")

    def update_movement():
        if auto_mode:
            return
        speed = max(setConfig.min_speed, min(setConfig.max_speed, current_speed))
        if   pressed['w']: moveForward(speed)
        elif pressed['s']: moveReverse(speed)
        elif pressed['q']: moveTurnLeft(speed)     # Q = tank rotate left
        elif pressed['e']: moveTurnRight(speed)    # E = tank rotate right
        elif pressed['a']: moveLeft(speed)         # A = strafe left (manual only)
        elif pressed['d']: moveRight(speed)        # D = strafe right (manual only)
        else:              stopAll()

    clock = pygame.time.Clock()

    while running:
        # ---- HUD ----
        screen.fill((30, 30, 30))
        mode_color = (0, 220, 100) if auto_mode else (220, 180, 0)
        mode_label = "AUTO  (line following)" if auto_mode else "MANUAL  (WASD+QE)"
        bits = get_ir_bits()
        pat  = (bits[2] << 2) | (bits[1] << 1) | bits[0]

        screen.blit(font.render(f"MODE: {mode_label}",                         True, mode_color),    (20, 20))
        screen.blit(font.render(f"Speed: {current_speed}",                     True, (200,200,200)), (20, 60))
        screen.blit(font.render(f"IR   L={bits[2]}  M={bits[1]}  R={bits[0]}", True, (100,180,255)), (20, 100))
        screen.blit(font.render(f"Pattern: {pat:03b}  ({pat})",                True, (100,180,255)), (20, 135))
        screen.blit(font.render("M=auto  1/2=speed  Esc=quit",                 True, (120,120,120)), (20, 240))
        pygame.display.flip()

        # ---- Events ----
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_m:
                    auto_mode = not auto_mode
                    pressed   = {k: False for k in pressed}
                    stopAll()
                    print("AUTO ON" if auto_mode else "MANUAL")

                elif event.key == pygame.K_w:      pressed['w'] = True;  update_movement()
                elif event.key == pygame.K_s:      pressed['s'] = True;  update_movement()
                elif event.key == pygame.K_a:      pressed['a'] = True;  update_movement()
                elif event.key == pygame.K_d:      pressed['d'] = True;  update_movement()
                elif event.key == pygame.K_q:      pressed['q'] = True;  update_movement()
                elif event.key == pygame.K_e:      pressed['e'] = True;  update_movement()
                elif event.key == pygame.K_SPACE:  pressed = {k: False for k in pressed}; stopAll()
                elif event.key == pygame.K_ESCAPE: running = False

                elif event.key == pygame.K_1:
                    current_speed = max(setConfig.min_speed, current_speed - 5)
                    print(f"Speed -> {current_speed}");  update_movement()
                elif event.key == pygame.K_2:
                    current_speed = min(setConfig.max_speed, current_speed + 5)
                    print(f"Speed -> {current_speed}");  update_movement()

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
