"""Forward-scan sonar console, drawn with pygame.

The matplotlib front end in simulator_app.py showed the same panels but spent
about 50 ms a frame inside its image compositing path, against 5 to 10 ms of
actual physics. This draws the same console directly: each waterfall is a numpy
RGB array turned into a surface, scaled once and blitted, so the display cost is
a few milliseconds and the frame rate is set by the simulator rather than by the
plotting library.

    .venv/bin/python python/console.py

Keys: space runs and pauses the platform, r resets the run, q or escape quits.
"""

import sys
import time

import numpy as np
import pygame

sys.path.insert(0, ".")
sys.path.insert(0, "python")
import display as disp
import scene as sc
import sonar

WIDTH, HEIGHT = 1560, 980
DRAFT_SUBRAYS, FULL_SUBRAYS = 192, 1280
N_AZIMUTH, N_RANGE, MAX_RANGE = 256, 512, 10.0
FOV_DEG = 30.0
HISTORY = 200
SWEEP_S = 0.35
FPS_CAP = 60      # 0 removes the cap, which is how the frame budget is measured

INK = (0, 0, 0)
PHOSPHOR = (0, 255, 64)
TRACE = (255, 255, 0)
MARK = (255, 43, 32)
RELATIVE = (79, 123, 255)
FRAME = (138, 20, 16)
DIM = (122, 138, 128)
PANEL = (13, 26, 13)

POWER = pygame.Rect(60, 34, 1440, 66)
COMPASS_Y, RELATIVE_Y = 106, 126
MAIN = pygame.Rect(60, 150, 1440, 320)
ASCAN = pygame.Rect(60, 486, 706, 56)
OPTICAL = pygame.Rect(794, 486, 706, 56)
BTR = pygame.Rect(60, 550, 706, 196)
RTR = pygame.Rect(794, 550, 706, 196)


class Slider:
    """A labelled bar the mouse can drag. Deliberately minimal: the console needs
    a value and a hit test, not a widget framework."""

    def __init__(self, rect, label, low, high, value, fmt="{:.0f}"):
        self.rect = rect
        self.label = label
        self.low, self.high, self.value = low, high, value
        self.fmt = fmt
        self.held = False

    def fraction(self):
        return (self.value - self.low) / (self.high - self.low)

    def hit(self, position):
        return self.rect.inflate(0, 14).collidepoint(position)

    def drag(self, position):
        fraction = np.clip((position[0] - self.rect.x) / self.rect.width, 0.0, 1.0)
        self.value = self.low + fraction * (self.high - self.low)

    def draw(self, surface, font):
        text = font.render(self.label, True, MARK)
        surface.blit(text, (self.rect.x - text.get_width() - 10,
                            self.rect.centery - text.get_height() // 2))
        pygame.draw.rect(surface, (58, 58, 58), self.rect)
        filled = pygame.Rect(self.rect.x, self.rect.y,
                             int(self.rect.width * self.fraction()), self.rect.height)
        pygame.draw.rect(surface, PHOSPHOR, filled)
        knob = (self.rect.x + int(self.rect.width * self.fraction()), self.rect.centery)
        pygame.draw.circle(surface, (245, 245, 245), knob, 7)
        value = font.render(self.fmt.format(self.value), True, TRACE)
        surface.blit(value, (self.rect.right + 12, self.rect.centery - value.get_height() // 2))


class Toggle:
    def __init__(self, rect, label, state):
        self.rect = rect
        self.label = label
        self.state = state

    def hit(self, position):
        return self.rect.inflate(190, 8).collidepoint(position)

    def draw(self, surface, font):
        pygame.draw.rect(surface, PHOSPHOR, self.rect, 1)
        if self.state:
            pygame.draw.rect(surface, PHOSPHOR, self.rect.inflate(-6, -6))
        text = font.render(self.label, True, PHOSPHOR)
        surface.blit(text, (self.rect.right + 12, self.rect.centery - text.get_height() // 2))


def surface_from(pixels):
    """numpy (rows, columns, 3) uint8 to a pygame surface, without a copy per row."""
    height, width = pixels.shape[:2]
    return pygame.image.frombuffer(np.ascontiguousarray(pixels).tobytes(),
                                   (width, height), "RGB")


def blit_panel(screen, pixels, rect):
    screen.blit(pygame.transform.scale(surface_from(pixels), rect.size), rect.topleft)
    pygame.draw.rect(screen, FRAME, rect, 1)


def label_left(screen, font, rect, text, y):
    surface = font.render(text, True, MARK)
    screen.blit(surface, (rect.x - surface.get_width() - 6, y - surface.get_height() // 2))


def label_below(screen, font, rect, text, x):
    surface = font.render(text, True, MARK)
    screen.blit(surface, (x - surface.get_width() // 2, rect.bottom + 3))


def polyline(screen, rect, values, low, high, colour, width=1):
    """A trace scaled into a panel, with the vertical axis running upward."""
    span = max(high - low, 1e-12)
    scaled = np.clip((np.asarray(values) - low) / span, 0.0, 1.0)
    columns = rect.x + np.linspace(0, rect.width - 1, len(scaled))
    rows = rect.bottom - 1 - scaled * (rect.height - 2)
    pygame.draw.lines(screen, colour, False, list(zip(columns, rows)), width)


def main():
    pygame.init()
    pygame.display.set_caption("Forward-scan sonar console")
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    font = pygame.font.SysFont("Menlo,Monaco,Courier", 13)
    small = pygame.font.SysFont("Menlo,Monaco,Courier", 12)
    clock = pygame.time.Clock()

    sliders = [
        Slider(pygame.Rect(150, 800, 240, 10), "tilt", 0.0, 35.0, 16.0),
        Slider(pygame.Rect(150, 828, 240, 10), "roll", 0.0, 90.0, 0.0),
        Slider(pygame.Rect(150, 856, 240, 10), "beamwidth", 4.0, 30.0, 12.0),
        Slider(pygame.Rect(150, 884, 240, 10), "freq kHz", 100.0, 2000.0, 900.0),
        Slider(pygame.Rect(150, 912, 240, 10), "range m", 2.0, 8.0, 4.5, "{:.2f}"),
        Slider(pygame.Rect(150, 940, 240, 10), "heading", 0.0, 359.0, 348.0),
        Slider(pygame.Rect(620, 800, 240, 10), "wave mm", 0.0, 2.0, 0.0, "{:.2f}"),
        Slider(pygame.Rect(620, 828, 240, 10), "turbidity", 0.02, 1.2, 0.10, "{:.2f}"),
        Slider(pygame.Rect(620, 856, 240, 10), "texture", 0.0, 0.9, 0.45, "{:.2f}"),
        Slider(pygame.Rect(620, 884, 240, 10), "surge m/s", 0.0, 2.0, 0.6, "{:.2f}"),
        Slider(pygame.Rect(620, 912, 240, 10), "yaw deg/s", -40.0, 40.0, 0.0),
        Slider(pygame.Rect(620, 940, 240, 10), "gain dB", -20.0, 20.0, 0.0),
    ]
    by_name = {slider.label: slider for slider in sliders}
    toggles = [Toggle(pygame.Rect(1080, 796, 16, 16), "multipath", True),
               Toggle(pygame.Rect(1080, 824, 16, 16), "speckle", True),
               Toggle(pygame.Rect(1080, 852, 16, 16), "cylinder", False),
               Toggle(pygame.Rect(1080, 880, 16, 16), "run", True),
               Toggle(pygame.Rect(1080, 908, 16, 16), "detections", True)]
    toggle_by_name = {toggle.label: toggle for toggle in toggles}

    azimuth_axis = -0.5 * FOV_DEG + FOV_DEG * (np.arange(N_AZIMUTH) + 0.5) / N_AZIMUTH
    range_axis = MAX_RANGE / N_RANGE * (np.arange(N_RANGE) + 0.5)
    ticks = np.linspace(-0.5 * FOV_DEG, 0.5 * FOV_DEG, 11)

    bearing_history = np.full((HISTORY, N_AZIMUTH), disp.BEARING_FLOOR_DB)
    range_history = np.full((HISTORY, N_RANGE), disp.FLOOR_DB)

    advance, frame, held, core_ms, running = 0.0, 0, None, 0.0, True
    while running:
        subrays = DRAFT_SUBRAYS if held is not None else FULL_SUBRAYS
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_SPACE:
                    toggle_by_name["run"].state = not toggle_by_name["run"].state
                elif event.key == pygame.K_r:
                    advance = 0.0
            elif event.type == pygame.MOUSEBUTTONDOWN:
                for toggle in toggles:
                    if toggle.hit(event.pos):
                        toggle.state = not toggle.state
                for slider in sliders:
                    if slider.hit(event.pos):
                        held = slider
                        slider.drag(event.pos)
            elif event.type == pygame.MOUSEBUTTONUP:
                held = None
            elif event.type == pygame.MOUSEMOTION and held is not None:
                held.drag(event.pos)

        if toggle_by_name["run"].state:
            advance += by_name["surge m/s"].value / 30.0
            if advance > by_name["range m"].value - 0.8:
                advance = 0.0

        started = time.perf_counter()
        frame += 1
        tilt = by_name["tilt"].value
        objects, centre = disp.build_scene(tilt, by_name["range m"].value,
                                           by_name["texture"].value,
                                           toggle_by_name["cylinder"].state)
        axes = sc.tilted_axes(tilt, by_name["roll"].value)
        position = np.array([0.0, 0.0, disp.SONAR_Z]) + advance * axes[:, 1]
        yaw_rate = by_name["yaw deg/s"].value

        sim = sonar.SonarSimulator(
            frequency_hz=by_name["freq kHz"].value * 1e3, num_azimuth_bins=N_AZIMUTH,
            num_range_bins=N_RANGE, horizontal_fov_deg=FOV_DEG,
            vertical_beamwidth_deg=by_name["beamwidth"].value, max_range_m=MAX_RANGE,
            num_elevation_subrays=subrays,
            multipath_enabled=toggle_by_name["multipath"].state, surface_z=0.0,
            surface_rms_height_m=by_name["wave mm"].value * 1e-3,
            platform_yaw_rate_dps=yaw_rate, sweep_duration_s=SWEEP_S if yaw_rate else 0.0)
        image = sim.render(objects, position=position, axes=axes,
                           speckle=toggle_by_name["speckle"].state, seed=frame)

        camera = sonar.OpticalCamera(width=160, height=120, focal_px=125.0,
                                     position=position + np.array([0.0, 0.0, disp.CAMERA_RISE]),
                                     axes=axes, attenuation_per_m=by_name["turbidity"].value,
                                     light_intensity=150.0)
        optical = camera.render(objects)
        core_ms = 1000 * (time.perf_counter() - started)

        gain = by_name["gain dB"].value
        decibels = sc.to_decibels(image, floor_db=disp.FLOOR_DB) + gain
        bearing_db = sc.to_decibels(image.sum(axis=1), floor_db=disp.BEARING_FLOOR_DB)
        centre_db = sc.to_decibels(image[N_AZIMUTH // 2], floor_db=disp.FLOOR_DB,
                                   reference=image.max())

        bearing_history[1:] = bearing_history[:-1]
        bearing_history[0] = bearing_db
        range_history[1:] = range_history[:-1]
        range_history[0] = centre_db

        screen.fill(INK)

        # Bearing power, with the reference line the console draws across the top.
        pygame.draw.rect(screen, FRAME, POWER, 1)
        pygame.draw.rect(screen, TRACE, pygame.Rect(POWER.x + 6, POWER.y + 10, 24, 44))
        for offset in range(0, POWER.width, 12):
            screen.set_at((POWER.x + offset, POWER.y + 6), (255, 255, 255))
        polyline(screen, POWER, 1.0 + bearing_db / -disp.BEARING_FLOOR_DB, 0.0, 1.08, TRACE)
        cursor = POWER.x + POWER.width // 2
        pygame.draw.line(screen, TRACE, (cursor, POWER.y), (cursor, POWER.bottom), 2)

        # Two scales: true bearing in red over relative bearing in blue.
        heading = by_name["heading"].value
        for tick in ticks:
            x = MAIN.x + int(MAIN.width * (tick + 0.5 * FOV_DEG) / FOV_DEG)
            compass = small.render(f"{(heading + tick) % 360:03.0f}", True, MARK)
            screen.blit(compass, (x - compass.get_width() // 2, COMPASS_Y))
            offset = small.render(f"{tick:+.0f}", True, RELATIVE)
            screen.blit(offset, (x - offset.get_width() // 2, RELATIVE_Y))

        # Main display: bearing across, range downward.
        blit_panel(screen, disp.to_pixels(decibels.T), MAIN)
        for metres in range(0, int(MAX_RANGE) + 1, 2):
            y = MAIN.y + int(MAIN.height * metres / MAX_RANGE)
            if metres:
                pygame.draw.line(screen, (40, 12, 10), (MAIN.x + 1, y), (MAIN.right - 1, y))
            label_left(screen, small, MAIN, f"{metres:2d}", y)
        if toggle_by_name["detections"].state:
            peaks = np.argmax(decibels, axis=1)
            lit = decibels[np.arange(N_AZIMUTH), peaks] > disp.FLOOR_DB + 22.0
            columns = MAIN.x + (np.nonzero(lit)[0] * MAIN.width) // N_AZIMUTH
            rows = MAIN.y + (peaks[lit] * MAIN.height) // N_RANGE
            for column, row in zip(columns, rows):
                pygame.draw.rect(screen, MARK, (column, row, 2, 2))

        pygame.draw.rect(screen, FRAME, ASCAN, 1)
        polyline(screen, ASCAN, centre_db, disp.FLOOR_DB, 4.0, TRACE)
        screen.blit(small.render("A-SCAN", True, MARK), (ASCAN.x + 6, ASCAN.y + 4))

        pygame.draw.rect(screen, FRAME, OPTICAL, 1)
        row = optical[optical.shape[0] // 2]
        polyline(screen, OPTICAL, row / max(row.max(), 1e-9), 0.0, 1.05, TRACE)
        screen.blit(small.render("OPTICAL", True, MARK), (OPTICAL.x + 6, OPTICAL.y + 4))

        blit_panel(screen, disp.to_pixels(bearing_history, floor=disp.BEARING_FLOOR_DB), BTR)
        blit_panel(screen, disp.to_pixels(range_history), RTR)
        screen.blit(small.render("BEARING-TIME", True, MARK), (BTR.x + 6, BTR.y + 4))
        screen.blit(small.render("RANGE-TIME", True, MARK), (RTR.x + 6, RTR.y + 4))
        # Time runs downward in both records, so the left scale is pings ago.
        for panel in (BTR, RTR):
            for ago in range(0, HISTORY + 1, 50):
                label_left(screen, small, panel, f"{ago:3d}",
                           panel.y + int(panel.height * ago / HISTORY))
        for tick in ticks[::2]:
            label_below(screen, small, BTR,
                        f"{tick:+.0f}", BTR.x + int(BTR.width * (tick + 0.5 * FOV_DEG) / FOV_DEG))
        for metres in range(0, int(MAX_RANGE) + 1, 2):
            label_below(screen, small, RTR, f"{metres}",
                        RTR.x + int(RTR.width * metres / MAX_RANGE))

        for slider in sliders:
            slider.draw(screen, font)
        for toggle in toggles:
            toggle.draw(screen, font)

        target_range = float(np.linalg.norm(centre - position))
        alpha = sonar.thorp_alpha(by_name["freq kHz"].value * 1e3)
        banner = (f"FSS  {by_name['freq kHz'].value:.0f} kHz  "
                  f"lambda {1000 * sim.wavelength_m:.2f} mm  alpha {alpha:.0f} dB/km  "
                  f"target {target_range:.2f} m  advance {advance:.2f} m")
        screen.blit(font.render(banner, True, PHOSPHOR), (60, 10))
        status = (f"{subrays:4d} sub-rays  {N_AZIMUTH}x{N_RANGE}  "
                  f"core {core_ms:5.1f} ms  {clock.get_fps():5.1f} fps  "
                  f"[space] run  [r] reset  [q] quit")
        screen.blit(font.render(status, True, DIM), (900, 10))

        pygame.display.flip()
        clock.tick(FPS_CAP)

    pygame.quit()


if __name__ == "__main__":
    main()
