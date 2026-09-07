"""Forward-scan sonar console, drawn with pygame.

The matplotlib front end in simulator_app.py showed the same panels but spent
about 50 ms a frame inside its image compositing path, against 5 to 10 ms of
actual physics. This draws the same console directly: each waterfall is a numpy
RGB array turned into a surface, scaled once and blitted, so the display cost is
a few milliseconds and the frame rate is set by the simulator rather than by the
plotting library.

    .venv/bin/python python/console.py

Keys: space runs and pauses the platform, f toggles fullscreen, r resets the
run, q or escape quits. The window is resizable and the layout follows it.
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

DESIGN_W, DESIGN_H = 1560, 980
DRAFT_SUBRAYS, MOVING_SUBRAYS, FULL_SUBRAYS = 192, 768, 1280
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

# The console is laid out once at a design size and every rectangle is derived
# from the actual window, so resizing and going fullscreen move the panels
# instead of stretching a scaled bitmap. Rescaling geometry costs nothing per
# frame; scaling a finished frame would cost several milliseconds of the budget
# the pygame port just bought.
def layout(width, height):
    x = width / DESIGN_W
    y = height / DESIGN_H

    def rect(left, top, wide, high):
        return pygame.Rect(round(left * x), round(top * y), round(wide * x), round(high * y))

    panels = {
        "power": rect(60, 34, 1440, 66),
        "main": rect(60, 150, 1440, 320),
        "ascan": rect(60, 486, 706, 56),
        "optical": rect(794, 486, 706, 56),
        "btr": rect(60, 550, 706, 196),
        "rtr": rect(794, 550, 706, 196),
        "compass_y": round(106 * y),
        "relative_y": round(126 * y),
        "banner": (round(60 * x), round(10 * y)),
        "status": (round(900 * x), round(10 * y)),
        "font": max(10, round(13 * min(x, y))),
        "small": max(9, round(12 * min(x, y))),
        "slider": [rect(150 + 470 * column, 800 + 28 * row, 240, 10)
                   for column in (0, 1) for row in range(6)],
        "toggle": [rect(1080, 796 + 28 * row, 16, 16) for row in range(5)],
    }
    return panels


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


# One scaled surface per panel, allocated once. pygame.transform.scale can write
# into a destination surface, which avoids allocating a full panel every frame.
SCALED = {}


def blit_panel(screen, pixels, rect):
    key = id(rect)
    if key not in SCALED:
        # 24-bit to match what frombuffer produces; scale refuses mismatched formats.
        SCALED[key] = pygame.Surface(rect.size, 0, 24)
    pygame.transform.scale(surface_from(pixels), rect.size, SCALED[key])
    screen.blit(SCALED[key], rect.topleft)
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
    screen = pygame.display.set_mode((DESIGN_W, DESIGN_H), pygame.RESIZABLE)
    clock = pygame.time.Clock()

    specs = [("tilt", 0.0, 35.0, 16.0, "{:.0f}"), ("roll", 0.0, 90.0, 0.0, "{:.0f}"),
             ("beamwidth", 4.0, 30.0, 12.0, "{:.0f}"),
             ("freq kHz", 100.0, 2000.0, 900.0, "{:.0f}"),
             ("range m", 2.0, 8.0, 4.5, "{:.2f}"), ("heading", 0.0, 359.0, 348.0, "{:.0f}"),
             ("wave mm", 0.0, 2.0, 0.0, "{:.2f}"), ("turbidity", 0.02, 1.2, 0.10, "{:.2f}"),
             ("texture", 0.0, 0.9, 0.45, "{:.2f}"), ("surge m/s", 0.0, 2.0, 0.6, "{:.2f}"),
             ("yaw deg/s", -40.0, 40.0, 0.0, "{:.0f}"), ("gain dB", -20.0, 20.0, 0.0, "{:.0f}")]
    names = ["multipath", "speckle", "cylinder", "run", "detections"]

    panels = layout(*screen.get_size())
    sliders = [Slider(panels["slider"][index], label, low, high, value, fmt)
               for index, (label, low, high, value, fmt) in enumerate(specs)]
    toggles = [Toggle(panels["toggle"][index], label, state)
               for index, (label, state) in enumerate(zip(names, [True, True, False, True, True]))]
    by_name = {slider.label: slider for slider in sliders}
    toggle_by_name = {toggle.label: toggle for toggle in toggles}
    font = pygame.font.SysFont("Menlo,Monaco,Courier", panels["font"])
    small = pygame.font.SysFont("Menlo,Monaco,Courier", panels["small"])

    def relayout(size):
        """Recompute every rectangle for a new window size."""
        nonlocal panels, font, small
        panels = layout(*size)
        for index, slider in enumerate(sliders):
            slider.rect = panels["slider"][index]
        for index, toggle in enumerate(toggles):
            toggle.rect = panels["toggle"][index]
        font = pygame.font.SysFont("Menlo,Monaco,Courier", panels["font"])
        small = pygame.font.SysFont("Menlo,Monaco,Courier", panels["small"])
        SCALED.clear()

    azimuth_axis = -0.5 * FOV_DEG + FOV_DEG * (np.arange(N_AZIMUTH) + 0.5) / N_AZIMUTH
    range_axis = MAX_RANGE / N_RANGE * (np.arange(N_RANGE) + 0.5)
    ticks = np.linspace(-0.5 * FOV_DEG, 0.5 * FOV_DEG, 11)

    bearing_history = np.full((HISTORY, N_AZIMUTH), disp.BEARING_FLOOR_DB)
    range_history = np.full((HISTORY, N_RANGE), disp.FLOOR_DB)

    advance, frame, held, core_ms, running = 0.0, 0, None, 0.0, True
    optical, fullscreen, windowed_size = None, False, (DESIGN_W, DESIGN_H)
    while running:
        # Full sampling only when the picture is standing still. A moving
        # platform is redrawn often enough that the difference between 768 and
        # 1280 sub-rays is invisible, and a held slider only needs a sketch.
        moving = toggle_by_name["run"].state and by_name["surge m/s"].value > 0.0
        subrays = (DRAFT_SUBRAYS if held is not None
                   else MOVING_SUBRAYS if moving else FULL_SUBRAYS)
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
                elif event.key == pygame.K_f:
                    # Rebuilding the display rather than toggling a flag, because
                    # the layout has to be recomputed for the new size anyway.
                    fullscreen = not fullscreen
                    if fullscreen:
                        windowed_size = screen.get_size()
                        screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
                    else:
                        screen = pygame.display.set_mode(windowed_size, pygame.RESIZABLE)
                    relayout(screen.get_size())
            elif event.type == pygame.VIDEORESIZE and not fullscreen:
                screen = pygame.display.set_mode(event.size, pygame.RESIZABLE)
                relayout(screen.get_size())
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

        # The optical panel shows one row of a slowly changing scene, so it is
        # refreshed on alternate frames rather than every one.
        if optical is None or frame % 2 == 0:
            camera = sonar.OpticalCamera(
                width=160, height=120, focal_px=125.0,
                position=position + np.array([0.0, 0.0, disp.CAMERA_RISE]), axes=axes,
                attenuation_per_m=by_name["turbidity"].value, light_intensity=150.0)
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
        power = panels["power"]
        main_panel = panels["main"]
        ascan = panels["ascan"]
        optical_panel = panels["optical"]
        btr = panels["btr"]
        rtr = panels["rtr"]

        # Bearing power, with the reference line the console draws across the top.
        pygame.draw.rect(screen, FRAME, power, 1)
        pygame.draw.rect(screen, TRACE, pygame.Rect(power.x + 6, power.y + 10, 24, 44))
        for offset in range(0, power.width, 12):
            screen.set_at((power.x + offset, power.y + 6), (255, 255, 255))
        polyline(screen, power, 1.0 + bearing_db / -disp.BEARING_FLOOR_DB, 0.0, 1.08, TRACE)
        cursor = power.x + power.width // 2
        pygame.draw.line(screen, TRACE, (cursor, power.y), (cursor, power.bottom), 2)

        # Two scales: true bearing in red over relative bearing in blue.
        heading = by_name["heading"].value
        for tick in ticks:
            x = main_panel.x + int(main_panel.width * (tick + 0.5 * FOV_DEG) / FOV_DEG)
            compass = small.render(f"{(heading + tick) % 360:03.0f}", True, MARK)
            screen.blit(compass, (x - compass.get_width() // 2, panels["compass_y"]))
            offset = small.render(f"{tick:+.0f}", True, RELATIVE)
            screen.blit(offset, (x - offset.get_width() // 2, panels["relative_y"]))

        # Main display: bearing across, range downward.
        blit_panel(screen, disp.to_pixels(decibels.T), main_panel)
        for metres in range(0, int(MAX_RANGE) + 1, 2):
            y = main_panel.y + int(main_panel.height * metres / MAX_RANGE)
            if metres:
                pygame.draw.line(screen, (40, 12, 10), (main_panel.x + 1, y), (main_panel.right - 1, y))
            label_left(screen, small, main_panel, f"{metres:2d}", y)
        if toggle_by_name["detections"].state:
            peaks = np.argmax(decibels, axis=1)
            lit = decibels[np.arange(N_AZIMUTH), peaks] > disp.FLOOR_DB + 22.0
            columns = main_panel.x + (np.nonzero(lit)[0] * main_panel.width) // N_AZIMUTH
            rows = main_panel.y + (peaks[lit] * main_panel.height) // N_RANGE
            for column, row in zip(columns, rows):
                pygame.draw.rect(screen, MARK, (column, row, 2, 2))

        pygame.draw.rect(screen, FRAME, ascan, 1)
        polyline(screen, ascan, centre_db, disp.FLOOR_DB, 4.0, TRACE)
        screen.blit(small.render("A-SCAN", True, MARK), (ascan.x + 6, ascan.y + 4))

        pygame.draw.rect(screen, FRAME, optical_panel, 1)
        row = optical[optical.shape[0] // 2]
        polyline(screen, optical_panel, row / max(row.max(), 1e-9), 0.0, 1.05, TRACE)
        screen.blit(small.render("OPTICAL", True, MARK), (optical_panel.x + 6, optical_panel.y + 4))

        blit_panel(screen, disp.to_pixels(bearing_history, floor=disp.BEARING_FLOOR_DB), btr)
        blit_panel(screen, disp.to_pixels(range_history), rtr)
        screen.blit(small.render("BEARING-TIME", True, MARK), (btr.x + 6, btr.y + 4))
        screen.blit(small.render("RANGE-TIME", True, MARK), (rtr.x + 6, rtr.y + 4))
        # Time runs downward in both records, so the left scale is pings ago.
        for panel in (btr, rtr):
            for ago in range(0, HISTORY + 1, 50):
                label_left(screen, small, panel, f"{ago:3d}",
                           panel.y + int(panel.height * ago / HISTORY))
        for tick in ticks[::2]:
            label_below(screen, small, btr,
                        f"{tick:+.0f}", btr.x + int(btr.width * (tick + 0.5 * FOV_DEG) / FOV_DEG))
        for metres in range(0, int(MAX_RANGE) + 1, 2):
            label_below(screen, small, rtr, f"{metres}",
                        rtr.x + int(rtr.width * metres / MAX_RANGE))

        for slider in sliders:
            slider.draw(screen, font)
        for toggle in toggles:
            toggle.draw(screen, font)

        target_range = float(np.linalg.norm(centre - position))
        alpha = sonar.thorp_alpha(by_name["freq kHz"].value * 1e3)
        banner = (f"FSS  {by_name['freq kHz'].value:.0f} kHz  "
                  f"lambda {1000 * sim.wavelength_m:.2f} mm  alpha {alpha:.0f} dB/km  "
                  f"target {target_range:.2f} m  advance {advance:.2f} m")
        screen.blit(font.render(banner, True, PHOSPHOR), panels["banner"])
        status = (f"{subrays:4d} sub-rays  {N_AZIMUTH}x{N_RANGE}  "
                  f"core {core_ms:5.1f} ms  {clock.get_fps():5.1f} fps  "
                  f"[space] run  [f] fullscreen  [r] reset  [q] quit")
        rendered = font.render(status, True, DIM)
        screen.blit(rendered, (screen.get_width() - rendered.get_width() - panels["banner"][0],
                               panels["status"][1]))

        pygame.display.flip()
        clock.tick(FPS_CAP)

    pygame.quit()


if __name__ == "__main__":
    main()
