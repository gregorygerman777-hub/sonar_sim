"""Interactive Pygame laboratory driven by the external Fortran BELLHOP solver."""

import argparse
import math
from pathlib import Path
import sys
import tempfile
import time

sys.path[:0] = [".", "python"]

import numpy as np
import pygame

import bellhop


NAVY = (6, 16, 26)
PANEL = (12, 29, 44)
GRID = (32, 57, 72)
TEXT = (221, 232, 239)
MUTED = (121, 149, 163)
CYAN = (34, 211, 238)
PINK = (244, 63, 94)
GOLD = (250, 204, 21)
WHITE = (226, 232, 240)
GREEN = (74, 222, 128)


class Slider:
    def __init__(self, name, minimum, maximum, value, units, integer=False):
        self.name = name
        self.minimum = minimum
        self.maximum = maximum
        self.value = value
        self.units = units
        self.integer = integer
        self.dragging = False
        self.track = pygame.Rect(0, 0, 100, 6)

    def set_x(self, x):
        fraction = np.clip((x - self.track.left) / self.track.width, 0.0, 1.0)
        value = self.minimum + fraction * (self.maximum - self.minimum)
        self.value = int(round(value)) if self.integer else float(value)

    def handle(self, event):
        changed = False
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            knob_x = self.track.left + self.track.width * (self.value - self.minimum) / (self.maximum - self.minimum)
            if self.track.inflate(16, 24).collidepoint(event.pos) or abs(event.pos[0] - knob_x) < 12:
                self.dragging = True
                self.set_x(event.pos[0])
                changed = True
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self.set_x(event.pos[0])
            changed = True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1 and self.dragging:
            self.set_x(event.pos[0])
            self.dragging = False
            changed = True
        return changed

    def draw(self, surface, font, y):
        self.track = pygame.Rect(1000, y + 30, 350, 5)
        pygame.draw.rect(surface, GRID, self.track, border_radius=3)
        fraction = (self.value - self.minimum) / (self.maximum - self.minimum)
        knob_x = int(self.track.left + fraction * self.track.width)
        pygame.draw.line(surface, CYAN, self.track.topleft, (knob_x, self.track.centery), 5)
        pygame.draw.circle(surface, WHITE if self.dragging else CYAN, (knob_x, self.track.centery), 8)
        label(surface, font, self.name.upper(), (1000, y), MUTED)
        suffix = f"{int(self.value)}" if self.integer else f"{self.value:.1f}"
        rendered = font.render(f"{suffix} {self.units}", True, TEXT)
        surface.blit(rendered, (1350 - rendered.get_width(), y))


def label(surface, font, text, position, colour=TEXT):
    surface.blit(font.render(text, True, colour), position)


def ray_kind(item):
    if item["top_bounces"] and item["bottom_bounces"]:
        return "combined", WHITE
    if item["top_bounces"]:
        return "surface", PINK
    if item["bottom_bounces"]:
        return "bottom", GOLD
    return "direct", CYAN


def graph_point(range_m, depth_m, rectangle, max_range_m, water_depth_m):
    x = rectangle.left + range_m / max_range_m * rectangle.width
    y = rectangle.top + depth_m / water_depth_m * rectangle.height
    return int(x), int(y)


def draw_rays(surface, result, environment, rectangle):
    pygame.draw.rect(surface, PANEL, rectangle, border_radius=8)
    for fraction in np.linspace(0.0, 1.0, 7):
        x = int(rectangle.left + fraction * rectangle.width)
        y = int(rectangle.top + fraction * rectangle.height)
        pygame.draw.line(surface, GRID, (x, rectangle.top), (x, rectangle.bottom))
        pygame.draw.line(surface, GRID, (rectangle.left, y), (rectangle.right, y))
    counts = {"direct": 0, "surface": 0, "bottom": 0, "combined": 0}
    for ray in result["rays"]:
        kind, colour = ray_kind(ray)
        counts[kind] += 1
        points = ray["points_m"]
        screen = [graph_point(p[0], p[1], rectangle, environment.max_range_m,
                              environment.water_depth_m) for p in points
                  if 0.0 <= p[0] <= environment.max_range_m]
        if len(screen) > 1:
            pygame.draw.lines(surface, colour, False, screen, 1)
    source = graph_point(0.0, environment.source_depth_m, rectangle,
                         environment.max_range_m, environment.water_depth_m)
    receiver = graph_point(environment.max_range_m, environment.receiver_depth_m, rectangle,
                           environment.max_range_m, environment.water_depth_m)
    pygame.draw.circle(surface, GREEN, source, 7)
    pygame.draw.circle(surface, WHITE, receiver, 7, 2)
    return counts


def draw_arrivals(surface, arrival_result, environment, rectangle, font, two_way):
    pygame.draw.rect(surface, PANEL, rectangle, border_radius=8)
    arrivals = arrival_result["records"][-1]["arrivals"]
    if not arrivals:
        return 0
    if two_way:
        plotted = bellhop.two_way_paths(arrivals)
        horizontal = np.array([item["apparent_range_m"] for item in plotted])
        levels = 10.0 * np.log10(np.maximum(
            [item["relative_intensity"] for item in plotted], 1e-15))
        padding = max(0.5, 0.04 * np.ptp(horizontal))
        suffix = "m"
    else:
        plotted = arrivals
        horizontal = np.array([item["delay_s"] for item in plotted]) * 1000.0
        levels = 20.0 * np.log10(np.maximum(
            [item["amplitude"] for item in plotted], 1e-15))
        padding = 0.5
        suffix = "ms"
    left, right = horizontal.min() - padding, horizontal.max() + padding
    floor = min(-80.0, float(levels.min()) - 4.0)
    for level in np.linspace(floor, 0.0, 5):
        y = rectangle.bottom - (level - floor) / -floor * rectangle.height
        pygame.draw.line(surface, GRID, (rectangle.left, y), (rectangle.right, y))
    for item, coordinate, level in zip(plotted, horizontal, levels):
        if two_way and item["mixed"]:
            colour = PINK
        else:
            _, colour = ray_kind(item)
        x = rectangle.left + (coordinate - left) / (right - left) * rectangle.width
        y = rectangle.bottom - (level - floor) / -floor * rectangle.height
        pygame.draw.line(surface, colour, (x, rectangle.bottom), (x, y), 2)
        pygame.draw.circle(surface, colour, (int(x), int(y)), 4)
    label(surface, font, f"{left:.1f} {suffix}", (rectangle.left, rectangle.bottom + 5), MUTED)
    label(surface, font, f"{right:.1f} {suffix}", (rectangle.right - 55, rectangle.bottom + 5), MUTED)
    return len(plotted)


def environment_from(sliders):
    values = {slider.name: slider.value for slider in sliders}
    water_depth = values["water depth"]
    source_depth = min(values["source depth"], water_depth - 1.0)
    receiver_depth = min(values["receiver depth"], water_depth - 1.0)
    return bellhop.BellhopEnvironment(
        frequency_hz=values["frequency"] * 1000.0,
        water_depth_m=water_depth,
        source_depth_m=source_depth,
        receiver_depth_m=receiver_depth,
        max_range_m=values["range"],
        sound_speed_surface_mps=values["surface sound speed"],
        sound_speed_bottom_mps=values["bottom water sound speed"],
        bottom_sound_speed_mps=values["sediment sound speed"],
        bottom_density_gcm3=values["sediment density"],
        launch_min_deg=-values["launch aperture"],
        launch_max_deg=values["launch aperture"],
        ray_count=values["rays"],
    )


def solve(environment, workspace):
    start = time.perf_counter()
    rays, arrivals, metadata = bellhop.solve(environment, workspace)
    return rays, arrivals, metadata, 1000.0 * (time.perf_counter() - start)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--screenshot")
    args = parser.parse_args()
    pygame.init()
    screen = pygame.display.set_mode((1440, 900))
    pygame.display.set_caption("Forward-Scan Sonar — BELLHOP Propagation Laboratory")
    clock = pygame.time.Clock()
    title_font = pygame.font.SysFont("Arial", 27, bold=True)
    body_font = pygame.font.SysFont("Arial", 16)
    small_font = pygame.font.SysFont("Arial", 13)
    sliders = [
        Slider("frequency", 5, 100, 30, "kHz"),
        Slider("water depth", 12, 80, 30, "m"),
        Slider("source depth", 1, 70, 5, "m"),
        Slider("receiver depth", 1, 70, 5, "m"),
        Slider("range", 50, 500, 120, "m"),
        Slider("surface sound speed", 1450, 1540, 1490, "m/s"),
        Slider("bottom water sound speed", 1450, 1560, 1520, "m/s"),
        Slider("sediment sound speed", 1500, 2200, 1700, "m/s"),
        Slider("sediment density", 1.0, 2.5, 1.8, "g/cm³"),
        Slider("launch aperture", 10, 80, 35, "deg"),
        Slider("rays", 81, 401, 241, "", integer=True),
    ]
    workspace = Path(tempfile.mkdtemp(prefix="sonar_bellhop_lab_"))
    environment = environment_from(sliders)
    ray_result, arrival_result, metadata, runtime_ms = solve(environment, workspace)
    dirty = False
    two_way = True
    running = True
    frame = 0
    while running:
        released = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_r:
                    released = True
                elif event.key == pygame.K_t:
                    two_way = not two_way
            for slider in sliders:
                if slider.handle(event):
                    dirty = True
            if event.type == pygame.MOUSEBUTTONUP and dirty:
                released = True
        if released:
            environment = environment_from(sliders)
            ray_result, arrival_result, metadata, runtime_ms = solve(environment, workspace)
            dirty = False

        screen.fill(NAVY)
        label(screen, title_font, "ACOUSTICS TOOLBOX  /  BELLHOP PROPAGATION LAB", (34, 22))
        label(screen, small_font,
              "Environment file → external Fortran ray trace → ASCII rays + complex-amplitude arrivals → parsed visualization",
              (36, 61), MUTED)
        label(screen, body_font, "BELLHOP FORTRAN ONLINE", (1170, 26), GREEN)

        ray_panel = pygame.Rect(36, 112, 910, 500)
        pygame.draw.rect(screen, PANEL, ray_panel, border_radius=8)
        ray_rect = pygame.Rect(36, 165, 910, 447)
        arrival_rect = pygame.Rect(36, 675, 910, 145)
        counts = draw_rays(screen, ray_result, environment, ray_rect)
        label(screen, body_font, "RANGE–DEPTH EIGENRAY FAN", (ray_panel.x + 14, ray_panel.y + 12))
        label(screen, small_font, "green source  •  white receiver", (ray_panel.x + 14, ray_panel.y + 37), MUTED)
        label(screen, small_font, "0 m", (ray_rect.x, ray_rect.bottom + 5), MUTED)
        label(screen, small_font, f"{environment.max_range_m:.0f} m", (ray_rect.right - 45, ray_rect.bottom + 5), MUTED)

        arrival_count = draw_arrivals(screen, arrival_result, environment, arrival_rect,
                                      small_font, two_way)
        heading = ("MONOSTATIC TWO-WAY OBJECT / GHOST / MIRROR RETURNS" if two_way else
                   f"ONE-WAY EIGENRAY ARRIVALS AT {environment.max_range_m:.0f} m")
        label(screen, body_font, heading, (arrival_rect.x + 14, arrival_rect.y + 10))

        pygame.draw.rect(screen, PANEL, pygame.Rect(974, 112, 430, 708), border_radius=8)
        label(screen, body_font, "ENVIRONMENT AND SOLVER INPUTS", (996, 130))
        for index, slider in enumerate(sliders):
            slider.draw(screen, small_font, 166 + index * 51)
        profile_y = 738
        c0 = environment.sound_speed_surface_mps
        c1 = environment.sound_speed_bottom_mps
        label(screen, small_font, f"SSP: {c0:.0f} → {c1:.0f} m/s, piecewise linear", (1000, profile_y), MUTED)
        result_name = "two-way paths" if two_way else "far-range arrivals"
        label(screen, small_font, f"solver {runtime_ms:.1f} ms  •  {arrival_count} {result_name}", (1000, profile_y + 25), TEXT)
        label(screen, small_font, f"rays: direct {counts['direct']}  surface {counts['surface']}  bottom {counts['bottom']}  both {counts['combined']}", (1000, profile_y + 48), TEXT)
        label(screen, small_font, "CYAN object/direct   PINK ghost/mixed   GOLD bottom   WHITE combined", (36, 855), MUTED)
        label(screen, small_font, "Drag and release to solve   •   T one-way/two-way   •   R rerun   •   Q quit", (735, 855), MUTED)
        pygame.display.flip()

        frame += 1
        if args.frames and frame >= args.frames:
            if args.screenshot:
                pygame.image.save(screen, args.screenshot)
            running = False
        clock.tick(60)
    pygame.quit()


if __name__ == "__main__":
    main()
