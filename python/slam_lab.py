"""Interactive Pygame view of the repeatable planar SLAM experiment."""

import argparse
import math
import os
import sys

sys.path[:0] = [".", "python"]

import numpy as np
import pygame

import slam
import slam_experiment


NAVY = (7, 18, 29)
PANEL = (13, 31, 47)
GRID = (35, 63, 78)
TEXT = (219, 231, 238)
MUTED = (123, 151, 166)
CYAN = (34, 211, 238)
ORANGE = (249, 115, 22)
RED = (244, 63, 94)
WHITE = (226, 232, 240)


def sonar_surface(image, size):
    values = np.log1p(image / max(float(image.max()), 1e-30) * 400.0)
    values /= max(float(values.max()), 1e-30)
    rgb = np.empty((*values.shape, 3), dtype=np.uint8)
    rgb[..., 0] = 5 + 34 * values
    rgb[..., 1] = 18 + 208 * values
    rgb[..., 2] = 30 + 116 * values
    surface = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
    return pygame.transform.smoothscale(surface, size)


def world_to_screen(points, rectangle, extent=6.7):
    points = np.asarray(points)
    x = rectangle.centerx + points[:, 0] * rectangle.width / (2.0 * extent)
    y = rectangle.centery - points[:, 1] * rectangle.height / (2.0 * extent)
    return np.column_stack((x, y)).astype(int)


def line_path(surface, points, rectangle, colour, width=2):
    screen = world_to_screen(points, rectangle)
    if len(screen) > 1:
        pygame.draw.lines(surface, colour, False, screen.tolist(), width)


def label(surface, font, text, position, colour=TEXT):
    surface.blit(font.render(text, True, colour), position)


def draw_map(surface, result, ping, rectangle, show_correction):
    pygame.draw.rect(surface, PANEL, rectangle, border_radius=8)
    for value in np.linspace(-6, 6, 7):
        vertical = world_to_screen([[value, -6.7], [value, 6.7]], rectangle)
        horizontal = world_to_screen([[-6.7, value], [6.7, value]], rectangle)
        pygame.draw.line(surface, GRID, vertical[0], vertical[1])
        pygame.draw.line(surface, GRID, horizontal[0], horizontal[1])

    truth = result["truth"][:ping + 1]
    dead = result["before"][:ping + 1]
    optimized = result["optimized"][:ping + 1]
    line_path(surface, truth[:, :2], rectangle, WHITE, 2)
    line_path(surface, dead[:, :2], rectangle, ORANGE, 2)
    if show_correction:
        line_path(surface, optimized[:, :2], rectangle, CYAN, 3)

    poses = result["optimized"] if show_correction else result["before"]
    for index in range(ping + 1):
        points = slam.points_in_world(result["features"][index], poses[index])
        for point in world_to_screen(points, rectangle):
            if rectangle.collidepoint(point):
                surface.set_at(point, CYAN if show_correction else ORANGE)

    for first, second, *_ in result["loop_edges"]:
        if second <= ping:
            # Draw the pre-optimization closure residual: after optimization the
            # two nodes nearly coincide, so the actual graph edge is sub-pixel.
            ends = world_to_screen([result["before"][first, :2],
                                    result["before"][second, :2]], rectangle)
            pygame.draw.line(surface, RED, ends[0], ends[1], 3)
    current = world_to_screen([poses[ping, :2]], rectangle)[0]
    pygame.draw.circle(surface, CYAN if show_correction else ORANGE, current, 7)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--screenshot")
    args = parser.parse_args()

    pygame.init()
    screen = pygame.display.set_mode((1440, 900))
    pygame.display.set_caption("Forward-Scan Sonar — SLAM Laboratory")
    clock = pygame.time.Clock()
    title = pygame.font.SysFont("Arial", 28, bold=True)
    body = pygame.font.SysFont("Arial", 17)
    small = pygame.font.SysFont("Arial", 14)
    result = slam_experiment.run_experiment()
    metrics = result["metrics"]
    ping = 0
    running = True
    animate = True
    show_correction = True
    frame = 0
    last_step = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_SPACE:
                    animate = not animate
                elif event.key == pygame.K_r:
                    ping = 0
                elif event.key == pygame.K_l:
                    show_correction = not show_correction

        now = pygame.time.get_ticks()
        if animate and now - last_step > 140:
            ping = (ping + 1) % len(result["images"])
            last_step = now

        screen.fill(NAVY)
        label(screen, title, "FORWARD-SCAN SONAR  /  SONAR–INERTIAL SLAM LAB", (34, 24))
        label(screen, small, "Rendered measurements → feature extraction → IMU edges → scan matching → loop closure → C++ pose graph", (36, 62), MUTED)

        sonar_rect = pygame.Rect(36, 105, 650, 640)
        map_rect = pygame.Rect(720, 105, 684, 640)
        pygame.draw.rect(screen, PANEL, sonar_rect, border_radius=8)
        image_rect = sonar_rect.inflate(-28, -72)
        image_rect.top += 20
        screen.blit(sonar_surface(result["images"][ping], image_rect.size), image_rect)
        label(screen, body, f"PING {ping:02d}  •  RANGE–BEARING RETURN", (sonar_rect.x + 14, sonar_rect.y + 12))
        label(screen, small, f"{len(result['features'][ping])} image maxima become local planar landmarks", (sonar_rect.x + 16, sonar_rect.bottom - 32), MUTED)

        draw_map(screen, result, ping, map_rect, show_correction)
        label(screen, body, "TRAJECTORY AND ACCUMULATED FEATURE MAP", (map_rect.x + 14, map_rect.y + 12))
        label(screen, small, "truth", (map_rect.x + 18, map_rect.bottom - 35), WHITE)
        label(screen, small, "IMU drift", (map_rect.x + 80, map_rect.bottom - 35), ORANGE)
        label(screen, small, "optimized", (map_rect.x + 165, map_rect.bottom - 35), CYAN)
        label(screen, small, "pre-opt loop residual", (map_rect.x + 252, map_rect.bottom - 35), RED)

        card_y = 770
        cards = [
            ("POSITION RMSE", f"{metrics['graph_before_rmse_m']:.3f} → {metrics['optimized_rmse_m']:.3f} m"),
            ("LOOP CLOSURE", f"{metrics['closure_before_m']:.3f} → {metrics['closure_after_m']:.4f} m"),
            ("CONSTRAINTS", f"{metrics['scan_constraint_count']} scan  +  {metrics['loop_closure_count']} loop"),
            ("ESTIMATOR", "planar SE(2), robust Gauss–Newton"),
        ]
        for index, (heading, value) in enumerate(cards):
            rect = pygame.Rect(36 + index * 342, card_y, 320, 82)
            pygame.draw.rect(screen, PANEL, rect, border_radius=8)
            label(screen, small, heading, (rect.x + 14, rect.y + 12), MUTED)
            label(screen, body, value, (rect.x + 14, rect.y + 39))
        label(screen, small, "SPACE pause   R restart   L toggle correction   Q quit", (36, 872), MUTED)
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
