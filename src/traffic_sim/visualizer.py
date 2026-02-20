from __future__ import annotations

import os

import pygame

from .recorder import Recorder
from .simulation import Simulation

# ── Colours ────────────────────────────────────────────────────────────────
BG_COLOR        = (20,  22,  30)
ROAD_COLOR      = (55,  57,  65)
EDGE_COLOR      = (230, 230, 230)
LANE_MARK_COLOR = (200, 190, 100)
HUD_COLOR       = (210, 215, 225)
RAMP_ON_COLOR   = (80,  200,  80)
RAMP_OFF_COLOR  = (200,  80,  80)

# Speed gradient: red → yellow → green  (in m/s)
_SPEED_LOW  = (220,  50,  50)   # 0 km/h
_SPEED_MID  = (250, 200,  50)   # ~60 km/h
_SPEED_HIGH = ( 50, 200, 100)   # ≥ 120 km/h (33 m/s)

# ── Layout constants ────────────────────────────────────────────────────────
LANE_HEIGHT = 52   # px per lane
MARGIN      = 55   # px left/right/top border
HUD_HEIGHT  = 140  # px below road

# Car visual sizes: cw (along road) > ch (across road), front bumper = right edge of rect
CAR_W   = 16   # px along road  — regular car
CAR_H   = 10   # px across road — regular car
TRUCK_W = 26   # px along road  — truck
TRUCK_H = 14   # px across road — truck


def _speed_color(v_ms: float, v_max: float = 33.0) -> tuple[int, int, int]:
    t = min(v_ms / v_max, 1.0)
    if t < 0.5:
        s = t * 2.0
        lo, hi = _SPEED_LOW, _SPEED_MID
    else:
        s = (t - 0.5) * 2.0
        lo, hi = _SPEED_MID, _SPEED_HIGH
    return (
        int(lo[0] + (hi[0] - lo[0]) * s),
        int(lo[1] + (hi[1] - lo[1]) * s),
        int(lo[2] + (hi[2] - lo[2]) * s),
    )


class Visualizer:
    def __init__(self, sim: Simulation, recorder: Recorder | None = None,
                 width: int = 1400, fps: int = 60):
        pygame.init()
        pygame.font.init()

        self.sim      = sim
        self.recorder = recorder
        self.fps      = fps
        self.running  = True
        self.paused   = False
        self.speed_mult: float = 1.0

        road_h = sim.road.num_lanes * LANE_HEIGHT
        self.W = width
        # Extra vertical space for on-ramp queue lane (if on-ramp is active)
        self.has_onramp = sim.cfg.ramp.onramp_rate > 0
        ramp_extra = LANE_HEIGHT if self.has_onramp else 0
        self.H = MARGIN + road_h + ramp_extra + HUD_HEIGHT

        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption("Traffic Simulator")
        self.clock = pygame.time.Clock()

        self.font     = pygame.font.SysFont("monospace", 15)
        self.font_sm  = pygame.font.SysFont("monospace", 12)

        # Road rect
        self.rx = MARGIN
        self.ry = MARGIN
        self.rw = self.W - MARGIN * 2
        self.rh = road_h
        self.scale = self.rw / sim.road.length   # px per metre
        # Ramp lane geometry (directly below road)
        self.ramp_lane_top = self.ry + self.rh
        self.ramp_lane_cy  = self.ramp_lane_top + LANE_HEIGHT // 2
        self.hud_extra     = ramp_extra   # pixels between road bottom and HUD

    # ── Coordinate helpers ─────────────────────────────────────────────────

    def _px(self, position: float) -> int:
        return self.rx + int(position * self.scale)

    def _lane_cy(self, lane: float) -> int:
        return self.ry + int((lane + 0.5) * LANE_HEIGHT)

    # ── Drawing ────────────────────────────────────────────────────────────

    def _draw_road(self) -> None:
        pygame.draw.rect(self.screen, ROAD_COLOR,
                         (self.rx, self.ry, self.rw, self.rh))

        # Top / bottom solid edges
        pygame.draw.line(self.screen, EDGE_COLOR,
                         (self.rx, self.ry), (self.rx + self.rw, self.ry), 3)
        pygame.draw.line(self.screen, EDGE_COLOR,
                         (self.rx, self.ry + self.rh),
                         (self.rx + self.rw, self.ry + self.rh), 3)

        # Dashed lane dividers
        for lane in range(1, self.sim.road.num_lanes):
            y = self.ry + lane * LANE_HEIGHT
            x = self.rx
            while x < self.rx + self.rw:
                end_x = min(x + 28, self.rx + self.rw)
                pygame.draw.line(self.screen, LANE_MARK_COLOR, (x, y), (end_x, y), 2)
                x += 28 + 18  # dash + gap

    def _draw_ramps(self) -> None:
        for ramp in self.sim.road.ramps:
            x = self._px(ramp.position)
            if ramp.is_onramp:
                # On-ramp is represented by the queue lane drawn separately
                continue
            # Off-ramp: triangle above road
            color = RAMP_OFF_COLOR
            y_base = self.ry
            pts = [(x - 8, y_base - 18), (x + 8, y_base - 18), (x, y_base - 2)]
            pygame.draw.polygon(self.screen, color, pts)
            surf = self.font_sm.render("OFF", True, color)
            self.screen.blit(surf, (x - surf.get_width() // 2, pts[0][1] - surf.get_height() - 2))

    def _draw_ramp_queue_lane(self) -> None:
        """Draw the on-ramp queue lane below the main road, with waiting cars."""
        if not self.has_onramp:
            return
        ramp_length_m = self.sim.cfg.ramp.ramp_length_m
        ramp_length_px = int(ramp_length_m * self.scale)
        lane_top = self.ramp_lane_top
        lane_cy  = self.ramp_lane_cy

        for ramp in self.sim.road.ramps:
            if not ramp.is_onramp:
                continue
            ramp_x = self._px(ramp.position)

            # Ramp lane background
            pygame.draw.rect(self.screen, ROAD_COLOR,
                             (ramp_x - ramp_length_px, lane_top, ramp_length_px, LANE_HEIGHT))
            # Bottom edge
            pygame.draw.line(self.screen, EDGE_COLOR,
                             (ramp_x - ramp_length_px, lane_top + LANE_HEIGHT),
                             (ramp_x, lane_top + LANE_HEIGHT), 2)
            # Left (closed) end
            pygame.draw.line(self.screen, EDGE_COLOR,
                             (ramp_x - ramp_length_px, lane_top),
                             (ramp_x - ramp_length_px, lane_top + LANE_HEIGHT), 2)

            # Merge connector: small angled wedge joining ramp to rightmost road lane
            rightmost_cy = self._lane_cy(self.sim.road.num_lanes - 1)
            hw = LANE_HEIGHT // 2  # half-width of connector
            wedge = [
                (ramp_x - hw, lane_top),                        # ramp lane top-left
                (ramp_x + hw, rightmost_cy - LANE_HEIGHT // 2), # road lane top-right
                (ramp_x + hw, rightmost_cy + LANE_HEIGHT // 2), # road lane bottom-right
                (ramp_x - hw, lane_top + LANE_HEIGHT),          # ramp lane bottom-left
            ]
            pygame.draw.polygon(self.screen, ROAD_COLOR, wedge)
            pygame.draw.line(self.screen, EDGE_COLOR,
                             (ramp_x - hw, lane_top),
                             (ramp_x + hw, rightmost_cy - LANE_HEIGHT // 2), 1)
            pygame.draw.line(self.screen, EDGE_COLOR,
                             (ramp_x - hw, lane_top + LANE_HEIGHT),
                             (ramp_x + hw, rightmost_cy + LANE_HEIGHT // 2), 2)

            # "ON" label
            surf = self.font_sm.render("ON", True, RAMP_ON_COLOR)
            self.screen.blit(surf, (ramp_x - ramp_length_px + 4, lane_top + 3))

            # Queue cars
            for car in ramp.queue:
                cx = ramp_x - int((ramp_length_m - car.position) * self.scale)
                is_truck = car.length > 8.0
                cw = TRUCK_W if is_truck else CAR_W
                ch = TRUCK_H if is_truck else CAR_H
                color = _speed_color(car.velocity)
                rect = pygame.Rect(cx - cw, lane_cy - ch // 2, cw, ch)
                pygame.draw.rect(self.screen, color, rect, border_radius=3)
                pygame.draw.rect(self.screen, (15, 15, 15), rect, 1, border_radius=3)
                lbl = self.font_sm.render(f"{car.velocity * 3.6:.0f}", True, (10, 10, 10))
                self.screen.blit(lbl, (cx - cw // 2 - lbl.get_width() // 2,
                                       lane_cy - lbl.get_height() // 2))

    def _draw_cars(self) -> None:
        for car in self.sim.cars:
            cx = self._px(car.position)
            cy = self._lane_cy(self.sim.get_visual_lane(car))
            is_truck = car.length > 8.0
            cw = TRUCK_W if is_truck else CAR_W   # long axis — along road
            ch = TRUCK_H if is_truck else CAR_H   # short axis — across road

            color = _speed_color(car.velocity)
            # cx is the front bumper — rect extends cw px behind it
            rect = pygame.Rect(cx - cw, cy - ch // 2, cw, ch)
            pygame.draw.rect(self.screen, color, rect, border_radius=3)
            pygame.draw.rect(self.screen, (15, 15, 15), rect, 1, border_radius=3)

            # Speed label centred on the rect
            lbl = self.font_sm.render(f"{car.velocity * 3.6:.0f}", True, (10, 10, 10))
            self.screen.blit(lbl, (cx - cw // 2 - lbl.get_width() // 2,
                                   cy - lbl.get_height() // 2))

    def _draw_hud(self) -> None:
        hud_y = self.ry + self.rh + self.hud_extra + 14
        status = "PAUSED" if self.paused else "RUNNING"

        limits_str = "  |  ".join(
            f"L{i + 1} {lim * 3.6:.0f} km/h"
            for i, lim in enumerate(self.sim.road.lane_speed_limits)
        )

        lines = [
            f"Time: {self.sim.time:7.1f}s  |  Speed: {self.speed_mult:.2f}x  |  {status}",
            f"Cars: {self.sim.car_count:3d}      |  Avg speed: {self.sim.avg_speed_kmh:5.1f} km/h",
            f"Limits (L\u2192R):  {limits_str}",
            "Controls:  [SPACE] pause   [↑/↓] sim speed   [Q] quit",
        ]
        for i, line in enumerate(lines):
            surf = self.font.render(line, True, HUD_COLOR)
            self.screen.blit(surf, (self.rx, hud_y + i * 22))

        # Density bar
        self._draw_density_bar(hud_y + 94)

    def _draw_density_bar(self, y: int) -> None:
        num_bins = 40
        bin_w = self.rw // num_bins
        counts = [0] * num_bins
        for car in self.sim.cars:
            idx = min(int(car.position / self.sim.road.length * num_bins), num_bins - 1)
            counts[idx] += 1
        max_count = max(counts) if any(counts) else 1
        bar_h = 22
        lbl = self.font_sm.render("density:", True, HUD_COLOR)
        self.screen.blit(lbl, (self.rx, y - 1))
        offset = lbl.get_width() + 6
        for i, cnt in enumerate(counts):
            h = max(2, int(cnt / max_count * bar_h))
            col = _speed_color(cnt / max_count * 33.0)
            x = self.rx + offset + i * bin_w
            pygame.draw.rect(self.screen, col, (x, y + bar_h - h, bin_w - 1, h))

    # ── Event handling ─────────────────────────────────────────────────────

    def _handle_events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_UP:
                    self.speed_mult = min(self.speed_mult * 2.0, 16.0)
                elif event.key == pygame.K_DOWN:
                    self.speed_mult = max(self.speed_mult / 2.0, 0.25)

    # ── Main loop ──────────────────────────────────────────────────────────

    def run(self) -> None:
        while self.running:
            self._handle_events()

            dt_wall = self.clock.tick(self.fps) / 1000.0

            if not self.paused:
                dt_sim = dt_wall * self.speed_mult
                # Substeps keep IDM numerically stable at high speed multipliers
                substeps = max(1, int(self.speed_mult))
                dt_sub = min(dt_sim / substeps, 0.05)
                for _ in range(substeps):
                    self.sim.step(dt_sub)
                if self.recorder:
                    self.recorder.sample(self.sim)

            self.screen.fill(BG_COLOR)
            self._draw_road()
            self._draw_ramp_queue_lane()
            self._draw_ramps()
            self._draw_cars()
            self._draw_hud()
            pygame.display.flip()

        if self.recorder:
            os.makedirs("logs", exist_ok=True)
            paths = self.recorder.save("logs")
            if paths:
                print(f"Recorded {self.recorder.sample_count} samples.")
                for p in paths:
                    print(f"  → {p}")
        pygame.quit()
