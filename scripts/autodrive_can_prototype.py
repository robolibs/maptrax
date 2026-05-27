#!/usr/bin/env python3
"""
Live GP AutoDrive bridge prototype.

No CSV. No CLI flags.

Edit the CONFIG SECTION, then run:

    ./scripts/autodrive_can_prototype.py

Shape of this script:

1. Generate a Maptrax field with AB/swath lines.
2. Build one continuous machine path from those lines.
3. Listen to the machine/display CAN messages:
   - VP1   0xFFEF: current GPS latitude/longitude
   - VDS   0xFEE8: heading/speed/etc
   - DSSTAT 0xFFCA: PPP, AutoDrive allowed, engage status, reject reason
   - DSAP  0xFFCB: anchor latitude/longitude
4. Send ADJOB 0xFFCC to activate/run/report progress.
5. Send ADWPI 0xFFCD for the *current + future 100 points* continuously.

The protocol packing is implemented from the tables in
`xtra/GP_AutoDrive_CanMessageProposal_V10.pdf`.
Keep the constants near the top editable because source address and a couple
of flag bit positions were marked/questioned in the proposal.
"""

from __future__ import annotations

import dataclasses
import json
import math
import struct
import sys
import time
from pathlib import Path
from typing import Sequence


# =============================================================================
# CONFIG SECTION — EDIT THIS BY HAND
# =============================================================================

# Use "fake" to print frames. Use "socketcan" to talk to a real SocketCAN iface.
CAN_BACKEND = "socketcan"
CAN_CHANNEL = "can0"

# Built-in demo field is used when FIELD_JSON_PATH is None.
# If set, file must contain JSON like: [[0, 0], [100, 0], [100, 50], [0, 50]]
FIELD_JSON_PATH: str | None = None

FIELD_BOUNDARY_ENU = [
    (0.0, 0.0),
    (-8.795, -3.606),
    (-10.674, -10.385),
    (-5.401, -20.865),
    (7.202, -27.906),
    (50.837, -39.694),
    (160.405, -46.377),
    (161.651, 5.508),
    (91.502, 16.831),
]

# Maptrax local ENU datum.
DATUM_LAT = 51.0
DATUM_LON = 5.0
DATUM_ALT = 0.0

# Field/AB-line generation.
SWATH_WIDTH_M = 3.0
SWATH_ANGLE_DEGREES = 0.0
HEADLAND_COUNT = 2
PART_INDEX = 0

# Machine route selection.
MACHINE_COUNT = 1
MACHINE_INDEX = 0
DIVISION_PATTERN = "block"
DIVISION_BALANCE = "count"
ROUTING_STRATEGY = "snake"
TURN_MODEL = "reeds_shepp"
CONNECTOR_MODE = "headland"
MIN_TURNING_RADIUS_M = 2.0
TURN_STEP_SIZE_M = 0.2
MACHINE_LENGTH_M = 6.0
MACHINE_WIDTH_M = 3.0

# True: send full route including headlands/turn connectors.
# False: send only ordered AB/swath line endpoints.
USE_TOUR_WITH_TURNS = True

# Resample generated path so ADWPI has dense line points.
# Proposal recommends gentle curves; do not make this huge.
WAYPOINT_SPACING_M = 0.5

# Start/run gates. Set to False only for bench testing.
REQUIRE_GPS_PPP = True
REQUIRE_AUTODRIVE_ALLOWED = True
REQUIRE_INSIDE_FIELD = True

# J1939/proposal constants.
BUS_SOURCE_DISPLAY = 40
SOURCE_AUTODRIVE = 29  # proposal says "29 ?" for ADJOB/ADWPI source
J1939_PRIORITY = 6

PGN_VP1 = 0xFFEF
PGN_VDS = 0xFEE8
PGN_DSSTAT = 0xFFCA
PGN_DSAP = 0xFFCB
PGN_ADJOB = 0xFFCC
PGN_ADWPI = 0xFFCD

JOB_ID = 1

# ADWPI coordinate table: coordinate value = raw + offset, 1 cm/bit.
ADWPI_COORD_OFFSET_CM = -250_000
ADWPI_COORD_RAW_MAX = (1 << 20) - 1

# Byte8 flag bits from the ADWPI table. These are intentionally editable.
ADWPI_FLAG_HEADLAND = 0x01
ADWPI_FLAG_REVERSE = 0x04

# Future-window behavior.
FUTURE_POINT_COUNT = 100
WINDOW_OVERLAP_POINTS = 3
RESEND_WINDOW_EVERY_S = 1.0
SEND_INTERVAL_S = 0.010

# ADJOB transmit rate: proposal says every 1s or on state change.
ADJOB_PERIOD_S = 1.0

# If GPS is missing, the loop cannot estimate progress. It will keep index 0.
NEAREST_SEARCH_BACKTRACK = 3
NEAREST_SEARCH_AHEAD = 200


# =============================================================================
# DATA TYPES
# =============================================================================

try:
    import maptrax
except Exception as exc:  # pragma: no cover - local environment
    maptrax = None
    MAPTRAX_IMPORT_ERROR = exc
else:
    MAPTRAX_IMPORT_ERROR = None


@dataclasses.dataclass
class RoutePoint:
    x: float
    y: float
    is_headland: bool = False
    is_reverse: bool = False


@dataclasses.dataclass
class Waypoint:
    index: int
    x: float
    y: float
    east_cm: int = 0
    north_cm: int = 0
    is_headland: bool = False
    is_reverse: bool = False


@dataclasses.dataclass
class MachineStatus:
    gps_lat: float | None = None
    gps_lon: float | None = None
    speed_kph: float = 0.0
    heading_deg: float | None = None
    gps_ppp_available: bool = False
    autodrive_allowed: bool = False
    autosteer_engaged: bool = False
    header_down: bool = False
    current_direction_reverse: bool = False
    reject_reason: int = 0
    anchor_lat: float | None = None
    anchor_lon: float | None = None
    last_rx_s: float = 0.0


@dataclasses.dataclass
class CanFrame:
    arbitration_id: int
    data: bytes


# =============================================================================
# MAPTRAX FIELD + AB LINES
# =============================================================================

def require_maptrax():
    if maptrax is None:
        raise SystemExit(
            "Could not import maptrax. Build/install the local binding first, "
            "for example: maturin develop --features python. "
            f"Original error: {MAPTRAX_IMPORT_ERROR}"
        )
    return maptrax


def field_boundary_enu() -> list[tuple[float, float]]:
    if FIELD_JSON_PATH is None:
        return list(FIELD_BOUNDARY_ENU)
    raw = json.loads(Path(FIELD_JSON_PATH).read_text())
    return [(float(x), float(y)) for x, y in raw]


def generate_route_points() -> list[RoutePoint]:
    mt = require_maptrax()
    planner = mt.Maptrax()

    planner.set_field(field_boundary_enu(), (DATUM_LAT, DATUM_LON, DATUM_ALT))
    planner.generate_field(SWATH_WIDTH_M, SWATH_ANGLE_DEGREES, HEADLAND_COUNT)

    plan = planner.plan_machines(
        part_index=PART_INDEX,
        machines=MACHINE_COUNT,
        pattern=DIVISION_PATTERN,
        balance=DIVISION_BALANCE,
        routing_strategy=ROUTING_STRATEGY,
        turn_model=TURN_MODEL,
        connector_mode=CONNECTOR_MODE,
        min_turning_radius=MIN_TURNING_RADIUS_M,
        step_size=TURN_STEP_SIZE_M,
        machine_length=MACHINE_LENGTH_M,
        machine_width=MACHINE_WIDTH_M,
        swath_width=SWATH_WIDTH_M,
    )

    machine = plan["machines"][MACHINE_INDEX]
    segments = machine["tour"] if USE_TOUR_WITH_TURNS else machine["ordered_swaths"]
    points = route_points_from_segments(segments)
    points = resample_route(points, WAYPOINT_SPACING_M)

    print(
        f"generated AB route: segments={len(segments)} points={len(points)} "
        f"tour={USE_TOUR_WITH_TURNS}",
        file=sys.stderr,
    )
    return points


def route_points_from_segments(segments: Sequence[dict]) -> list[RoutePoint]:
    out: list[RoutePoint] = []

    for segment in segments:
        typ = str(segment.get("type", "")).lower()
        points = [(float(x), float(y)) for x, y in segment.get("points", [])]
        reverse_flags = list(segment.get("point_reverse") or [])
        if len(reverse_flags) != len(points):
            reverse_flags = [False] * len(points)

        is_headland = typ == "headland"

        for (x, y), reverse in zip(points, reverse_flags):
            point = RoutePoint(x=x, y=y, is_headland=is_headland, is_reverse=bool(reverse))
            if out and point_dist(out[-1], point) < 1e-9:
                previous = out[-1]
                previous.is_headland = previous.is_headland or point.is_headland
                previous.is_reverse = previous.is_reverse or point.is_reverse
            else:
                out.append(point)

    return out


def point_dist(a: RoutePoint, b: RoutePoint) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def resample_route(points: Sequence[RoutePoint], spacing_m: float) -> list[RoutePoint]:
    if spacing_m <= 0.0 or len(points) < 2:
        return list(points)

    out = [points[0]]
    for a, b in zip(points, points[1:]):
        length = point_dist(a, b)
        if length < 1e-9:
            continue
        steps = max(1, int(math.ceil(length / spacing_m)))
        for step in range(1, steps + 1):
            t = step / steps
            out.append(
                RoutePoint(
                    x=a.x + (b.x - a.x) * t,
                    y=a.y + (b.y - a.y) * t,
                    is_headland=a.is_headland or b.is_headland,
                    is_reverse=a.is_reverse or b.is_reverse,
                )
            )
    return out


# =============================================================================
# WGS/ENU HELPERS
# =============================================================================

def wgs_to_enu_approx(lat: float, lon: float) -> tuple[float, float]:
    """Small-field WGS84 approximation: returns east,north meters from datum."""
    lat0 = math.radians(DATUM_LAT)
    north = (lat - DATUM_LAT) * 111_320.0
    east = (lon - DATUM_LON) * 111_320.0 * math.cos(lat0)
    return east, north


def route_to_anchor_waypoints(points: Sequence[RoutePoint], anchor_lat: float, anchor_lon: float) -> list[Waypoint]:
    anchor_e, anchor_n = wgs_to_enu_approx(anchor_lat, anchor_lon)
    out: list[Waypoint] = []
    for i, point in enumerate(points):
        out.append(
            Waypoint(
                index=i,
                x=point.x,
                y=point.y,
                east_cm=round((point.x - anchor_e) * 100.0),
                north_cm=round((point.y - anchor_n) * 100.0),
                is_headland=point.is_headland,
                is_reverse=point.is_reverse,
            )
        )
    return out


def point_inside_field(x: float, y: float) -> bool:
    polygon = field_boundary_enu()
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


# =============================================================================
# J1939 / PROTOCOL PACKING
# =============================================================================

def j1939_id(pgn: int, source: int = SOURCE_AUTODRIVE) -> int:
    return ((J1939_PRIORITY & 0x7) << 26) | ((pgn & 0x3FFFF) << 8) | (source & 0xFF)


def pgn_from_id(arbitration_id: int) -> int:
    return (arbitration_id >> 8) & 0x3FFFF


def unavailable_u32(raw: int) -> bool:
    return raw == 0xFFFFFFFF


def decode_latlon_u32(raw: int) -> float | None:
    if unavailable_u32(raw):
        return None
    return raw * 0.0000001 - 210.0


def decode_vp1(data: bytes, status: MachineStatus) -> None:
    if len(data) < 8:
        return
    lat_raw = struct.unpack_from("<I", data, 0)[0]
    lon_raw = struct.unpack_from("<I", data, 4)[0]
    status.gps_lat = decode_latlon_u32(lat_raw)
    status.gps_lon = decode_latlon_u32(lon_raw)
    status.last_rx_s = time.monotonic()


def decode_vds(data: bytes, status: MachineStatus) -> None:
    if len(data) < 8:
        return
    compass = struct.unpack_from("<H", data, 0)[0]
    speed = struct.unpack_from("<H", data, 2)[0]
    status.heading_deg = compass / 128.0
    status.speed_kph = speed / 256.0
    status.last_rx_s = time.monotonic()


def decode_dsap(data: bytes, status: MachineStatus) -> None:
    if len(data) < 8:
        return
    lat_raw = struct.unpack_from("<I", data, 0)[0]
    lon_raw = struct.unpack_from("<I", data, 4)[0]
    status.anchor_lat = decode_latlon_u32(lat_raw)
    status.anchor_lon = decode_latlon_u32(lon_raw)
    status.last_rx_s = time.monotonic()


def decode_dsstat(data: bytes, status: MachineStatus) -> None:
    if len(data) < 8:
        return
    b1 = data[0]
    b2 = data[1]
    status.gps_ppp_available = bool(b1 & 0x80)      # Byte1 bit8
    status.autosteer_engaged = bool(b1 & 0x20)      # Byte1 bit6
    status.header_down = bool(b1 & 0x08)            # Byte1 bit4
    status.current_direction_reverse = bool(b1 & 0x02)  # Byte1 bit2
    status.autodrive_allowed = bool(b2 & 0x01)      # Byte2 bit1
    status.reject_reason = (b2 >> 1) & 0x7F
    status.last_rx_s = time.monotonic()


def process_frame(frame: CanFrame, status: MachineStatus) -> None:
    pgn = pgn_from_id(frame.arbitration_id)
    if pgn == PGN_VP1:
        decode_vp1(frame.data, status)
    elif pgn == PGN_VDS:
        decode_vds(frame.data, status)
    elif pgn == PGN_DSSTAT:
        decode_dsstat(frame.data, status)
    elif pgn == PGN_DSAP:
        decode_dsap(frame.data, status)


def encode_adjob(system_active: bool, run_command: bool, current_index: int, total_points: int, error_code: int = 0) -> bytes:
    """
    ADJOB 0xFFCC:
    Byte1: reserved
    Byte2: error code in high nibble, RunCommand at bit4, SystemActive at bit1
    Byte3-4: current waypoint index
    Byte5-6: line total point count
    Byte7-8: job ID
    """
    b = bytearray(8)
    b[0] = 0
    b[1] = ((error_code & 0x0F) << 4) | (0x08 if run_command else 0) | (0x01 if system_active else 0)
    struct.pack_into("<H", b, 2, clamp_u16(current_index))
    struct.pack_into("<H", b, 4, clamp_u16(total_points))
    struct.pack_into("<H", b, 6, clamp_u16(JOB_ID))
    return bytes(b)


def encode_adwpi(point: Waypoint) -> bytes:
    """
    ADWPI 0xFFCD:
    Byte1-2: point index, 0 = first point
    Byte3-5.5: east coordinate to anchor, 20 bits, 1 cm/bit, offset -250000 cm
    Byte5.5-7: north coordinate to anchor, 20 bits, 1 cm/bit, offset -250000 cm
    Byte8: flags/reserved
    """
    east_raw = cm_to_adwpi_raw(point.east_cm)
    north_raw = cm_to_adwpi_raw(point.north_cm)
    flags = 0
    if point.is_headland:
        flags |= ADWPI_FLAG_HEADLAND
    if point.is_reverse:
        flags |= ADWPI_FLAG_REVERSE

    b = bytearray(8)
    struct.pack_into("<H", b, 0, clamp_u16(point.index))
    b[2] = east_raw & 0xFF
    b[3] = (east_raw >> 8) & 0xFF
    b[4] = ((east_raw >> 16) & 0x0F) | ((north_raw & 0x0F) << 4)
    b[5] = (north_raw >> 4) & 0xFF
    b[6] = (north_raw >> 12) & 0xFF
    b[7] = flags
    return bytes(b)


def cm_to_adwpi_raw(cm: int) -> int:
    raw = cm - ADWPI_COORD_OFFSET_CM
    return max(0, min(ADWPI_COORD_RAW_MAX, raw))


def clamp_u16(value: int) -> int:
    return max(0, min(0xFFFF, int(value)))


# =============================================================================
# CAN BACKENDS
# =============================================================================

class FakeBus:
    def send(self, frame: CanFrame) -> None:
        print_frame("TX", frame)

    def recv(self, timeout: float = 0.0) -> CanFrame | None:
        time.sleep(timeout)
        return None


class SocketCanBus:
    def __init__(self, channel: str):
        try:
            import can
        except Exception as exc:  # pragma: no cover - host dependent
            raise SystemExit("python-can missing. Enter nix develop shell.") from exc
        self.can = can
        self.bus = can.Bus(interface="socketcan", channel=channel)

    def send(self, frame: CanFrame) -> None:
        self.bus.send(
            self.can.Message(
                arbitration_id=frame.arbitration_id,
                data=frame.data,
                is_extended_id=True,
            )
        )

    def recv(self, timeout: float = 0.0) -> CanFrame | None:
        msg = self.bus.recv(timeout=timeout)
        if msg is None:
            return None
        return CanFrame(arbitration_id=msg.arbitration_id, data=bytes(msg.data))


def make_bus():
    if CAN_BACKEND == "fake":
        return FakeBus()
    if CAN_BACKEND == "socketcan":
        return SocketCanBus(CAN_CHANNEL)
    raise SystemExit(f"unknown CAN_BACKEND {CAN_BACKEND!r}")


def print_frame(direction: str, frame: CanFrame) -> None:
    data = " ".join(f"{x:02X}" for x in frame.data)
    print(f"{direction} 0x{frame.arbitration_id:08X} [{len(frame.data)}] {data}")


# =============================================================================
# LIVE AUTODRIVE LOOP
# =============================================================================

def ready_for_system_active(status: MachineStatus) -> bool:
    if REQUIRE_GPS_PPP and not status.gps_ppp_available:
        return False
    if REQUIRE_AUTODRIVE_ALLOWED and not status.autodrive_allowed:
        return False
    if REQUIRE_INSIDE_FIELD:
        if status.gps_lat is None or status.gps_lon is None:
            return False
        x, y = wgs_to_enu_approx(status.gps_lat, status.gps_lon)
        if not point_inside_field(x, y):
            return False
    return True


def anchor_is_valid(status: MachineStatus) -> bool:
    return status.anchor_lat is not None and status.anchor_lon is not None


def estimate_current_index(status: MachineStatus, waypoints: Sequence[Waypoint], previous: int) -> int:
    if status.gps_lat is None or status.gps_lon is None:
        return previous

    x, y = wgs_to_enu_approx(status.gps_lat, status.gps_lon)
    start = max(0, previous - NEAREST_SEARCH_BACKTRACK)
    end = min(len(waypoints), previous + NEAREST_SEARCH_AHEAD)
    if start >= end:
        return previous

    best = previous
    best_dist = float("inf")
    for i in range(start, end):
        d = math.hypot(waypoints[i].x - x, waypoints[i].y - y)
        if d < best_dist:
            best = i
            best_dist = d
    return max(previous, best)


def send_adjob(bus, system_active: bool, run_command: bool, current_index: int, total_points: int, error_code: int = 0) -> None:
    bus.send(
        CanFrame(
            arbitration_id=j1939_id(PGN_ADJOB),
            data=encode_adjob(system_active, run_command, current_index, total_points, error_code),
        )
    )


def drain_rx(bus, status: MachineStatus, max_frames: int = 50) -> None:
    for _ in range(max_frames):
        frame = bus.recv(timeout=0.0)
        if frame is None:
            return
        process_frame(frame, status)


def send_future_window(bus, waypoints: Sequence[Waypoint], current_index: int, status: MachineStatus) -> None:
    start = max(0, current_index - WINDOW_OVERLAP_POINTS)
    end = min(len(waypoints), current_index + FUTURE_POINT_COUNT)
    for point in waypoints[start:end]:
        drain_rx(bus, status, max_frames=5)
        bus.send(CanFrame(arbitration_id=j1939_id(PGN_ADWPI), data=encode_adwpi(point)))
        if SEND_INTERVAL_S > 0.0:
            time.sleep(SEND_INTERVAL_S)


def run() -> None:
    route_points = generate_route_points()
    bus = make_bus()
    status = MachineStatus()

    active = False
    run_command = False
    current_index = 0
    last_adjob_s = 0.0
    last_window_s = 0.0
    anchor_used: tuple[float, float] | None = None
    waypoints: list[Waypoint] = []

    print("live AutoDrive loop started", file=sys.stderr)

    while True:
        frame = bus.recv(timeout=0.02)
        if frame is not None:
            process_frame(frame, status)

        now = time.monotonic()
        should_activate = ready_for_system_active(status)
        active = should_activate

        if active and anchor_is_valid(status):
            anchor_pair = (status.anchor_lat, status.anchor_lon)
            if anchor_pair != anchor_used:
                anchor_used = anchor_pair
                waypoints = route_to_anchor_waypoints(route_points, status.anchor_lat, status.anchor_lon)
                current_index = 0
                last_window_s = 0.0
                print(
                    f"anchor received lat={status.anchor_lat:.8f} lon={status.anchor_lon:.8f}; "
                    f"prepared {len(waypoints)} ADWPI points",
                    file=sys.stderr,
                )

        if waypoints:
            current_index = estimate_current_index(status, waypoints, current_index)
            # Run after the first window has been sent at least once.
            can_send_window = active and anchor_used is not None
            if can_send_window and (now - last_window_s) >= RESEND_WINDOW_EVERY_S:
                send_future_window(bus, waypoints, current_index, status)
                last_window_s = time.monotonic()
                run_command = True

        # ADJOB every second or when active but still waiting for anchor.
        if (now - last_adjob_s) >= ADJOB_PERIOD_S:
            send_adjob(
                bus,
                system_active=active,
                run_command=active and run_command,
                current_index=current_index,
                total_points=len(waypoints) if waypoints else len(route_points),
                error_code=0,
            )
            last_adjob_s = now


if __name__ == "__main__":
    run()
