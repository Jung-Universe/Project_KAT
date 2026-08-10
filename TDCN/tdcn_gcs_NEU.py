#!/usr/bin/env python3
"""
TDCN GCS 모사 스크립트  /* Sejong */

SITL(또는 실기체)로 MAV_CMD_USER_1 (31010) 을 보내 TDCN 시나리오 상태와
타겟 정보를 전달한다.

대화형 모드는 키보드로 state 번호(1~11)를 받고, state 6(추종 비행)일 때만
타겟 정보 4개를 추가로 받아 함께 전송한다.

COMMAND_INT 필드 매핑  (COMMAND_LONG 이 아니다!)
------------------------------------------------
    frame   uint8   1 = MAV_FRAME_LOCAL_NED   x/y 가 home 기준 NEU 라는 선언
    param1  float   state              시나리오 상태 (1~11)
    param2  float   Target Heading     진북 기준 deg, state 6 에서만 유효
    param3  -       (예약, 0)
    param4  -       (예약, 0)
    x       int32   Target North       cm, home 기준, state 6 에서만 유효
    y       int32   Target East        cm, home 기준, state 6 에서만 유효
    z       float   Target Altitude    m, up-positive, home 기준

이 스크립트는 SITL 테스트용이다.  기체는 frame 을 보고 NEU 를 위경도로 바꿔
CLAW 와 위치제어에 넘긴다.  최종 결과물에서는 tdcn_gcs_Lat_Lon.py 를 쓰면 되고,
기체 코드는 바꿀 필요가 없다.

Heading 은 진북(True North) 기준 deg 이다.  이 스크립트는 입력값을 정규화하지
않고 그대로 전송한다 — 정규화(0~360 / ±180)나 rad 변환이 필요하면
Target.as_params() 한 곳만 고치면 된다.

사용 예
-------
    # 대화형 (기본)
    ./tdcn_gcs.py

    # 상태 하나만 보내고 종료
    ./tdcn_gcs.py --state 3

    # 타겟과 함께 추종 비행(state 6) 한 발
    ./tdcn_gcs.py --state 6 --north 12.5 --east -4.0 --heading 90 --alt 20

    # 위와 같으나 20초간 5Hz 로 스트리밍
    ./tdcn_gcs.py --state 6 --north 12.5 --alt 20 --track 20

    # 1~11 전체 시나리오 자동 재생
    ./tdcn_gcs.py --scenario

주의: ArduPilot 에 MAV_CMD_USER_1 핸들러가 추가되기 전에는 기체가
MAV_RESULT_UNSUPPORTED 로 응답한다.  이는 정상이며, 스크립트는 그 결과를
그대로 출력한다.
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import sys
import threading
import time
from dataclasses import dataclass, field

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink 이 필요합니다:  pip install pymavlink")


MAV_CMD_USER_1 = 31010

# COMMAND_INT 의 frame.  1 = 좌표가 home 기준 NEU 라는 선언.
# 최종 결과물에서 위경도로 보낼 때는 3 (MAV_FRAME_GLOBAL_RELATIVE_ALT) 을 쓴다.
MAV_FRAME_LOCAL_NED = 1

# 타겟 정보를 실을 수 있는 유일한 상태
TARGET_STATE = 6

# state 번호 -> (짧은 이름, 설명)
STATES: dict[int, tuple[str, str]] = {
    1:  ("HANGAR_OPEN",   "격납함 열기"),
    2:  ("TAKEOFF_WAIT",  "이륙 대기"),
    3:  ("ARMED",         "ARMED"),
    4:  ("LAUNCH",        "이륙 사출"),
    5:  ("FLIGHT_WAIT",   "비행 대기"),
    6:  ("TRACKING",      "추종 비행"),
    7:  ("LANDING_WAIT",  "착륙 대기"),
    8:  ("LANDING_SYNC",  "착륙 동기"),
    9:  ("LANDING_STOW",  "착륙 수납"),
    10: ("DISARMED",      "DISARMED"),
    11: ("HANGAR_CLOSE",  "격납함 닫기"),
}

MAV_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}


# ---------------------------------------------------------------------------
# 타겟
# ---------------------------------------------------------------------------

@dataclass
class Target:
    """추종 비행 타겟.

    North/East/Alt 는 m (Alt 는 up-positive), Heading 은 진북 기준 deg.
    """

    north: float = 0.0
    east: float = 0.0
    heading: float = 0.0
    alt: float = 0.0

    # 선박 운동 모사 (0 이면 정지 타겟)
    ship_speed: float = 0.0      # m/s
    ship_course: float = 0.0     # deg, heading 과 동일 기준

    def advance(self, dt: float) -> None:
        """선박 속도/침로에 따라 타겟 위치를 dt 초만큼 전진시킨다.

        heading 은 건드리지 않는다 — 선박의 침로(course)와 선수방위(heading)는
        별개이고, 사용자가 지정한 heading 을 조용히 덮어쓰지 않는다.
        """
        if self.ship_speed == 0.0:
            return
        rad = math.radians(self.ship_course)
        self.north += self.ship_speed * math.cos(rad) * dt
        self.east += self.ship_speed * math.sin(rad) * dt

    def as_command_int_fields(self) -> tuple[int, int, float, float]:
        """(x, y, z, heading) 로 변환.

        x/y 는 int32 cm 라서 1 cm 단위가 유지된다 (float 파라미터로 보내면
        기체 쪽에서 0.4~1.4 m 로 양자화된다).
        Heading 은 진북 기준 deg 를 그대로 넘긴다.
        """
        return (int(round(self.north * 100.0)),   # North cm
                int(round(self.east * 100.0)),    # East  cm
                float(self.alt),
                float(self.heading))

    def __str__(self) -> str:
        s = (f"N={self.north:+.2f}m E={self.east:+.2f}m "
             f"HDG={self.heading:.1f}deg ALT={self.alt:+.2f}m")
        if self.ship_speed:
            s += f"  (ship {self.ship_speed:.2f}m/s @ {self.ship_course:.1f}deg)"
        return s


# ---------------------------------------------------------------------------
# 시나리오
# ---------------------------------------------------------------------------

# state, dwell(초), 그리고 state 6 은 dwell 동안 stream
DEFAULT_SCENARIO: list[dict] = [
    {"state": 1,  "dwell": 3.0},
    {"state": 2,  "dwell": 3.0},
    {"state": 3,  "dwell": 2.0},
    {"state": 4,  "dwell": 3.0},
    {"state": 5,  "dwell": 3.0},
    {"state": 6,  "dwell": 30.0, "stream": True,
     "target": {"north": 0.0, "east": 0.0, "heading": 0.0, "alt": 20.0}},
    {"state": 7,  "dwell": 3.0},
    {"state": 8,  "dwell": 5.0},
    {"state": 9,  "dwell": 3.0},
    {"state": 10, "dwell": 2.0},
    {"state": 11, "dwell": 3.0},
]


# ---------------------------------------------------------------------------
# 링크
# ---------------------------------------------------------------------------

class TdcnGCS:
    def __init__(self, device: str, target_system: int = 0,
                 target_component: int = 1, source_system: int = 254,
                 ack_timeout: float = 2.0):
        print(f"[link] connecting to {device} ...")
        self.master = mavutil.mavlink_connection(device,
                                                source_system=source_system)
        hb = self.master.wait_heartbeat(timeout=30)
        if hb is None:
            raise SystemExit("[link] heartbeat 수신 실패")
        self.target_system = target_system or self.master.target_system
        self.target_component = target_component
        self.ack_timeout = ack_timeout
        print(f"[link] connected — sysid={self.target_system} "
              f"compid={self.target_component}")

        # 기체가 마지막으로 받아들인 state.  ACK 를 해석해 조작자에게
        # "다음은 몇 번" 을 알려주기 위해 GCS 쪽에서도 추적한다.
        # 기체는 모드 진입 시 NONE(0) 에서 시작한다.
        self.state: int = 0

        self._acks: queue.Queue = queue.Queue()
        self._latest: dict[str, object] = {}
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    # -- 수신 --------------------------------------------------------------

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self.master.recv_match(blocking=True, timeout=0.5)
            except Exception:
                continue
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == "COMMAND_ACK":
                self._acks.put(msg)
            elif mtype in ("HEARTBEAT", "STATUSTEXT"):
                self._latest[mtype] = msg
                if mtype == "STATUSTEXT":
                    # TDCN 이 보내는 진행 상황(예: prearm OK/FAIL)은 조작자가
                    # 다음 단계로 넘어가도 되는지 판단하는 근거라 강조해 둔다.
                    if "TDCN" in msg.text:
                        print(f"\n  [기체] {msg.text}")
                    else:
                        print(f"\n[vehicle] {msg.text}")

    def close(self) -> None:
        self._stop.set()
        self._reader.join(timeout=2.0)
        self.master.close()

    def status(self) -> str:
        hb = self._latest.get("HEARTBEAT")
        if hb is None:
            return "heartbeat 없음"
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return (f"custom_mode={hb.custom_mode} "
                f"armed={'YES' if armed else 'no'} "
                f"system_status={hb.system_status}")

    # -- 송신 --------------------------------------------------------------

    def send(self, state: int, target: Target | None = None,
             wait_ack: bool = True, quiet: bool = False) -> str | None:
        """MAV_CMD_USER_1 한 발 전송.  state 6 이 아니면 타겟 파라미터는 0."""
        if state not in STATES:
            raise ValueError(f"알 수 없는 state: {state} (1~11)")

        if state == TARGET_STATE:
            if target is None:
                raise ValueError(f"state {TARGET_STATE} 는 타겟 정보가 필요합니다")
            x, y, z, heading = target.as_command_int_fields()
        else:
            x = y = 0
            z = heading = 0.0

        # 오래된 ACK 를 버려 이전 명령의 응답을 오인하지 않도록 한다
        while not self._acks.empty():
            try:
                self._acks.get_nowait()
            except queue.Empty:
                break

        # COMMAND_LONG 이 아니라 COMMAND_INT 로 보낸다.
        # frame = MAV_FRAME_LOCAL_NED 로 "x/y 는 home 기준 NEU cm" 임을 선언하면,
        # 기체가 받는 즉시 위경도로 바꿔 CLAW 와 위치제어에 넘긴다.
        # 최종 결과물에서는 GCS 가 frame 을 3 으로 바꾸고 x/y 에 위경도(1e7 deg)를
        # 실으면 된다 - 기체 코드는 그대로다.
        self.master.mav.command_int_send(
            self.target_system,
            self.target_component,
            MAV_FRAME_LOCAL_NED,             # x/y = home 기준 NEU cm
            MAV_CMD_USER_1,
            0,              # current
            0,              # autocontinue
            float(state),   # param1  state
            heading,        # param2  Target Heading (deg, 진북)
            0.0,            # param3  예약
            0.0,            # param4  예약
            x,              # x       Target North (int32, cm)
            y,              # y       Target East  (int32, cm)
            z,              # z       Target Altitude (m, up)
        )

        if not quiet:
            name, desc = STATES[state]
            line = f"[tx] state={state:<2} {name:<13} ({desc})"
            if state == TARGET_STATE:
                line += f"  {target}"
            print(line)

        if not wait_ack:
            return None
        return self._wait_ack(sent_state=state, quiet=quiet)

    def send_until_ready(self, state: int, target: "Target | None" = None,
                         timeout: float = 60.0,
                         retry_period: float = 1.0) -> str | None:
        """state 를 보내고, 아직 완료 전이면 될 때까지 대신 재시도한다.

        기체는 순서가 맞아도 현재 단계가 끝나지 않았으면
        TEMPORARILY_REJECTED 로 돌려보낸다.  조작자가 그걸 손으로 반복해서
        누를 필요가 없도록 여기서 자동으로 다시 보낸다.

        멈추는 조건
          ACCEPTED  성공 - 다음 단계로 넘어갔다
          DENIED    순서 자체가 틀렸다 - 재시도해도 소용없으므로 즉시 중단
          timeout   그때까지 완료되지 않았다
        """
        result = self.send(state, target)
        if result != "TEMPORARILY_REJECTED":
            return result

        deadline = time.time() + timeout
        waited = 0.0
        while time.time() < deadline:
            time.sleep(retry_period)
            waited += retry_period

            # 재시도는 조용히 보내고, 결과가 바뀔 때만 출력한다
            result = self.send(state, target, quiet=True)
            if result == "ACCEPTED":
                name, desc = STATES.get(state, ("?", "?"))
                print(f"     -> OK   완료되어 state {state} {name} ({desc}) "
                      f"진입  ({waited:.0f}초 대기)")
                self.state = state
                return result
            if result == "DENIED":
                print(f"     -> 거부  순서 위반으로 바뀌었습니다.  중단합니다.")
                return result

            # 5초마다 한 번씩만 대기 중임을 알린다
            if int(waited) % 5 == 0:
                print(f"        ... 대기 중 ({waited:.0f}초)")

        print(f"     -> 시간 초과  {timeout:.0f}초 안에 완료되지 않았습니다.")
        return result

    def _wait_ack(self, sent_state: int = 0, quiet: bool = False) -> str | None:
        """ACK 를 받아 조작자가 바로 알아볼 수 있게 해석해 출력한다.

        기체는 state 순서를 단방향으로 강제한다.  거부 이유가 두 가지라서
        ACK 로 구분해 준다:

          DENIED                순서가 틀렸다 (건너뛰기 / 되돌아가기).
                                다시 보내도 소용없다.
          TEMPORARILY_REJECTED  다음 번호는 맞는데 현재 단계가 아직 안 끝났다.
                                잠시 뒤 다시 누르면 통과한다.
          ACCEPTED              정상 반영.
        """
        deadline = time.time() + self.ack_timeout
        while time.time() < deadline:
            try:
                ack = self._acks.get(timeout=0.1)
            except queue.Empty:
                continue
            if ack.command != MAV_CMD_USER_1:
                continue
            result = MAV_RESULT_NAMES.get(ack.result, str(ack.result))

            if not quiet:
                if ack.result == 0:                       # ACCEPTED
                    if sent_state == self.state:
                        print(f"     -> state {sent_state} 유지 중")
                    else:
                        self.state = sent_state
                        name, desc = STATES.get(sent_state, ("?", "?"))
                        print(f"     -> OK   state {sent_state} {name} ({desc}) 진입")
                elif ack.result == 1:                     # TEMPORARILY_REJECTED
                    cur = self.state
                    cname = STATES.get(cur, ("NONE", "명령 대기"))[0]
                    print(f"     -> 대기  state {cur}({cname}) 가 아직 완료되지 "
                          f"않았습니다.")
                    print(f"             잠시 뒤 state {sent_state} 를 다시 "
                          f"보내면 넘어갑니다.")
                elif ack.result == 2:                     # DENIED
                    cur = self.state
                    nxt = cur + 1
                    print(f"     -> 거부  순서 위반.  현재 state {cur} 이므로 "
                          f"다음은 state {nxt} 만 가능합니다.")
                else:
                    print(f"     -> {result}")
            return result

        if not quiet:
            print("     -> 응답 없음 (timeout)")
        return None

    def stream_target(self, target: Target, duration: float,
                      rate: float = 5.0) -> None:
        """state 6 을 duration 초간 rate Hz 로 반복 전송한다."""
        period = 1.0 / rate
        n = 0
        first_ack: str | None = None
        t_end = time.time() + duration
        t_prev = time.time()
        print(f"[stream] state {TARGET_STATE} @ {rate:g}Hz, {duration:g}s — "
              f"Ctrl-C 로 중단")
        try:
            while time.time() < t_end:
                now = time.time()
                target.advance(now - t_prev)
                t_prev = now

                # 첫 발만 ACK 를 확인하고, 이후에는 조용히 스트리밍한다
                if n == 0:
                    first_ack = self.send(target=target, state=TARGET_STATE)
                else:
                    self.send(state=TARGET_STATE, target=target,
                              wait_ack=False, quiet=True)
                n += 1
                if n % int(max(rate, 1)) == 0:
                    print(f"\r[stream] {n:5d} sent   {target}", end="", flush=True)
                time.sleep(max(0.0, period - (time.time() - now)))
        except KeyboardInterrupt:
            print("\n[stream] 사용자 중단")
        print(f"\n[stream] 종료 — {n} 발 전송, 첫 ACK = {first_ack}")

    # -- 시나리오 ----------------------------------------------------------

    def run_scenario(self, steps: list[dict], rate: float = 5.0) -> None:
        print(f"[scenario] {len(steps)} 단계 시작")
        for i, step in enumerate(steps, 1):
            state = int(step["state"])
            dwell = float(step.get("dwell", 2.0))
            name = STATES[state][0]
            print(f"\n--- [{i}/{len(steps)}] state {state} {name} "
                  f"(dwell {dwell:g}s) ---")

            if state == TARGET_STATE:
                tgt_cfg = step.get("target") or {}
                target = Target(**tgt_cfg)
                if step.get("stream", True):
                    # 스트리밍 전에 6번이 받아들여질 때까지 기다린다
                    self.send_until_ready(state, target, timeout=180.0)
                    self.stream_target(target, duration=dwell, rate=rate)
                    continue
                self.send_until_ready(state, target, timeout=180.0)
            else:
                # 앞 단계가 끝날 때까지 자동으로 재시도한다.  dwell 만 믿고
                # 다음 번호를 보내면 이륙(약 10초) 처럼 오래 걸리는 단계에서
                # TEMPORARILY_REJECTED 를 맞고 시나리오가 멈춘다.
                self.send_until_ready(state, timeout=180.0)

            time.sleep(dwell)
        print("\n[scenario] 완료")


# ---------------------------------------------------------------------------
# 대화형 입력 — state 번호를 받고, state 6 일 때만 타겟 4개를 추가로 받는다
# ---------------------------------------------------------------------------

class Quit(Exception):
    """사용자가 종료를 요청했다."""


def _ask(prompt: str) -> str:
    """한 줄 입력.  q / quit / exit / EOF / Ctrl-C 는 종료."""
    try:
        text = input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        print()
        raise Quit
    if text.lower() in ("q", "quit", "exit"):
        raise Quit
    return text


def print_states() -> None:
    for num, (name, desc) in STATES.items():
        mark = "   <-- 타겟 정보 4개 추가 입력" if num == TARGET_STATE else ""
        print(f"  {num:2d}  {name:<13} {desc}{mark}")


def ask_state() -> int:
    """state 번호(1~11)를 받는다.  빈 입력이면 목록을 다시 보여준다."""
    while True:
        text = _ask("\nstate (1-11, 엔터=목록, q=종료) > ")
        if not text:
            print_states()
            continue
        try:
            state = int(text)
        except ValueError:
            print("  숫자를 입력하세요.")
            continue
        if state not in STATES:
            print(f"  state 는 1~11 이어야 합니다 (받은 값: {state})")
            continue
        return state


def ask_float(label: str, previous: float) -> float:
    """실수 하나를 받는다.  빈 입력이면 이전 값을 유지한다."""
    while True:
        text = _ask(f"  {label} [{previous:+.2f}] > ")
        if not text:
            return previous
        try:
            return float(text)
        except ValueError:
            print("    숫자를 입력하세요.")


def ask_target(previous: Target) -> Target:
    """state 6 전용 — 타겟 4개 값을 받는다.  빈 입력은 이전 값 유지."""
    print("  타겟 정보 입력 (엔터 = 이전 값 유지)")
    return Target(
        north=ask_float("Target North    (m)      ", previous.north),
        east=ask_float("Target East     (m)      ", previous.east),
        heading=ask_float("Target Heading  (deg,진북)", previous.heading),
        alt=ask_float("Target Altitude (m, up)  ", previous.alt),
        ship_speed=previous.ship_speed,
        ship_course=previous.ship_course,
    )


def interactive(gcs: TdcnGCS) -> None:
    print_states()
    target = Target()
    while True:
        try:
            state = ask_state()
            if state == TARGET_STATE:
                target = ask_target(target)
                gcs.send_until_ready(state, target)
            else:
                gcs.send_until_ready(state)
        except Quit:
            print("종료합니다.")
            return


# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description="TDCN GCS 모사 — MAV_CMD_USER_1 로 상태/타겟 전송",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connect", "-c", default="tcp:127.0.0.1:5760",
                    help="MAVLink 접속 문자열 (기본: tcp:127.0.0.1:5760)")
    ap.add_argument("--sysid", type=int, default=0,
                    help="대상 system id (0 = heartbeat 에서 자동)")
    ap.add_argument("--compid", type=int, default=1, help="대상 component id")
    ap.add_argument("--source-system", type=int, default=254,
                    help="이 GCS 의 system id (기본 254)")
    ap.add_argument("--rate", type=float, default=5.0,
                    help="state 6 스트리밍 주기 Hz (기본 5)")
    ap.add_argument("--ack-timeout", type=float, default=2.0,
                    help="COMMAND_ACK 대기 시간 초 (기본 2)")

    ap.add_argument("--state", type=int,
                    help="이 상태를 한 번 보내고 종료 (비대화형)")
    ap.add_argument("--north", type=float, default=0.0, help="Target North (m)")
    ap.add_argument("--east", type=float, default=0.0, help="Target East (m)")
    ap.add_argument("--heading", type=float, default=0.0,
                    help="Target Heading (deg, 진북 기준)")
    ap.add_argument("--alt", type=float, default=0.0,
                    help="Target Altitude (m, up)")
    ap.add_argument("--track", type=float, metavar="SEC",
                    help="--state 6 과 함께 쓰면 SEC 초간 스트리밍")
    ap.add_argument("--ship-speed", type=float, default=0.0,
                    help="선박 속도 m/s (스트리밍 중 타겟 이동)")
    ap.add_argument("--ship-course", type=float, default=0.0,
                    help="선박 침로 deg")

    ap.add_argument("--scenario", nargs="?", const="", metavar="FILE",
                    help="시나리오 재생 후 종료 (FILE 생략 시 내장 기본값)")
    ap.add_argument("--list-states", action="store_true",
                    help="state 목록만 출력하고 종료")
    args = ap.parse_args()

    if args.list_states:
        print_states()
        return 0

    if args.state is not None and args.state not in STATES:
        ap.error(f"--state 는 1~11 이어야 합니다 (받은 값: {args.state})")

    gcs = TdcnGCS(args.connect, target_system=args.sysid,
                  target_component=args.compid,
                  source_system=args.source_system,
                  ack_timeout=args.ack_timeout)
    try:
        if args.scenario is not None:
            if args.scenario:
                with open(args.scenario, encoding="utf-8") as fh:
                    steps = json.load(fh)
            else:
                steps = DEFAULT_SCENARIO
            gcs.run_scenario(steps, rate=args.rate)

        elif args.state is not None:
            target = Target(north=args.north, east=args.east,
                            heading=args.heading, alt=args.alt,
                            ship_speed=args.ship_speed,
                            ship_course=args.ship_course)
            if args.track and args.state == TARGET_STATE:
                gcs.stream_target(target, duration=args.track, rate=args.rate)
            elif args.state == TARGET_STATE:
                gcs.send(args.state, target)
            else:
                gcs.send(args.state)

        else:
            interactive(gcs)
    finally:
        gcs.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
