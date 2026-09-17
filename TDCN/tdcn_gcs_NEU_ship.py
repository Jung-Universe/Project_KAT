#!/usr/bin/env python3
"""
TDCN state 6 — 선박 탑재 GCS 모사  /* Sejong */

실제 운용에서 타겟값 4개는 사람이 입력하는 값이 아니다.
**GCS 는 배에 탑재되어 있고, 배의 상태를 텔레메트리로 드론에 계속 흘린다.**

    배(GCS)  ──MAVLink/텔레메트리──>  드론
       N, E      배의 현재 위치 (m, home 기준)
       Heading   배의 진행방향 (deg, 진북) — 계산해서 채운다
       Alt       고도 오프셋 (m)

드론은 그 배를 추종한다.  이 스크립트는 그 GCS 를 모사한다 —
state 6 에서 tdcn_gcs_NEU.py 가 손으로 받던 타겟 4개를, 배의 운동에서
자동으로 만들어 주기적으로 보낸다.

링크 / ACK / 전송 / 입력 로직은 tdcn_gcs_NEU.py 를 import 해서 쓴다.
**원본 스크립트는 건드리지 않는다.**


주기가 둘이다
-------------
배가 자기 상태를 **계산**하는 주기와, 그것을 텔레메트리로 **전송**하는 주기는
다르다.  둘을 따로 둔다.

    SHIP_UPDATE_RATE_HZ   배가 자기 위치/헤딩을 갱신하는 주기.
                          웨이포인트 사이를 이 주기로 쪼갠다 (궤적 해상도)
    TELEM_SEND_RATE_HZ    그 값을 FC 로 보내는 주기

구현은 스레드를 둘로 나누지 않는다.  전송 시점마다 "그때까지 갱신된 최신
상태" 를 고르면 두 주기의 관계가 그대로 재현된다.

    t = now - t0
    k = floor(t * SHIP_UPDATE_RATE_HZ)      마지막으로 갱신된 스텝
    상태 = ship.at(k / SHIP_UPDATE_RATE_HZ)

그래서

    갱신 > 전송    계산된 값 중 일부만 나간다 (중간값은 솎인다)
    갱신 < 전송    새 값이 나올 때까지 같은 값을 반복 전송한다 (ZOH)
    갱신 = 전송    계산되는 족족 하나씩 나간다


전제 (사용자 확인)
------------------
* Heading 은 배의 **진행방향(침로)** 이다.  독립 값이 아니라 구간 방향에서
  나온다.  진북이 0 이고 그것이 N 축이므로, 첫 구간이 N+ 면 출발 헤딩이 0 이다.
  꺾이는 지점에서 헤딩은 순간 바뀐다 — 둥글리지 않는다.

* Alt 는 배 고도 기준 **오프셋** 이다.  드론이 배에서 이륙하므로 드론의 home z
  가 곧 배의 z 이고, 따라서 오프셋을 그대로 목표 고도로 보내면 된다.
  펌웨어도 packet.z 를 그대로 고도로 쓴다 (mode_tdcn.cpp 의 set_alt_cm).

* 궤적은 **한 번만** 지난다.  순회하지 않는다.  마지막 위치에 닿으면 그 값을
  계속 보내 그 자리에 호버시킨다.  조작자가 다음 state 를 넣을 때까지.

* state 6 실행 중 아무 때나 7 을 넣으면 그 자리에서 끊고 전환한다.


재전송을 하지 않는 이유
-----------------------
실제 GCS 는 배의 현재 상태를 주기적으로 흘릴 뿐, 놓친 값을 다시 보내지 않는다.
다음 발이 항상 더 새로운 정보를 담고 있기 때문이다.  이 스크립트도 같다.
(state 전환만 ACK 까지 재시도한다 — 원본의 send_until_ready 가 한다.)


사용 예
-------
    ./tdcn_gcs_NEU_ship.py                     # 플롯 -> 접속 -> CLI
    ./tdcn_gcs_NEU_ship.py --preview           # 접속 없이 궤적/전송값 확인
    ./tdcn_gcs_NEU_ship.py --send-rate 2       # 텔레메트리 2Hz
    ./tdcn_gcs_NEU_ship.py --update-rate 1     # 배 상태 갱신 1Hz
    ./tdcn_gcs_NEU_ship.py --ship-speed 0.5 --alt 5
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


# ===========================================================================
#  설정 — 여기만 고치면 된다 (CLI 인자로도 덮어쓸 수 있다)
# ===========================================================================

#: 배가 지나는 궤적.  (dNorth, dEast) 숫자쌍, m.
#: **직전 위치로부터의 변위** 다 (절대 좌표가 아니다).  순서대로 누적된다.
#: 두 값을 동시에 주면 대각선으로 간다 — 예: (3, 3) 이면 북동쪽 45도로 4.24m.
traj_NE = [
    (+10.0,   0.0),     
    (  0.0, +10.0),     
    (-10.0,   0.0),    
    (  0.0, -10.0),     
]

#: 고도 오프셋 (m).  궤적 내내 일정하다.
traj_U = 10.0

#: 배의 속도 (m/s).  웨이포인트 사이를 이 속도로 지난다.
SHIP_SPEED_MPS = 1.0

#: 배가 자기 상태(위치/헤딩)를 갱신하는 주기 (Hz).  궤적 해상도.
SHIP_UPDATE_RATE_HZ = 10.0

#: 텔레메트리로 FC 에 타겟을 보내는 주기 (Hz).
TELEM_SEND_RATE_HZ = 1.0

#: 배의 출발 위치 (m, home 기준)
SHIP_START_NORTH = 0.0
SHIP_START_EAST = 0.0

# ===========================================================================


from tdcn_gcs_NEU import (        # noqa: E402
    STATES,
    TARGET_STATE,
    Quit,
    Target,
    TdcnGCS,
    _ask,
)


# ---------------------------------------------------------------------------
# 배의 운동
# ---------------------------------------------------------------------------

class Ship:
    """웨이포인트 사이를 등속으로 지나는 배.

    traj 는 (dNorth, dEast) 변위의 목록이다.  두 값이 동시에 0 이 아니면
    대각선 구간이 된다.

    at(t) 가 그 시각의 (North, East, Heading) 을 준다.  Heading 은 그 구간의
    진행방향이고, 구간이 바뀌는 지점에서 순간 바뀐다.
    t 가 궤적 소요시간을 넘으면 마지막 위치에 멈춰 있는 것으로 본다.
    """

    def __init__(self, traj: list[tuple[float, float]], speed: float,
                 north: float = 0.0, east: float = 0.0):
        if speed <= 0.0:
            raise ValueError(f"배 속도는 양수여야 합니다 (받은 값: {speed})")
        if not traj:
            raise ValueError("궤적이 비어 있습니다")

        self.traj = [(float(dn), float(de)) for dn, de in traj]
        self.speed = float(speed)
        self.n0 = float(north)
        self.e0 = float(east)

        # 웨이포인트(구간 끝점)를 누적으로 만든다.  points[0] 이 출발점.
        self.points: list[tuple[float, float]] = [(self.n0, self.e0)]
        n, e = self.n0, self.e0
        for dn, de in self.traj:
            n += dn
            e += de
            self.points.append((n, e))

        self.seg_len = [math.hypot(dn, de) for dn, de in self.traj]
        for i, L in enumerate(self.seg_len):
            if L <= 0.0:
                raise ValueError(f"{i + 1}번째 구간의 길이가 0 입니다: "
                                 f"{self.traj[i]}")

        # 구간별 진행방향 (deg, 진북 0, 동쪽 90)
        self.seg_heading = [math.degrees(math.atan2(de, dn)) % 360.0
                            for dn, de in self.traj]

        # 누적 거리
        self.cum = [0.0]
        for L in self.seg_len:
            self.cum.append(self.cum[-1] + L)

    # -- 파생값 ------------------------------------------------------------

    @property
    def length(self) -> float:
        """궤적 전체 길이 (m)."""
        return self.cum[-1]

    @property
    def duration(self) -> float:
        """궤적을 다 지나는 데 걸리는 시간 (s)."""
        return self.length / self.speed

    @property
    def closed(self) -> bool:
        """마지막 점이 출발점과 같은가."""
        return math.hypot(self.points[-1][0] - self.n0,
                          self.points[-1][1] - self.e0) < 1e-9

    def describe(self) -> str:
        """사람이 읽을 구간 설명."""
        out = []
        for (dn, de), L, h in zip(self.traj, self.seg_len, self.seg_heading):
            if dn and de:
                out.append(f"(N{dn:+g},E{de:+g}) {L:.2f}m @{h:.0f}deg")
            elif de:
                out.append(f"E{de:+g}")
            else:
                out.append(f"N{dn:+g}")
        return " -> ".join(out)

    # -- 상태 ---------------------------------------------------------------

    def at(self, t: float) -> tuple[float, float, float]:
        """시각 t (s) 의 (North, East, Heading).

        t <= 0        출발점, 첫 구간 방향
        0 < t < dur   그 시각의 위치, 그 구간 방향
        t >= dur      마지막 점에 정지, 마지막 구간 방향
        """
        if t <= 0.0:
            return (self.n0, self.e0, self.seg_heading[0])
        if t >= self.duration:
            n, e = self.points[-1]
            return (n, e, self.seg_heading[-1])

        s = self.speed * t
        for i, L in enumerate(self.seg_len):
            if s <= self.cum[i + 1]:
                u = (s - self.cum[i]) / L
                n0, e0 = self.points[i]
                n1, e1 = self.points[i + 1]
                return (n0 + (n1 - n0) * u,
                        e0 + (e1 - e0) * u,
                        self.seg_heading[i])
        n, e = self.points[-1]
        return (n, e, self.seg_heading[-1])

    def state_at(self, t: float, update_rate: float
                 ) -> tuple[float, float, float, int]:
        """전송 시점 t 에 GCS 가 들고 있는 **최신 갱신 상태**.

        배는 update_rate 로만 자기 상태를 갱신하므로, 그 사이에 전송이 일어나면
        직전에 갱신된 값이 나간다 (zero-order hold).  이 함수가 그 규칙이다.

        돌려주는 마지막 값은 갱신 스텝 번호 k 다 — 같은 k 가 연달아 나오면
        그 구간에서 같은 값이 반복 전송됐다는 뜻이다.
        """
        k = int(math.floor(max(t, 0.0) * update_rate))
        t_state = k / update_rate
        # 배가 이미 멈췄으면 그 상태로 고정된다
        t_state = min(t_state, self.duration)
        n, e, h = self.at(t_state)
        return (n, e, h, k)

    def target_at(self, t: float, update_rate: float,
                  alt_offset: float) -> tuple[Target, int]:
        """전송 시점 t 에 보낼 타겟 4개와 그때의 갱신 스텝 번호."""
        n, e, h, k = self.state_at(t, update_rate)
        return (Target(north=n, east=e, heading=h, alt=alt_offset), k)

    def samples(self, n: int = 800):
        """플롯용 — 배의 연속 궤적 (t, N, E, Hdg)."""
        ts, ns, es, hs = [], [], [], []
        for i in range(n + 1):
            t = self.duration * i / n
            north, east, hdg = self.at(t)
            ts.append(t); ns.append(north); es.append(east); hs.append(hdg)
        return ts, ns, es, hs


def plan_messages(ship: Ship, update_rate: float, send_rate: float,
                  alt: float, extra: float = 0.0):
    """실제로 나갈 메시지를 미리 계산한다 (표/플롯/검산용).

    extra 는 도착 뒤 호버 구간을 몇 초까지 그릴지.
    """
    period = 1.0 / send_rate
    total = ship.duration + extra
    out = []
    i = 0
    while True:
        t = i * period
        if t > total + 1e-9:
            break
        n, e, h, k = ship.state_at(t, update_rate)
        out.append((i, t, n, e, h, alt, k))
        i += 1
    return out


# ---------------------------------------------------------------------------
# 출력
# ---------------------------------------------------------------------------

def print_plan(ship: Ship, update_rate: float, send_rate: float,
               alt: float) -> None:
    n1, e1, h1 = ship.at(ship.duration)
    msgs = plan_messages(ship, update_rate, send_rate, alt)
    n_state = int(math.floor(ship.duration * update_rate)) + 1
    ratio = update_rate / send_rate

    print("-" * 74)
    print(f"  배 출발     N={ship.n0:+.2f}m  E={ship.e0:+.2f}m  "
          f"HDG={ship.seg_heading[0]:.1f}deg")
    print(f"  배 도착     N={n1:+.2f}m  E={e1:+.2f}m  HDG={h1:.1f}deg"
          f"{'   (출발점과 동일)' if ship.closed else ''}")
    print(f"  궤적        {ship.describe()}")
    print(f"  웨이포인트  " + " -> ".join(f"({n:+.1f},{e:+.1f})"
                                          for n, e in ship.points))
    print(f"  구간 헤딩   " + " -> ".join(f"{h:.0f}deg"
                                          for h in ship.seg_heading))
    print(f"  전체        {ship.length:.2f}m @ {ship.speed:g}m/s "
          f"= {ship.duration:.2f}s  (한 번만, 반복 없음)")
    print("-" * 74)
    print(f"  고도 오프셋 {alt:+.2f}m  (배 기준 = home 기준, 내내 일정)")
    print(f"  배 갱신     {update_rate:g}Hz  ->  "
          f"{ship.speed / update_rate:.3f}m 마다 상태 계산, "
          f"총 {n_state}스텝")
    print(f"  전송        {send_rate:g}Hz  ->  "
          f"궤적 동안 {len(msgs)}발")
    if ratio > 1.0:
        print(f"  관계        갱신이 전송보다 {ratio:g}배 빠름 — "
              f"계산된 상태 {ratio:g}개 중 1개만 나갑니다")
    elif ratio < 1.0:
        print(f"  관계        전송이 갱신보다 {1 / ratio:g}배 빠름 — "
              f"같은 값이 {1 / ratio:g}번씩 반복 전송됩니다 (ZOH)")
    else:
        print(f"  관계        갱신 = 전송 — 계산되는 족족 하나씩 나갑니다")
    print(f"  도착 후     마지막 값을 계속 전송 (다음 state 입력 시 중지)")
    print("-" * 74)


def print_table(ship: Ship, update_rate: float, send_rate: float,
                alt: float, max_rows: int = 26) -> None:
    """실제로 나갈 메시지 목록.  k 가 갱신 스텝 번호다."""
    msgs = plan_messages(ship, update_rate, send_rate, alt,
                         extra=2.0 / send_rate)
    print(f"\n{'#':>4} {'t(s)':>7} {'North':>8} {'East':>8} "
          f"{'HDG':>7} {'Alt':>7} {'k':>5}   비고")
    print("-" * 70)

    def row(m) -> None:
        i, t, n, e, h, a, k = m
        note = ""
        if i == 0:
            note = "출발"
        elif t > ship.duration + 1e-9:
            note = "도착 후 호버"
        print(f"{i:4d} {t:7.2f} {n:+8.2f} {e:+8.2f} "
              f"{h:7.1f} {a:+7.2f} {k:5d}   {note}")

    if len(msgs) <= max_rows:
        for m in msgs:
            row(m)
    else:
        half = max_rows // 2
        for m in msgs[:half]:
            row(m)
        print(f"{'...':>4}   ({len(msgs) - max_rows}개 생략)")
        for m in msgs[-half:]:
            row(m)
    print("-" * 70)
    print(f"궤적 {ship.duration:.2f}s 동안 "
          f"{sum(1 for m in msgs if m[1] <= ship.duration + 1e-9)}발 "
          f"(k = 배 갱신 스텝 번호. 같은 k 가 연달면 같은 값 반복)")


# ---------------------------------------------------------------------------
# 플롯
# ---------------------------------------------------------------------------

def _setup_mpl():
    import matplotlib.pyplot as plt
    from matplotlib import font_manager as fm
    for cand in ("NanumGothic", "Malgun Gothic", "Noto Sans CJK KR",
                 "Noto Sans CJK JP"):
        if any(f.name == cand for f in fm.fontManager.ttflist):
            plt.rcParams["font.family"] = cand
            break
    plt.rcParams["axes.unicode_minus"] = False
    return plt


def _save_name(save: str, k: int) -> str:
    root, ext = os.path.splitext(save)
    return f"{root}_{k}{ext or '.png'}"


def _update_points(ship: Ship, update_rate: float):
    """배가 자기 상태를 계산한 시점들 — (t, N, E, Hdg)."""
    n_step = int(math.floor(ship.duration * update_rate))
    ts, ns, es, hs = [], [], [], []
    for k in range(n_step + 1):
        t = min(k / update_rate, ship.duration)
        n, e, h = ship.at(t)
        ts.append(t); ns.append(n); es.append(e); hs.append(h)
    if ts[-1] < ship.duration - 1e-9:      # 끝점을 반드시 포함
        n, e, h = ship.at(ship.duration)
        ts.append(ship.duration); ns.append(n); es.append(e); hs.append(h)
    return ts, ns, es, hs


def plot_path(ship: Ship, update_rate: float, send_rate: float,
              alt: float, save: str | None = None) -> None:
    """Figure 1 — 설정한 웨이포인트 기반 2D 궤적.

    나중에 로깅된 드론 위치를 이 위에 겹쳐 비교할 기준 경로다.

    경로를 **시간으로 색칠한다.**  왕복 궤적(예: N+10 -> E+10 -> E-10 -> N-10)
    은 갈 때와 올 때가 같은 선 위에 겹쳐서, 단색으로 그리면 어느 쪽이 먼저인지
    구분되지 않는다.  색이 곧 경과시간이므로 겹친 구간도 순서가 읽힌다.

    Heading 은 여기 그리지 않는다.  왕복 구간에서 화살표가 양방향으로 겹쳐
    오히려 헷갈리기 때문이다.  Heading 은 Figure 2 에 전용 축이 있다.
    """
    try:
        plt = _setup_mpl()
        from matplotlib.collections import LineCollection
        import numpy as np
    except Exception as exc:
        print(f"[plot] matplotlib 을 쓸 수 없어 건너뜁니다: {exc}")
        return

    ts, ns, es, hs = ship.samples()
    msgs = plan_messages(ship, update_rate, send_rate, alt)
    mt = [m[1] for m in msgs]
    mn = [m[2] for m in msgs]
    me = [m[3] for m in msgs]
    mh = [m[4] for m in msgs]

    fig, ax = plt.subplots(figsize=(8.6, 7.6))
    fig.suptitle(f"[Figure 1] 설정 웨이포인트 기반 2D 궤적  —  "
                 f"{ship.length:.1f}m @ {ship.speed:g}m/s = {ship.duration:.1f}s",
                 fontsize=12)

    # --- 기준 경로 : 시간으로 색칠 (겹치는 왕복 구간을 구분하기 위해) ---
    pts = np.array([es, ns]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="viridis", linewidths=3.0, alpha=0.9,
                        zorder=2)
    lc.set_array(np.array(ts[:-1]))
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, pad=0.02, fraction=0.045)
    cb.set_label("경과 시간 (s)", fontsize=9)

    # --- 웨이포인트 : 겹치는 점은 번호를 합쳐 표시한다 ---
    seen: dict[tuple[float, float], list[int]] = {}
    for i, (n, e) in enumerate(ship.points):
        seen.setdefault((round(n, 6), round(e, 6)), []).append(i)
    pn = [p[0] for p in ship.points]
    pe = [p[1] for p in ship.points]
    ax.plot(pe, pn, "s", color="#222222", ms=12, mfc="none", mew=1.8,
            label=f"설정한 웨이포인트 (traj_NE, {len(ship.points)}개)", zorder=6)
    for (n, e), idxs in seen.items():
        ax.annotate(",".join(str(k) for k in idxs), (e, n),
                    textcoords="offset points", xytext=(11, 8),
                    fontsize=10, color="#222222", zorder=7,
                    fontweight="bold")

    ax.plot(ship.e0, ship.n0, "o", color="#2ca02c", ms=13,
            label="출발점 (t=0)", zorder=8)
    ax.plot(ship.points[-1][1], ship.points[-1][0], "X", color="#d62728",
            ms=12, label=f"도착점 (t={ship.duration:.0f}s, 이후 계속 호버)",
            zorder=8)

    # --- 전송 지점 : 드론이 실제로 받은 타겟 ---
    ax.plot(me, mn, "o", color="#d62728", ms=4.5, alpha=0.9,
            label=f"텔레메트리로 실제 보낸 타겟 N/E "
                  f"({send_rate:g}Hz, {len(msgs)}발)", zorder=5)

    ax.set_aspect("equal", adjustable="datalim")
    ax.margins(0.12)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(f"선 = 배가 지나는 연속 경로 (색 = 경과시간, 겹친 왕복 구간 "
                 f"구분용)   |   Heading 은 Figure 2 참조", fontsize=9)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    if save:
        out = _save_name(save, 1)
        fig.savefig(out, dpi=130)
        print(f"[plot] 저장: {out}")
        plt.close(fig)
        return
    print("[plot] Figure 1 — 창을 닫으면 다음으로 넘어갑니다 ...")
    plt.show()


def plot_targets(ship: Ship, update_rate: float, send_rate: float,
                 alt: float, save: str | None = None) -> None:
    """Figure 2 — N / E / Heading / U 를 축별로 4x1.

    x 축은 시간, y 축은 그 축의 명령값.  점이 두 가지다.

        작은 점   배가 상태를 **계산한** 시점 (SHIP_UPDATE_RATE_HZ)
        큰 점     그중 실제로 **전송된** 값 (TELEM_SEND_RATE_HZ)

    둘을 같이 보면 두 주기의 관계가 그대로 드러난다 — 계산은 촘촘한데
    전송이 성글면 큰 점이 작은 점을 띄엄띄엄 집어간다.
    """
    try:
        plt = _setup_mpl()
    except Exception as exc:
        print(f"[plot] matplotlib 을 쓸 수 없어 건너뜁니다: {exc}")
        return

    ut, un, ue, uh = _update_points(ship, update_rate)
    ua = [alt] * len(ut)

    extra = 3.0 / send_rate
    msgs = plan_messages(ship, update_rate, send_rate, alt, extra=extra)
    mt = [m[1] for m in msgs]
    mn = [m[2] for m in msgs]
    me = [m[3] for m in msgs]
    mh = [m[4] for m in msgs]
    ma = [m[5] for m in msgs]

    # 순서: N, E, Heading, U
    rows = (
        ("North (m)",          un, mn, "#1f77b4"),
        ("East (m)",           ue, me, "#9467bd"),
        ("Heading (deg, 진북)", uh, mh, "#ff7f0e"),
        ("Up / Alt 오프셋 (m)", ua, ma, "#2ca02c"),
    )

    fig, axes = plt.subplots(4, 1, figsize=(12, 10.5), sharex=True)
    fig.suptitle(f"[Figure 2] 축별 명령값  —  배 상태 계산 {update_rate:g}Hz "
                 f"({len(ut)}개) / 텔레메트리 전송 {send_rate:g}Hz "
                 f"({sum(1 for t in mt if t <= ship.duration + 1e-9)}개)",
                 fontsize=12)

    small = 3 if len(ut) <= 500 else 1.6
    big = 6 if len(mt) <= 80 else 3

    for ax, (label, uy, my, color) in zip(axes, rows):
        # 배가 계산한 상태 — 촘촘한 작은 점
        ax.plot(ut, uy, ".", color=color, ms=small, alpha=0.45,
                label=f"배 계산 ({update_rate:g}Hz)", zorder=2)
        # 실제 전송값 — 계단 + 큰 점
        ax.step(mt, my, where="post", color=color, lw=1.6, alpha=0.9,
                label=f"전송 ({send_rate:g}Hz)", zorder=3)
        ax.plot(mt, my, "o", color=color, ms=big, mfc="white", mew=1.4,
                zorder=4)

        ax.axvline(ship.duration, color="#888888", lw=1.0, ls="--", zorder=0)
        ax.set_ylabel(label, fontsize=9)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")

    # 궤적 끝 표시는 세로 점선과 x축 라벨로 충분하다.  맨 위 패널에 주석을
    # 달면 범례와 겹친다.
    axes[-1].set_xlabel("t (s)   — 점선 = 궤적 끝, 이후는 마지막 값 계속 전송")
    fig.tight_layout(rect=(0, 0, 1, 0.95))

    if save:
        out = _save_name(save, 2)
        fig.savefig(out, dpi=130)
        print(f"[plot] 저장: {out}")
        plt.close(fig)
        return
    print("[plot] Figure 2 — 창을 닫으면 다음으로 넘어갑니다 ...")
    plt.show()


def plot_all(ship: Ship, update_rate: float, send_rate: float,
             alt: float, save: str | None = None) -> None:
    plot_path(ship, update_rate, send_rate, alt, save=save)
    plot_targets(ship, update_rate, send_rate, alt, save=save)


# ---------------------------------------------------------------------------
# 전송
# ---------------------------------------------------------------------------

class ShipFeed:
    """배의 상태를 텔레메트리 주기로 흘리는 백그라운드 전송기.

    실제 GCS 가 하는 일과 같다 — 배의 "지금" 상태를 계속 보낸다.  놓친 값은
    다시 보내지 않는다 (다음 발이 더 새로운 정보를 담고 있다).

    별도 스레드인 이유는 input() 이 메인 스레드를 막기 때문이다.  전송 중에도
    조작자가 7 을 넣을 수 있어야 한다.
    stop() 은 **스레드가 완전히 끝난 뒤 돌아온다** — 그래야 다음 state 전송이
    이 스레드의 state 6 뒤에 놓이는 것이 보장된다.
    """

    def __init__(self, gcs: TdcnGCS, ship: Ship, update_rate: float,
                 send_rate: float, alt_offset: float):
        self._gcs = gcs
        self._ship = ship
        self._update_rate = update_rate
        self._period = 1.0 / send_rate
        self._alt = alt_offset
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.sent = 0
        self.arrived = False

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        self._stop.clear()
        self.sent = 0
        self.arrived = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[feed] 전송 시작 — 배 갱신 {self._update_rate:g}Hz / "
              f"전송 {1 / self._period:g}Hz, 궤적 {self._ship.duration:.1f}s")
        print(f"[feed] 전송 중에도 입력을 받습니다. "
              f"7 을 넣으면 그 자리에서 끊고 전환합니다.")

    def _loop(self) -> None:
        t0 = time.time()
        last_report = -1.0
        while not self._stop.is_set():
            now = time.time()
            t = now - t0

            if t >= self._ship.duration and not self.arrived:
                self.arrived = True
                n, e, h = self._ship.at(self._ship.duration)
                print(f"\n[feed] 궤적 끝 도착 "
                      f"(N={n:+.2f} E={e:+.2f} HDG={h:.1f}) — "
                      f"{self.sent}발 전송")
                print(f"[feed] 이 값을 계속 보내 그 자리에 호버시킵니다. "
                      f"다음 state 를 입력하세요.")

            target, _k = self._ship.target_at(t, self._update_rate, self._alt)
            try:
                self._gcs.send(state=TARGET_STATE, target=target,
                               wait_ack=False, quiet=True)
                self.sent += 1
            except Exception:
                pass        # 링크가 끊겨도 CLI 는 살아 있어야 한다

            if not self.arrived and t - last_report >= 1.0:
                last_report = t
                print(f"\r[feed] {t:5.1f}/{self._ship.duration:.1f}s  "
                      f"{self.sent:4d}발  {target}", end="", flush=True)

            self._stop.wait(max(0.0, self._period - (time.time() - now)))

    def stop(self) -> None:
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            print("[warn] 전송 스레드가 아직 살아 있습니다 — "
                  "다음 명령과 겹칠 수 있습니다")
        self._thread = None
        print(f"\n[feed] 중지 ({self.sent}발 보냈습니다)")


# ---------------------------------------------------------------------------
# 대화형 CLI
# ---------------------------------------------------------------------------

def print_menu(ship: Ship, update_rate: float, send_rate: float,
               alt: float) -> None:
    print()
    for num, (name, desc) in STATES.items():
        mark = ""
        if num == TARGET_STATE:
            mark = f"   <-- 배 상태 자동 전송 ({ship.duration:.1f}s)"
        print(f"  {num:2d}  {name:<13} {desc}{mark}")
    print(f"\n  * state {TARGET_STATE} 을 한 번 누르면 끝까지 자동입니다 —")
    print(f"    타겟 입력 없이 배의 N/E/Heading + 고도 {alt:+.1f}m 를 "
          f"{send_rate:g}Hz 로 보냅니다.")
    print(f"    궤적을 한 번만 지나고, 끝나면 그 자리 값을 계속 보냅니다.")
    print(f"    실행 중 아무 때나 7 을 넣으면 바로 전환합니다.")
    print(f"\n  p   플롯 보기")
    print(f"  l   전송할 값 목록")
    print(f"  q   종료")


def interactive(gcs: TdcnGCS, ship: Ship, update_rate: float,
                send_rate: float, alt: float,
                plot_save: str | None) -> None:
    feed: ShipFeed | None = None
    print_menu(ship, update_rate, send_rate, alt)
    while True:
        try:
            text = _ask("\nstate (1-11, p=플롯, l=목록, 엔터=메뉴, q=종료) > ")

            # 무엇을 하든 전송을 먼저 멈춘다.  전송 스레드가 state 6 을 계속
            # 쏘고 있으면 다음 state 전환과 충돌한다.  stop() 은 스레드가
            # 끝난 뒤 돌아오므로, 아래 전송은 반드시 그 뒤에 놓인다.
            if text and feed is not None and feed.running:
                feed.stop()
                feed = None

            if not text:
                print_menu(ship, update_rate, send_rate, alt)
                continue

            cmd = text.lower()
            if cmd == "p":
                plot_all(ship, update_rate, send_rate, alt, save=plot_save)
                continue
            if cmd == "l":
                print_table(ship, update_rate, send_rate, alt)
                continue

            try:
                state = int(text)
            except ValueError:
                print("  숫자 또는 p / l / q 를 입력하세요.")
                continue
            if state not in STATES:
                print(f"  state 는 1~11 이어야 합니다 (받은 값: {state})")
                continue

            if state != TARGET_STATE:
                # state 전환은 반드시 도달해야 하므로 ACK 까지 재시도한다
                gcs.send_until_ready(state)
                continue

            # --- state 6: 여기부터 끝까지 자동 ---
            # 조작자는 6 을 한 번 누르고 끝이다.  타겟 입력이 없다.
            print_plan(ship, update_rate, send_rate, alt)
            first, _ = ship.target_at(0.0, update_rate, alt)
            gcs.send_until_ready(state, first)
            feed = ShipFeed(gcs, ship, update_rate, send_rate, alt)
            feed.start()

        except Quit:
            if feed is not None and feed.running:
                feed.stop()
            print("종료합니다.")
            return


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="TDCN state 6 — 선박 탑재 GCS 모사 (배 상태를 자동 전송)",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    ap.add_argument("--connect", "-c", default="udp:127.0.0.1:14551",
                    help="MAVLink 연결 문자열 (기본 udp:127.0.0.1:14551)")
    ap.add_argument("--sysid", type=int, default=0)
    ap.add_argument("--compid", type=int, default=1)
    ap.add_argument("--source-system", type=int, default=254)
    ap.add_argument("--ack-timeout", type=float, default=2.0)

    ap.add_argument("--ship-speed", type=float, default=SHIP_SPEED_MPS,
                    metavar="MPS",
                    help=f"배 속도 (기본 {SHIP_SPEED_MPS:g}m/s)")
    ap.add_argument("--update-rate", type=float, default=SHIP_UPDATE_RATE_HZ,
                    metavar="HZ",
                    help=f"배 상태 갱신 주기 "
                         f"(기본 {SHIP_UPDATE_RATE_HZ:g}Hz)")
    ap.add_argument("--send-rate", type=float, default=TELEM_SEND_RATE_HZ,
                    metavar="HZ",
                    help=f"텔레메트리 전송 주기 "
                         f"(기본 {TELEM_SEND_RATE_HZ:g}Hz)")
    ap.add_argument("--alt", type=float, default=traj_U, metavar="M",
                    help=f"고도 오프셋 (기본 {traj_U:g}m)")
    ap.add_argument("--north", type=float, default=SHIP_START_NORTH,
                    help="배 출발 North (m)")
    ap.add_argument("--east", type=float, default=SHIP_START_EAST,
                    help="배 출발 East (m)")

    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--plot-save", metavar="FILE")
    ap.add_argument("--preview", action="store_true",
                    help="접속하지 않고 궤적과 전송값만 확인하고 종료")
    ap.add_argument("--verbose", "-v", action="store_true")

    args = ap.parse_args(argv)

    if args.update_rate <= 0.0:
        ap.error(f"--update-rate 는 양수여야 합니다 (받은 값: {args.update_rate})")
    if args.send_rate <= 0.0:
        ap.error(f"--send-rate 는 양수여야 합니다 (받은 값: {args.send_rate})")

    try:
        ship = Ship(traj_NE, args.ship_speed,
                    north=args.north, east=args.east)
    except ValueError as exc:
        ap.error(str(exc))

    print(f"[gcs] 선박 탑재 GCS 모사 — state {TARGET_STATE} "
          f"{STATES[TARGET_STATE][0]} ({STATES[TARGET_STATE][1]})")
    print_plan(ship, args.update_rate, args.send_rate, args.alt)

    if args.preview:
        print_table(ship, args.update_rate, args.send_rate, args.alt)
        if not args.no_plot:
            plot_all(ship, args.update_rate, args.send_rate, args.alt,
                     save=args.plot_save)
        return 0

    if not args.no_plot:
        plot_all(ship, args.update_rate, args.send_rate, args.alt,
                 save=args.plot_save)

    gcs = TdcnGCS(args.connect, target_system=args.sysid,
                  target_component=args.compid,
                  source_system=args.source_system,
                  ack_timeout=args.ack_timeout,
                  verbose=args.verbose)
    try:
        interactive(gcs, ship, args.update_rate, args.send_rate, args.alt,
                    plot_save=args.plot_save)
    finally:
        gcs.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
