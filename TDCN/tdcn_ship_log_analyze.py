#!/usr/bin/env python3
"""
TDCN 선박 추종 로그 분석 (v4)  /* Sejong */

tdcn_gcs_NEU_ship.py 로 state 6 에서 배 궤적을 보낸 비행을 네 장으로 본다.
한꺼번에 띄운다.

    Figure 1   목표 (배가 보낸 값) vs 현재 — North / East / Up / Heading     4x1
    Figure 2   드론이 그린 궤적 (GCS 로 나가는 MAVLink 주기로 샘플링)        2D
    Figure 3   자세 — Roll / Pitch / Heading, 각속도 p / q / r              3x2
    Figure 4   믹서 직전 제어값 — 아두파일럿 vs CLAW                         4x1

v3 의 tdcn_log_analyze.py (선박 추종용) 를 v4 로그 (ArduCopter/mode_tdcn_log.cpp)
에 맞게 옮긴 것이다.  v3 의 Figure 1 (목표 vs 현재) / Figure 3 (궤적) 이 여기
Figure 1 / 2 이고, 자세 (Figure 3) 를 더했다.  v3 Figure 2 (믹서) 는 실제 믹서
입력 한 줄이었는데, 여기 Figure 4 는 아두파일럿 값과 CLAW 값을 겹쳐 그린다.
v4 의 tdcn_log_analyze.py 와는 보는 것이 다르다.

    tdcn_log_analyze.py        비행 전체 (TDCN 진입 ~ 이탈), 펌웨어 좌표
                               (EKF origin 기준 NED)
    tdcn_ship_log_analyze.py   배 추종 구간만, 배가 보낸 좌표 (home 기준 N / E / Up).
                               tdcn_gcs_NEU_ship.py 의 traj_NE / traj_U 숫자가
                               그림에 그대로 나온다


쓰는 로그 메시지
----------------
    MAVC   **목표값** — 기체가 받은 MAV_CMD_USER_1 (Cmd=31010) 원본 그대로.
           P1=state, P2=Heading (deg, 진북), X=North (cm), Y=East (cm),
           Z=Alt (m, home 기준 up).  배가 보낸 값이라 중간 변환이 끼지 않는다.
    TDST   **현재값** — 위치 (EKF origin 기준 NED, m), 자세 (deg), 모드 / 단계
    TDIM   각속도 — 주 IMU 자이로 (rad/s 를 deg/s 로 바꿔 그린다)
    TDMX   믹서 직전 제어값.  아두파일럿 (AR/AP/AY/AT) 과 CLAW (CR/CP/CY/CT).
           CLAW 는 state 6 에서 늘 계산되지만, 실제로 믹서에 들어간 것은
           Act = 1 인 구간뿐이다 (그 밖에는 아두파일럿 값이 들어갔다)
    ORGN   EKF origin (Type 0) / home (Type 1).  TDST 를 home 기준으로 옮긴다
    PARM   SR*_POSITION — GLOBAL_POSITION_INT 스트림 주기 (Figure 2)


목표값에서 state 6 만 고르는 이유
---------------------------------
MAVC 는 받은 명령을 전부 남긴다.  state 1~5, 12, 13 도 같은 Cmd 로 찍히는데
그때 X/Y/Z 는 0 이다.  P1 == 6 (TRACKING) 이고 기체가 받아들인 (Res == 0,
ACCEPTED) 것만 골라야 실제 타겟이 된다.


구간 / 시간축
-------------
배 추종 구간은 받아들인 첫 state 6 명령부터 단계가 6 을 벗어날 때 (보통 7 이나
13 을 넣은 뒤) 까지다.  시간축은 **그 첫 명령을 0** 으로 한 초다.  마지막 명령
뒤로는 기체가 그 값을 계속 쫓으므로 목표도 그 값으로 이어 그린다.
12 로 6 에 먼저 들어가 있던 구간 (초기 목표 = 현재 위치) 은 넣지 않는다.
TDCN 에 다시 들어와 또 보내 구간이 여럿이면 --seg 로 고른다.


좌표 기준
---------
MAVC 는 home 기준, TDST 는 EKF origin 기준이다.  ORGN 의 두 점 차이 (origin ->
home) 를 TDST 에서 빼서 home 기준으로 맞춘다.  home 은 무장할 때 다시 잡히므로
구간 시작 전 마지막 ORGN 을 쓴다.  Up = -PD - (home 고도 - origin 고도).
ORGN 의 lat/lng 는 소수 7자리라 ~1cm 로 양자화돼 있다.  추종 오차가 수십 cm
단위이므로 무시할 수 있다.


그리는 규칙 (v3 과 같다)
------------------------
목표는 빨강 굵은 선, 현재는 축마다 다른 색 (North 파랑, East 보라, Up 초록,
Heading 주황).  Roll / Pitch / Yaw 도 같은 순서로 파랑 / 보라 / 주황이다.
제어값 (Figure 4) 만 축이 아니라 출처로 색을 나눈다 — 아두파일럿 파랑, CLAW 빨강.
CLAW 가 믹서를 몬 구간 (TDMX.Act = 1) 이 있으면 시간축 그림에 빨강 음영을 깐다.

N/E/U 는 모두 m 라서 **같은 폭(span)** 으로 맞춘다.  세 축 중 변화폭이 가장 큰
것을 기준으로 삼고, 각 축은 자기 데이터의 중앙에 그 폭을 씌운다.  Roll / Pitch,
각속도 세 축도 같은 규칙이다.  Heading 은 0~360, 제어값은 정의역 (-1~1, 0~1)
으로 고정하고, ±1 / 0 / 1 에 붙은 값이 테두리에 가리지 않게 5% 여유를 둔다.


사용 예
-------
    ./tdcn_ship_log_analyze.py                        # 가장 최근 .BIN (logs/, ../logs/)
    ./tdcn_ship_log_analyze.py 00000012.BIN           # 특정 로그
    ./tdcn_ship_log_analyze.py --seg 2                # 두 번째 배 추종 구간
    ./tdcn_ship_log_analyze.py --save out/ship.png    # 창 대신 파일로 (ship_1.png ...)
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 로그 찾기 / 플롯 공통 도구는 v4 로그 분석 스크립트의 것을 쓴다
from tdcn_log_analyze import (  # noqa: E402
    TDCN_MODE,
    at_times,
    break_wrap,
    equal_span_limits,
    find_log,
    finish,
    setup_mpl,
    spans,
)

import numpy as np  # noqa: E402
from pymavlink import mavutil  # noqa: E402


MAV_CMD_USER_1 = 31010
TRACKING = 6
MAV_RESULT_ACCEPTED = 0

#: 위도 1도의 거리 (m).  ArduPilot 의 LOCATION_SCALING_FACTOR (1e-7 deg 당 m) 와 같다.
DEG_TO_M = 0.011131884502145034 * 1e7

#: SR*_POSITION 이 로그에 없을 때 쓰는 GCS 위치 스트림 주기 (Hz)
SR_POSITION_DEFAULT = 4.0

FIELDS = {
    "TDST": ("TimeUS", "Mode", "St", "Stp", "PN", "PE", "PD", "Roll", "Pitch", "Yaw"),
    "TDIM": ("TimeUS", "GyrX", "GyrY", "GyrZ"),
    "TDMX": ("TimeUS", "Act", "AR", "AP", "AY", "AT", "CR", "CP", "CY", "CT"),
    "MAVC": ("TimeUS", "Cmd", "P1", "P2", "X", "Y", "Z", "Res"),
    "ORGN": ("TimeUS", "Type", "Lat", "Lng", "Alt"),
}

# 색 — v3 과 같다
C_TARGET = "#d62728"
C_N, C_E, C_U, C_H = "#1f77b4", "#9467bd", "#2ca02c", "#ff7f0e"
C_AP, C_CLAW = "#1f77b4", "#d62728"       # Figure 4 — 출처별 (v4 tdcn_log_analyze.py 와 같다)
SHADE_ACT = dict(color="#d62728", alpha=0.10, lw=0)

LBL_TARGET = "목표"
LBL_CURRENT = "현재"
LBL_ACT = "CLAW 구동"
XLABEL = "t (s)"
FIGSIZE_4x1 = (14.0, 11.5)
FIGSIZE_3x2 = (17.0, 11.5)
FIGSIZE_2D = (10.0, 9.0)

# 글자 — 모두 굵게 (setup_plt).  크기는 여기서 한 번에 바꾼다
FS_TITLE = 15       # 그림 제목 (suptitle)
FS_LABEL = 14       # 축 이름 (x / y), 열 제목, 컬러바
FS_TICK = 12        # 눈금 숫자
FS_LEGEND = 11      # 범례
FS_NOTE = 11        # 그림 안 글씨 (실제 범위, 부제)


# ---------------------------------------------------------------------------
# 로그 읽기
# ---------------------------------------------------------------------------

def read_log(path: str) -> tuple[dict, float | None]:
    """필요한 메시지만 뽑아 필드별 배열로 돌려준다.  시간은 s 로 바꾼다.

    두 번째 값은 SR*_POSITION (Hz).  채널이 여럿이면 가장 큰 값을 쓴다 (실제로
    GCS 가 붙은 채널이 어느 쪽인지 로그만으로는 알 수 없다).
    """
    print(f"[log] 읽는 중: {path}")
    m = mavutil.mavlink_connection(path)
    cols = {k: {f: [] for f in fs} for k, fs in FIELDS.items()}
    sr_position: float | None = None

    while True:
        msg = m.recv_match(type=list(FIELDS) + ["PARM"])
        if msg is None:
            break
        t = msg.get_type()
        if t == "PARM":
            name = getattr(msg, "Name", "")
            if name.startswith("SR") and name.endswith("_POSITION"):
                v = float(getattr(msg, "Value", 0.0) or 0.0)
                if v > 0 and (sr_position is None or v > sr_position):
                    sr_position = v
            continue
        d = cols[t]
        for f in FIELDS[t]:
            d[f].append(getattr(msg, f, np.nan))

    data = {}
    for k, d in cols.items():
        n = len(d["TimeUS"])
        print(f"[log]   {k}  {n:>7}개")
        if n == 0:
            continue
        arr = {f: np.asarray(v, dtype=float) for f, v in d.items()}
        arr["t"] = arr.pop("TimeUS") / 1e6
        data[k] = arr
    if sr_position:
        print(f"[log]   SR*_POSITION = {sr_position:g} Hz "
              f"(GCS 로 나가는 GLOBAL_POSITION_INT 주기)")

    for need in FIELDS:
        if need not in data:
            sys.exit(f"{need} 메시지가 없습니다 — TDCN Part 3 로그 (mode_tdcn_log.cpp) "
                     f"가 들어간 v4 펌웨어의 로그인지, TDCN_LOG_HZ 가 0 이 아닌지 "
                     f"확인하세요.")
    return data, sr_position


def target_mask(c: dict) -> np.ndarray:
    """MAVC 중 기체가 받아들인 state 6 명령."""
    return ((c["Cmd"] == MAV_CMD_USER_1) & (np.round(c["P1"]) == TRACKING)
            & (c["Res"] == MAV_RESULT_ACCEPTED))


def ship_segments(data: dict) -> list[tuple[float, float]]:
    """배 추종 구간 [(시작, 끝)] (부팅 후 s).

    시작 = 받아들인 state 6 명령, 끝 = 그 뒤 단계가 6 을 벗어난 곳 (없으면 로그 끝).
    """
    c, s = data["MAVC"], data["TDST"]
    tc = c["t"][target_mask(c)]
    in6 = (s["Stp"] == TRACKING) & (s["Mode"] == TDCN_MODE)

    segs = []
    i = 0
    while i < tc.size:
        # 명령은 다음 run() 에 반영되므로 받은 직후 한두 샘플은 아직 이전 단계다.
        # 명령 뒤 처음 6 이 된 곳부터 6 을 벗어난 곳까지를 본다.
        k = int(np.searchsorted(s["t"], tc[i]))
        on = np.where(in6[k:])[0]
        if on.size == 0:
            break
        k += int(on[0])
        off = np.where(~in6[k:])[0]
        t_end = float(s["t"][k + off[0]]) if off.size else float(s["t"][-1])
        segs.append((float(tc[i]), t_end))
        i = int(np.searchsorted(tc, t_end, side="right"))
    return segs


def home_offset(data: dict, t_ref: float) -> tuple[float, float, float]:
    """EKF origin 에서 본 home 위치 (North, East, Up m).  t_ref 전 마지막 ORGN."""
    o = data["ORGN"]

    def last(kind: int):
        idx = np.where((o["Type"] == kind) & (o["t"] <= t_ref))[0]
        return int(idx[-1]) if idx.size else None

    io, ih = last(0), last(1)
    if io is None or ih is None:
        sys.exit("구간 시작 전에 ORGN (EKF origin / home) 이 없어 home 기준으로 "
                 "옮길 수 없습니다.")
    lat0, lng0, alt0 = o["Lat"][io], o["Lng"][io], o["Alt"][io]
    lat1, lng1, alt1 = o["Lat"][ih], o["Lng"][ih], o["Alt"][ih]
    coslat = math.cos(math.radians(0.5 * (lat0 + lat1)))
    off = (float((lat1 - lat0) * DEG_TO_M),
           float((lng1 - lng0) * DEG_TO_M * coslat),
           float(alt1 - alt0))
    print(f"[log] home (origin 기준): N={off[0]:+.3f} E={off[1]:+.3f} "
          f"U={off[2]:+.3f} m  (ORGN 은 소수 7자리라 ~1cm 양자화)")
    return off


def window(d: dict, t0: float, t1: float) -> dict:
    """[t0, t1] 만 남기고 시간축을 t0 기준으로 옮긴다."""
    sel = (d["t"] >= t0) & (d["t"] <= t1)
    out = {k: v[sel] for k, v in d.items()}
    out["t"] = out["t"] - t0
    return out


def extract_target(data: dict, t0: float, t1: float) -> dict:
    """구간 안의 state 6 목표.  마지막 명령 값을 구간 끝까지 잇는다."""
    c = data["MAVC"]
    c = {k: v[target_mask(c)] for k, v in c.items()}
    c = window(c, t0, t1)
    out = {
        "t": c["t"],
        "north": c["X"] * 0.01,         # cm -> m
        "east": c["Y"] * 0.01,
        "up": c["Z"],                   # 이미 m (home 기준 up)
        "hdg": c["P2"] % 360.0,         # deg, 0~360
        "n": c["t"].size,               # 실제로 받은 명령 수
    }
    t_end = t1 - t0
    if out["t"][-1] < t_end:
        for k in ("north", "east", "up", "hdg"):
            out[k] = np.append(out[k], out[k][-1])
        out["t"] = np.append(out["t"], t_end)
    return out


def extract_current(data: dict, off: tuple[float, float, float],
                    t0: float, t1: float) -> dict:
    """TDST 를 home 기준 NEU 로 옮긴 현재 위치 / 자세, TDIM 각속도."""
    s = window(data["TDST"], t0, t1)
    g = window(data["TDIM"], t0, t1)
    return {
        "t": s["t"],
        "north": s["PN"] - off[0],
        "east": s["PE"] - off[1],
        "up": -s["PD"] - off[2],
        "hdg": s["Yaw"] % 360.0,
        "roll": s["Roll"],
        "pitch": s["Pitch"],
        "gt": g["t"],
        "p": np.degrees(g["GyrX"]),
        "q": np.degrees(g["GyrY"]),
        "r": np.degrees(g["GyrZ"]),
    }


def extract_mixer(data: dict, t0: float, t1: float) -> dict:
    """믹서 직전 제어값 (TDMX 그대로) 과 CLAW 가 믹서를 몬 구간."""
    x = window(data["TDMX"], t0, t1)
    x["act_spans"] = spans(x["t"], x["Act"] > 0.5)
    return x


# ---------------------------------------------------------------------------
# 플롯 공통
# ---------------------------------------------------------------------------

def setup_plt():
    """한글 글꼴 (tdcn_log_analyze.setup_mpl) 에 더해 모든 글자를 굵게, 눈금을 크게."""
    plt = setup_mpl()
    plt.rcParams.update({
        "font.weight": "bold",
        "axes.labelweight": "bold",
        "axes.titleweight": "bold",
        "figure.titleweight": "bold",
        "xtick.labelsize": FS_TICK,
        "ytick.labelsize": FS_TICK,
    })
    return plt


def shade_act(ax, act_spans) -> None:
    """CLAW 가 믹서를 몬 구간.  범례에는 한 번만 올린다."""
    for i, (a, b) in enumerate(act_spans):
        ax.axvspan(a, b, zorder=0, label=LBL_ACT if i == 0 else None, **SHADE_ACT)


def set_heading_axis(ax) -> None:
    ax.set_ylim(0.0, 360.0)
    ax.set_yticks([0, 90, 180, 270, 360])


# ---------------------------------------------------------------------------
# Figure 1 — 목표 vs 현재
# ---------------------------------------------------------------------------

def figure1(tgt: dict, cur: dict, mix: dict, title: str,
            save: str | None) -> None:
    plt = setup_plt()
    tt, ct = tgt["t"], cur["t"]

    # N/E/U 는 같은 폭으로 (가장 큰 축 기준)
    lims, _ = equal_span_limits([
        np.concatenate([tgt["north"], cur["north"]]),
        np.concatenate([tgt["east"], cur["east"]]),
        np.concatenate([tgt["up"], cur["up"]]),
    ])

    rows = [
        ("North (m)",     "north", C_N, lims[0]),
        ("East (m)",      "east",  C_E, lims[1]),
        ("Up (m)",        "up",    C_U, lims[2]),
        ("Heading (deg)", "hdg",   C_H, None),
    ]

    fig, axes = plt.subplots(4, 1, figsize=FIGSIZE_4x1, sharex=True)
    fig.suptitle(f"[Figure 1] 목표 vs 현재  —  {title}", fontsize=FS_TITLE)

    for ax, (label, key, color, ylim) in zip(axes, rows):
        yt, yc = tgt[key], cur[key]
        if ylim is None:
            yt, yc = break_wrap(yt), break_wrap(yc)
        shade_act(ax, mix["act_spans"])
        ax.plot(tt, yt, "-", color=C_TARGET, lw=2.0, alpha=0.9, label=LBL_TARGET)
        ax.plot(ct, yc, "-", color=color, lw=1.4, alpha=0.95, label=LBL_CURRENT)
        ax.set_ylabel(label, fontsize=FS_LABEL)
        if ylim is None:
            set_heading_axis(ax)
        else:
            ax.set_ylim(*ylim)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=FS_LEGEND, loc="upper right")

    axes[-1].set_xlabel(XLABEL, fontsize=FS_LABEL)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    finish(fig, plt, save, 1)


# ---------------------------------------------------------------------------
# Figure 2 — 드론이 그린 궤적 (GCS 로 나가는 주기)
# ---------------------------------------------------------------------------

def figure2(tgt: dict, cur: dict, sr: float, title: str,
            save: str | None) -> None:
    plt = setup_plt()
    from matplotlib.collections import LineCollection

    pt, pn, pe = cur["t"], cur["north"], cur["east"]

    # GCS 가 실제로 받았을 주기로 솎는다.
    # GLOBAL_POSITION_INT 는 EKF 위치에서 나오고 SR*_POSITION Hz 로 나간다.
    period = 1.0 / sr
    keep = [0]
    last = pt[0]
    for i in range(1, pt.size):
        if pt[i] - last >= period - 1e-9:
            keep.append(i)
            last = pt[i]
    keep = np.asarray(keep)
    gn, ge, gt = pn[keep], pe[keep], pt[keep]

    fig, ax = plt.subplots(figsize=FIGSIZE_2D)
    fig.suptitle(f"[Figure 2] 궤적  —  {title}", fontsize=FS_TITLE)

    ax.plot(tgt["east"], tgt["north"], "-", color=C_TARGET, lw=1.6,
            alpha=0.5, label=LBL_TARGET, zorder=2)

    pts = np.array([ge, gn]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="viridis", linewidths=2.6, alpha=0.95,
                        zorder=3)
    lc.set_array(gt[:-1])
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, pad=0.02, fraction=0.045)
    cb.set_label("경과 시간 (s)", fontsize=FS_LABEL)

    ax.plot(ge, gn, "o", color="#333333", ms=3, alpha=0.7,
            label=f"{LBL_CURRENT} ({sr:g}Hz)", zorder=4)
    ax.plot(ge[0], gn[0], "o", color="#2ca02c", ms=12, label="시작", zorder=6)
    ax.plot(ge[-1], gn[-1], "X", color="#d62728", ms=11, label="끝", zorder=6)

    ax.set_aspect("equal", adjustable="datalim")
    ax.margins(0.12)
    ax.set_xlabel("East (m)", fontsize=FS_LABEL)
    ax.set_ylabel("North (m)", fontsize=FS_LABEL)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=FS_LEGEND, loc="best")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    finish(fig, plt, save, 2)


# ---------------------------------------------------------------------------
# Figure 3 — 자세 / 각속도
# ---------------------------------------------------------------------------

def figure3(tgt: dict, cur: dict, mix: dict, title: str,
            save: str | None) -> None:
    plt = setup_plt()
    ct, gt = cur["t"], cur["gt"]

    (rl, pl), _ = equal_span_limits([cur["roll"], cur["pitch"]])
    rate_lims, _ = equal_span_limits([cur["p"], cur["q"], cur["r"]])

    # (축 이름, 현재값, 색, y 범위).  Heading 은 목표를 겹친다
    left = [
        ("Roll (deg)",    cur["roll"],  C_N, rl),
        ("Pitch (deg)",   cur["pitch"], C_E, pl),
        ("Heading (deg)", cur["hdg"],   C_H, None),
    ]
    right = [
        ("p (deg/s)", cur["p"], C_N, rate_lims[0]),
        ("q (deg/s)", cur["q"], C_E, rate_lims[1]),
        ("r (deg/s)", cur["r"], C_H, rate_lims[2]),
    ]

    fig, axes = plt.subplots(3, 2, figsize=FIGSIZE_3x2, sharex=True)
    fig.suptitle(f"[Figure 3] 자세  —  {title}", fontsize=FS_TITLE)

    for r, (label, y, color, ylim) in enumerate(left):
        ax = axes[r][0]
        shade_act(ax, mix["act_spans"])
        if ylim is None:
            ax.plot(tgt["t"], break_wrap(tgt["hdg"]), "-", color=C_TARGET,
                    lw=2.0, alpha=0.9, label=LBL_TARGET)
            y = break_wrap(y)
        ax.plot(ct, y, "-", color=color, lw=1.2, alpha=0.95, label=LBL_CURRENT)
        ax.set_ylabel(label, fontsize=FS_LABEL)
        if ylim is None:
            set_heading_axis(ax)
        else:
            ax.set_ylim(*ylim)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=FS_LEGEND, loc="upper right")

    for r, (label, y, color, ylim) in enumerate(right):
        ax = axes[r][1]
        shade_act(ax, mix["act_spans"])
        ax.axhline(0.0, color="#888888", lw=0.8, ls=":", zorder=1)
        ax.plot(gt, y, "-", color=color, lw=1.0, alpha=0.95, label=LBL_CURRENT)
        ax.set_ylabel(label, fontsize=FS_LABEL)
        ax.set_ylim(*ylim)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=FS_LEGEND, loc="upper right")

    axes[0][0].set_title("자세", fontsize=FS_LABEL)
    axes[0][1].set_title("각속도", fontsize=FS_LABEL)
    axes[-1][0].set_xlabel(XLABEL, fontsize=FS_LABEL)
    axes[-1][1].set_xlabel(XLABEL, fontsize=FS_LABEL)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    finish(fig, plt, save, 3)


# ---------------------------------------------------------------------------
# Figure 4 — 믹서 직전 제어값
# ---------------------------------------------------------------------------

def range_text(ax, y, color: str, name: str, row: int) -> None:
    """실제 사용 범위를 숫자로 적는다 (축은 정의역 고정이라 좁아 보인다)."""
    y = y[np.isfinite(y)]
    txt = f"{name}  {y.min():+.3f} ~ {y.max():+.3f}" if y.size else f"{name}  -"
    ax.text(0.995, 0.05 + 0.09 * row, txt, transform=ax.transAxes,
            ha="right", va="bottom", fontsize=FS_NOTE, color=color,
            bbox=dict(facecolor="white", edgecolor="none", alpha=0.7, pad=1.0))


def figure4(mix: dict, title: str, save: str | None) -> None:
    plt = setup_plt()
    t = mix["t"]

    # (이름, 아두파일럿 필드, CLAW 필드, 정의역)
    rows = [
        ("Roll",     "AR", "CR", (-1.0, 1.0)),
        ("Pitch",    "AP", "CP", (-1.0, 1.0)),
        ("Yaw",      "AY", "CY", (-1.0, 1.0)),
        ("Throttle", "AT", "CT", (0.0, 1.0)),
    ]

    fig, axes = plt.subplots(4, 1, figsize=FIGSIZE_4x1, sharex=True)
    fig.suptitle(f"[Figure 4] 믹서 직전 제어값  —  {title}", fontsize=FS_TITLE)

    for ax, (label, ak, ck, ylim) in zip(axes, rows):
        shade_act(ax, mix["act_spans"])
        ax.axhline(0.0, color="#888888", lw=0.8, ls=":", zorder=1)
        ax.plot(t, mix[ck], "-", color=C_CLAW, lw=1.2, label="CLAW")
        ax.plot(t, mix[ak], "-", color=C_AP, lw=1.2, label="아두파일럿")
        ax.set_ylabel(label, fontsize=FS_LABEL)
        pad = 0.05 * (ylim[1] - ylim[0]) / 2.0     # 정의역 끝 (±1 포화) 이 테두리에 가리지 않게
        ax.set_ylim(ylim[0] - pad, ylim[1] + pad)
        ax.set_yticks(np.linspace(ylim[0], ylim[1], 5))
        ax.grid(alpha=0.3)
        ax.legend(fontsize=FS_LEGEND, loc="upper right")
        range_text(ax, mix[ak], C_AP, "아두파일럿", 1)
        range_text(ax, mix[ck], C_CLAW, "CLAW", 0)

    axes[-1].set_xlabel(XLABEL, fontsize=FS_LABEL)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    finish(fig, plt, save, 4)


# ---------------------------------------------------------------------------
# 요약
# ---------------------------------------------------------------------------

def print_summary(path: str, segs: list, seg: int, tgt: dict, cur: dict,
                  mix: dict, off: tuple[float, float, float]) -> None:
    """추종 오차 — 현재값 (TDST) 시각에 목표값을 ZOH 로 맞춰 비교한다."""
    ct = cur["t"]
    dn = cur["north"] - at_times(tgt["t"], tgt["north"], ct)
    de = cur["east"] - at_times(tgt["t"], tgt["east"], ct)
    du = cur["up"] - at_times(tgt["t"], tgt["up"], ct)
    dxy = np.hypot(dn, de)
    # 헤딩 오차는 원형이라 최단거리로 뺀다
    dh = (cur["hdg"] - at_times(tgt["t"], tgt["hdg"], ct) + 180.0) % 360.0 - 180.0

    t0, t1 = segs[seg - 1]
    dur = t1 - t0
    n = tgt["n"]
    t_last = float(tgt["t"][n - 1])
    rate = (n - 1) / t_last if n > 1 and t_last > 0 else 0.0
    act = sum(b - a for a, b in mix["act_spans"])

    print()
    print("=" * 64)
    print(f"  {os.path.basename(path)}   배 추종 구간 {seg}/{len(segs)}   "
          f"부팅 후 {t0:.1f} ~ {t1:.1f} s ({dur:.1f} s)")
    if len(segs) > 1:
        for i, (a, b) in enumerate(segs, 1):
            print(f"    구간 {i}   {a:7.1f} ~ {b:7.1f} s   (--seg {i})")
    print(f"  목표 (state 6 명령) {n}개, 명령 사이 {t_last:.1f} s = {rate:.1f} Hz")
    print(f"  home (origin 기준)  N {off[0]:+.3f}  E {off[1]:+.3f}  U {off[2]:+.3f} m")
    print(f"  CLAW 구동 (Act = 1)  {act:.1f} s")
    print("-" * 64)
    print(f"  {'축':<12}{'RMS':>10}{'평균':>10}{'최대|오차|':>12}")
    for name, d in (("North (m)", dn), ("East (m)", de), ("Up (m)", du)):
        print(f"  {name:<12}{np.sqrt(np.mean(d**2)):>10.3f}"
              f"{np.mean(d):>10.3f}{np.max(np.abs(d)):>12.3f}")
    print(f"  {'수평거리 (m)':<12}{np.sqrt(np.mean(dxy**2)):>10.3f}"
          f"{np.mean(dxy):>10.3f}{np.max(dxy):>12.3f}")
    print(f"  {'Heading(deg)':<12}{np.sqrt(np.mean(dh**2)):>10.3f}"
          f"{np.mean(dh):>10.3f}{np.max(np.abs(dh)):>12.3f}")
    print("=" * 64)
    print("  오차 = 현재 - 목표.  목표는 계단이라 ZOH 로 맞췄다.")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="TDCN 선박 추종 로그 분석 (v4) — 목표 vs 현재, 궤적, 자세, 제어값",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?", help="로그 파일 (생략 시 가장 최근 .BIN)")
    ap.add_argument("--seg", type=int, default=1, metavar="N",
                    help="배 추종 구간 번호 (기본 1)")
    ap.add_argument("--save", metavar="FILE",
                    help="플롯을 창 대신 파일로 저장 (FILE_1.png 등)")
    ap.add_argument("--sr-position", type=float, metavar="HZ",
                    help="GLOBAL_POSITION_INT 주기를 직접 지정 "
                         "(기본: 로그의 SR*_POSITION)")
    args = ap.parse_args(argv)

    path = find_log(args.log)
    data, sr = read_log(path)
    sr = args.sr_position or sr or SR_POSITION_DEFAULT

    segs = ship_segments(data)
    if not segs:
        sys.exit("기체가 받아들인 state 6 명령 (배 궤적) 이 없습니다 — "
                 "tdcn_gcs_NEU_ship.py 로 6 을 보낸 로그가 아닙니다.  "
                 "비행 전체는 tdcn_log_analyze.py 로 보세요.")
    if not 1 <= args.seg <= len(segs):
        sys.exit(f"--seg 는 1~{len(segs)} 이어야 합니다 (받은 값: {args.seg})")
    t0, t1 = segs[args.seg - 1]

    off = home_offset(data, t0)
    tgt = extract_target(data, t0, t1)
    cur = extract_current(data, off, t0, t1)
    mix = extract_mixer(data, t0, t1)

    print_summary(path, segs, args.seg, tgt, cur, mix, off)

    title = os.path.basename(path)
    if len(segs) > 1:
        title += f"  구간 {args.seg}"
    figure1(tgt, cur, mix, title, args.save)
    figure2(tgt, cur, sr, title, args.save)
    figure3(tgt, cur, mix, title, args.save)
    figure4(mix, title, args.save)

    if not args.save:
        # 네 장을 한꺼번에 띄운다.  장마다 show() 를 부르면 창을 하나 닫아야
        # 다음 장이 뜬다.  여기서 한 번만 부른다.
        print("[plot] Figure 1~4 를 함께 띄웁니다 — 모두 닫으면 종료합니다 ...")
        setup_mpl().show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
