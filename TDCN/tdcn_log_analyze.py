#!/usr/bin/env python3
"""
TDCN 비행 로그 분석  /* Sejong */

state 6 (추종 비행) 구간을 세 장으로 본다.

    Figure 1   타겟(N,E,U,Heading) vs FC 의 EKF 추정값 — 축별 4x1
    Figure 2   믹서에 들어가기 직전 제어값 (roll/pitch/yaw/throttle) — 4x1
    Figure 3   드론이 실제로 그린 궤적 (GCS 로 나가는 MAVLink 주기로 샘플링)

쓰는 로그 메시지
----------------
    MAVC   **목표값** — 수신된 MAV_CMD_USER_1 (Cmd=31010) 원본 그대로.
           P1=state, P2=Heading(deg, 0~360), X=North(cm), Y=East(cm),
           Z=Alt(m, home 기준 up).  Fr=1 (MAV_FRAME_LOCAL_NED).
           GCS 가 실제로 보낸 값이므로 중간 변환이 끼지 않는다.
    POS    **현재값** — ArduPilot 표준 위치 로그.  Lat/Lng (deg),
           RelHomeAlt (m).  GLOBAL_POSITION_INT 와 같은 EKF 위치다.
    ATT    **현재 헤딩** — Yaw (deg, 0~360)
    TDCC   제어값 — MR/MP/MY (믹서 입력, -1~1), MT (스로틀, 0~1), ACT
    ORGN   home / EKF 원점 (lat/lng).  POS 를 NE 로 바꿀 때 쓴다
    PARM   SR*_POSITION — GLOBAL_POSITION_INT 스트림 주기 (Hz)


좌표 기준
---------
MAVC 의 X/Y 는 **home 기준** North/East cm 다 (펌웨어가 MAV_FRAME_LOCAL_NED 를
ahrs.get_home() + offset 으로 푼다).  Z 도 home 기준 up (m).
그래서 POS 의 lat/lng 도 **home** 을 원점으로 NE 로 바꿔야 둘이 같은 좌표계가
된다.  ORGN Type 1 (home) 을 쓴다.

ORGN 의 lat/lng 는 소수 7자리라 ~1cm 로 양자화돼 있다.  추종 오차가 수십 cm
단위이므로 무시할 수 있다.


목표값에서 state 6 만 고르는 이유
---------------------------------
MAVC 는 받은 명령을 전부 남긴다.  state 1~5 를 보낼 때도 같은 Cmd 로 찍히는데
그때 X/Y/Z 는 0 이다.  P1 == 6 (TRACKING) 인 것만 골라야 실제 타겟이 된다.


Figure 1 의 y 축 규칙
---------------------
N/E/U 는 모두 m 라서 **같은 폭(span)** 으로 맞춘다.  세 축 중 변화폭이 가장 큰
것을 기준으로 삼고, 각 축은 자기 데이터의 중앙에 그 폭을 씌운다.  그래야 축마다
움직인 크기를 눈으로 바로 비교할 수 있다 (한 축만 확대돼 보이지 않는다).

Heading 은 MAVC.P2 도 ATT.Yaw 도 **0~360** 이므로 그 범위로 고정한다.
데이터에 맞춰 확대하지 않는다.


사용 예
-------
    ./tdcn_log_analyze.py                      # logs/ 에서 가장 최근 .BIN
    ./tdcn_log_analyze.py 00000004.BIN         # 특정 로그
    ./tdcn_log_analyze.py --save out/fig.png   # 창 대신 파일로
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink 이 필요합니다:  pip install pymavlink")

try:
    import numpy as np
except ImportError:
    sys.exit("numpy 가 필요합니다:  pip install numpy")


# 로그를 찾을 곳
LOG_DIRS = (
    "logs",
    "../logs",
    os.path.expanduser("~/Desktop/KAT/ardupilot/logs"),
)

#: 위도 1도의 거리 (m).  ArduPilot 과 같은 값을 쓴다.
DEG_TO_M = 111319.5


# ---------------------------------------------------------------------------
# 로그 읽기
# ---------------------------------------------------------------------------

def find_log(name: str | None) -> str:
    """로그 경로를 정한다.  이름이 없으면 가장 최근 .BIN."""
    if name and os.path.isfile(name):
        return name
    cands: list[str] = []
    for d in LOG_DIRS:
        if name:
            p = os.path.join(d, name)
            if os.path.isfile(p):
                return p
        cands.extend(glob.glob(os.path.join(d, "*.BIN")))
        cands.extend(glob.glob(os.path.join(d, "*.bin")))
    if not cands:
        sys.exit(f"로그를 찾을 수 없습니다.  찾아본 곳: {', '.join(LOG_DIRS)}")
    newest = max(cands, key=os.path.getmtime)
    return newest


def read_log(path: str) -> dict:
    """필요한 메시지만 뽑아 배열로 돌려준다."""
    print(f"[log] 읽는 중: {path}")
    m = mavutil.mavlink_connection(path)

    keep = {"MAVC", "POS", "ATT", "TDCC", "ORGN"}
    out: dict[str, dict[str, list]] = {k: {} for k in keep}
    sr_position: float | None = None
    n = 0

    while True:
        msg = m.recv_match()
        if msg is None:
            break
        t = msg.get_type()

        if t == "PARM":
            # GCS 로 나가는 위치 스트림 주기.  채널이 여럿이면 가장 큰 값을 쓴다
            # (실제로 GCS 가 붙은 채널이 어느 쪽인지 로그만으로는 알 수 없다).
            name = getattr(msg, "Name", "")
            if name.endswith("_POSITION") and name.startswith("SR"):
                v = float(getattr(msg, "Value", 0.0) or 0.0)
                if v > 0 and (sr_position is None or v > sr_position):
                    sr_position = v
            continue

        if t not in keep:
            continue
        n += 1
        d = out[t]
        for k, v in msg.to_dict().items():
            if k == "mavpackettype":
                continue
            d.setdefault(k, []).append(v)

    data = {k: {kk: np.asarray(vv, dtype=float) for kk, vv in v.items()}
            for k, v in out.items() if v}
    for k in keep:
        cnt = len(data.get(k, {}).get("TimeUS", []))
        print(f"[log]   {k:<5} {cnt:>7}개")
    if sr_position:
        print(f"[log]   SR*_POSITION = {sr_position:g} Hz "
              f"(GCS 로 나가는 GLOBAL_POSITION_INT 주기)")
    data["_sr_position"] = sr_position
    return data


def home_origin(data: dict) -> tuple[float, float]:
    """POS 를 NE 로 바꿀 원점 — home (ORGN Type 1).

    MAVC 의 X/Y 가 home 기준 NE 이므로 현재값도 home 을 원점으로 삼아야
    같은 좌표계가 된다.  ORGN 은 Type 0 = EKF 원점, Type 1 = home 이다.
    """
    o = data.get("ORGN")
    if not o:
        sys.exit("ORGN 메시지가 없어 home 을 알 수 없습니다.")
    types = o.get("Type")
    lat, lng = o.get("Lat"), o.get("Lng")
    sel = np.where(types == 1)[0]
    if sel.size == 0:
        sel = np.where(types == 0)[0]
        print("[log] ! home(ORGN Type1) 이 없어 EKF 원점(Type0) 을 씁니다")
    if sel.size == 0:
        sys.exit("ORGN 에 쓸 만한 원점이 없습니다.")
    lat0, lng0 = float(lat[sel[-1]]), float(lng[sel[-1]])
    print(f"[log] 원점(home): lat={lat0:.7f} lng={lng0:.7f}  "
          f"(ORGN 은 소수 7자리라 ~1cm 양자화)")
    return lat0, lng0


def extract_target(data: dict) -> dict:
    """MAVC 에서 state 6 타겟만 뽑는다.

    MAVC 는 받은 명령을 전부 남기므로 Cmd(=31010) 와 P1(=6, TRACKING) 로
    거른다.  state 1~5 도 같은 Cmd 로 찍히는데 그때 X/Y/Z 는 0 이다.
    """
    c = data.get("MAVC")
    if not c:
        sys.exit("MAVC 메시지가 없습니다 — 수신 명령 로깅이 꺼져 있었을 수 "
                 "있습니다 (LOG_BITMASK).")
    sel = (c["Cmd"] == 31010) & (c["P1"] == 6)
    if not np.any(sel):
        sys.exit("MAV_CMD_USER_1 의 state 6 명령이 로그에 없습니다.")
    t = c["TimeUS"][sel] / 1e6
    out = {
        "t": t,
        "north": c["X"][sel] * 0.01,      # cm -> m
        "east": c["Y"][sel] * 0.01,
        "up": c["Z"][sel],                # 이미 m (home 기준 up)
        "hdg": c["P2"][sel] % 360.0,      # deg, 0~360
    }
    dur = t[-1] - t[0]
    print(f"[log] 타겟(MAV_CMD_USER_1, state 6): {t.size}개 / {dur:.1f}s "
          f"= {t.size / dur:.1f}Hz")
    return out


def to_ne(lat, lng, lat0: float, lng0: float):
    """lat/lng (deg) -> 원점 기준 North/East (m)."""
    coslat = math.cos(math.radians(lat0))
    return ((lat - lat0) * DEG_TO_M,
            (lng - lng0) * DEG_TO_M * coslat)


def wrap180(deg):
    """-180 ~ 180 으로 감는다.  **이미 범위 안인 값은 건드리지 않는다.**

    그냥 (x+180)%360-180 을 쓰면 정확히 +180 인 값이 -180 으로 뒤집힌다.
    로그의 THdg 는 펌웨어가 이미 wrap_180 을 거쳐 기록한 값이라 180.0 이
    그대로 들어 있는데, 그걸 -180 으로 바꾸면 90 -> -180 처럼 거대한 역주행으로
    보인다.  범위를 벗어난 값만 감는다.
    """
    a = np.asarray(deg, dtype=float)
    out = np.where((a >= -180.0) & (a <= 180.0), a,
                   (a + 180.0) % 360.0 - 180.0)
    return out


def break_wrap(y, limit: float = 180.0):
    """+-180 경계를 넘는 자리를 NaN 으로 끊는다.

    끊지 않으면 +179.9 -> -179.9 가 한 점에서 위아래를 잇는 세로선으로 그려져,
    실제로는 없는 급변처럼 보인다.  값이 경계에서 떨릴 때 줄무늬가 된다.
    """
    y = np.asarray(y, dtype=float).copy()
    d = np.abs(np.diff(y))
    y[:-1][d > limit] = np.nan
    return y


# ---------------------------------------------------------------------------
# 플롯 공통
# ---------------------------------------------------------------------------

def setup_mpl():
    import matplotlib.pyplot as plt
    from matplotlib import font_manager as fm
    for cand in ("NanumGothic", "Malgun Gothic", "Noto Sans CJK KR",
                 "Noto Sans CJK JP"):
        if any(f.name == cand for f in fm.fontManager.ttflist):
            plt.rcParams["font.family"] = cand
            break
    plt.rcParams["axes.unicode_minus"] = False
    return plt


def save_name(save: str, k: int) -> str:
    root, ext = os.path.splitext(save)
    return f"{root}_{k}{ext or '.png'}"


def finish(fig, plt, save: str | None, k: int) -> None:
    """저장 모드면 파일로 쓰고 닫는다.  아니면 열어 둔 채 돌아간다.

    장마다 plt.show() 를 부르면 창을 하나 닫아야 다음 장이 뜬다.  세 장을
    한꺼번에 보려면 여기서 show 하지 않고, 마지막에 한 번만 불러야 한다.
    """
    if save:
        out = save_name(save, k)
        d = os.path.dirname(os.path.abspath(out))
        if d:
            os.makedirs(d, exist_ok=True)
        fig.savefig(out, dpi=130)
        print(f"[plot] 저장: {out}")
        plt.close(fig)


def equal_span_limits(series_list: list[np.ndarray], pad: float = 0.08):
    """여러 축을 같은 폭으로 맞춘다.

    가장 변화폭이 큰 축을 기준으로 폭을 정하고, 각 축은 자기 데이터 중앙에
    그 폭을 씌운다.  폭이 같아야 축끼리 움직인 크기를 눈으로 비교할 수 있다.
    """
    spans = [float(np.nanmax(s) - np.nanmin(s)) for s in series_list]
    span = max(spans) if spans else 1.0
    if span <= 0:
        span = 1.0
    span *= (1.0 + 2.0 * pad)
    lims = []
    for s in series_list:
        mid = 0.5 * (float(np.nanmax(s)) + float(np.nanmin(s)))
        lims.append((mid - span / 2.0, mid + span / 2.0))
    return lims, span


# ---------------------------------------------------------------------------
# Figure 1 — 타겟 vs EKF
# ---------------------------------------------------------------------------

def zoh(t_src, y_src, t_dst):
    """타겟을 목적 시각에 맞춘다 (zero-order hold).

    타겟은 계단으로 갱신되므로 선형보간하면 없던 기울기가 생긴다.
    각 시각에서 "직전에 받은 값" 을 쓴다.
    """
    idx = np.searchsorted(t_src, t_dst, side="right") - 1
    idx = np.clip(idx, 0, len(t_src) - 1)
    return np.asarray(y_src)[idx]


def figure1(data: dict, tgt: dict, lat0: float, lng0: float, t0: float,
            save: str | None) -> None:
    plt = setup_mpl()
    pos, att = data["POS"], data["ATT"]

    # 타겟 — MAVC 원본 (home 기준 NEU + heading)
    tt = tgt["t"] - t0
    tn, te, tu, th = tgt["north"], tgt["east"], tgt["up"], tgt["hdg"]

    # 현재값 — POS (표준 위치 로그) + ATT (자세)
    pt = pos["TimeUS"] / 1e6 - t0
    pn, pe = to_ne(pos["Lat"], pos["Lng"], lat0, lng0)
    pu = pos["RelHomeAlt"]
    at = att["TimeUS"] / 1e6 - t0
    ah = att["Yaw"] % 360.0

    # state 6 구간만 본다
    t_end = float(tt[-1])
    ps = (pt >= 0.0) & (pt <= t_end)
    as_ = (at >= 0.0) & (at <= t_end)
    pt, pn, pe, pu = pt[ps], pn[ps], pe[ps], pu[ps]
    at, ah = at[as_], ah[as_]

    # N/E/U 는 같은 폭으로 (가장 큰 축 기준)
    lims, span = equal_span_limits([
        np.concatenate([tn, pn]),
        np.concatenate([te, pe]),
        np.concatenate([tu, pu]),
    ])

    rows = [
        ("North (m)", tt, tn, pt, pn, "#1f77b4", lims[0], False),
        ("East (m)",  tt, te, pt, pe, "#9467bd", lims[1], False),
        ("Up (m)",    tt, tu, pt, pu, "#2ca02c", lims[2], False),
        ("Heading (deg)", tt, th, at, ah, "#ff7f0e", (0.0, 360.0), True),
    ]

    fig, axes = plt.subplots(4, 1, figsize=(12.5, 10.5), sharex=True)
    fig.suptitle(
        "[Figure 1] 목표값(MAV_CMD_USER_1 수신 원본) vs 현재값(EKF 위치 로그)\n"
        f"N/E/U 는 같은 폭 {span:.1f}m 로 고정 (가장 큰 축 기준), "
        f"Heading 은 0~360 고정", fontsize=12)

    for ax, (label, xt, yt, xc, yc, color, ylim, is_hdg) in zip(axes, rows):
        yt_p = break_wrap(yt, 180.0) if is_hdg else yt
        yc_p = break_wrap(yc, 180.0) if is_hdg else yc
        ax.plot(xt, yt_p, "-", color="#d62728", lw=2.0, alpha=0.9,
                label="목표 (GCS 가 보낸 MAV_CMD_USER_1)")
        ax.plot(xc, yc_p, "-", color=color, lw=1.4, alpha=0.95,
                label="현재 (FC EKF 위치/자세 로그)")
        ax.set_ylabel(label, fontsize=10)
        ax.set_ylim(*ylim)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")

    axes[3].set_yticks([0, 90, 180, 270, 360])
    axes[-1].set_xlabel("t (s)   — state 6 진입을 0 으로")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    finish(fig, plt, save, 1)


# ---------------------------------------------------------------------------
# Figure 2 — 믹서 직전 제어값
# ---------------------------------------------------------------------------

def figure2(data: dict, t0: float, save: str | None) -> None:
    plt = setup_mpl()
    c = data["TDCC"]
    t = c["TimeUS"] / 1e6 - t0

    act = c.get("ACT")
    claw_drove = bool(act is not None and np.any(act > 0.5))

    rows = [
        ("Roll 명령",     c["MR"], (-1.0, 1.0), "#1f77b4"),
        ("Pitch 명령",    c["MP"], (-1.0, 1.0), "#9467bd"),
        ("Yaw 명령",      c["MY"], (-1.0, 1.0), "#ff7f0e"),
        ("Throttle 명령", c["MT"], (0.0, 1.0),  "#2ca02c"),
    ]

    fig, axes = plt.subplots(4, 1, figsize=(12.5, 10.5), sharex=True)
    src = ("CLAW 가 믹서를 몬 구간이 있습니다 (TDCC.ACT=1)"
           if claw_drove else
           "전 구간 아두파일럿 출력 (TDCC.ACT=0, CLAW 는 믹서를 몰지 않음)")
    fig.suptitle(f"[Figure 2] 믹서 직전 제어값  —  {src}\n"
                 f"y 축은 각 변수의 정의역으로 고정: "
                 f"Roll/Pitch/Yaw = -1~1, Throttle = 0~1", fontsize=12)

    for ax, (label, y, ylim, color) in zip(axes, rows):
        ax.plot(t, y, "-", color=color, lw=1.2)
        ax.axhline(0.0, color="#888888", lw=0.8, ls=":", zorder=0)
        ax.set_ylabel(label, fontsize=10)
        ax.set_ylim(*ylim)
        ax.grid(alpha=0.3)
        # 실제 사용 범위를 숫자로 적어 둔다 (축은 정의역 고정이라 좁아 보인다)
        ax.text(0.995, 0.06,
                f"실제 범위 {np.nanmin(y):+.3f} ~ {np.nanmax(y):+.3f}",
                transform=ax.transAxes, ha="right", fontsize=8,
                color="#555555")

    axes[0].set_yticks([-1, -0.5, 0, 0.5, 1])
    axes[1].set_yticks([-1, -0.5, 0, 0.5, 1])
    axes[2].set_yticks([-1, -0.5, 0, 0.5, 1])
    axes[3].set_yticks([0, 0.25, 0.5, 0.75, 1])
    axes[-1].set_xlabel("t (s)   — state 6 진입을 0 으로")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    finish(fig, plt, save, 2)


# ---------------------------------------------------------------------------
# Figure 3 — 드론이 그린 궤적 (GCS 로 나가는 주기)
# ---------------------------------------------------------------------------

def figure3(data: dict, tgt: dict, lat0: float, lng0: float, t0: float,
            save: str | None) -> None:
    plt = setup_mpl()
    from matplotlib.collections import LineCollection

    sr = data.get("_sr_position") or 4.0
    pos = data.get("POS")
    if not pos:
        print("[plot] POS 메시지가 없어 Figure 3 을 건너뜁니다.")
        return

    t_end = float(tgt["t"][-1] - t0)
    pt = pos["TimeUS"] / 1e6 - t0
    pn, pe = to_ne(pos["Lat"], pos["Lng"], lat0, lng0)

    sel = (pt >= 0.0) & (pt <= t_end)
    pt, pn, pe = pt[sel], pn[sel], pe[sel]
    if pt.size == 0:
        print("[plot] state 6 구간에 POS 가 없습니다.")
        return

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

    fig, ax = plt.subplots(figsize=(8.8, 7.8))
    fig.suptitle(f"[Figure 3] 드론이 그린 궤적  —  GCS 로 나가는 "
                 f"GLOBAL_POSITION_INT 주기 {sr:g}Hz 로 샘플링 ({gn.size}점)",
                 fontsize=12)

    ax.plot(tgt["east"], tgt["north"], "-", color="#d62728", lw=1.6,
            alpha=0.5, label="목표 경로 (MAV_CMD_USER_1)", zorder=2)

    pts = np.array([ge, gn]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="viridis", linewidths=2.6, alpha=0.95,
                        zorder=3)
    lc.set_array(gt[:-1])
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, pad=0.02, fraction=0.045)
    cb.set_label("경과 시간 (s)", fontsize=9)

    ax.plot(ge, gn, "o", color="#333333", ms=3, alpha=0.7,
            label=f"GCS 수신 위치 ({sr:g}Hz, {gn.size}점)", zorder=4)
    ax.plot(ge[0], gn[0], "o", color="#2ca02c", ms=12, label="시작", zorder=6)
    ax.plot(ge[-1], gn[-1], "X", color="#d62728", ms=11, label="끝", zorder=6)

    ax.set_aspect("equal", adjustable="datalim")
    ax.margins(0.12)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("선 색 = 경과시간 (겹친 구간 구분용)", fontsize=9)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    finish(fig, plt, save, 3)


# ---------------------------------------------------------------------------
# 요약
# ---------------------------------------------------------------------------

def print_summary(data: dict, tgt: dict, lat0: float, lng0: float,
                  t0: float) -> None:
    """추종 오차 — 현재값(POS/ATT) 시각에 목표값을 ZOH 로 맞춰 비교한다."""
    pos, att = data["POS"], data["ATT"]
    t_end = float(tgt["t"][-1] - t0)

    pt = pos["TimeUS"] / 1e6 - t0
    pn, pe = to_ne(pos["Lat"], pos["Lng"], lat0, lng0)
    pu = pos["RelHomeAlt"]
    sel = (pt >= 0.0) & (pt <= t_end)
    pt, pn, pe, pu = pt[sel], pn[sel], pe[sel], pu[sel]

    tt = tgt["t"] - t0
    dn = pn - zoh(tt, tgt["north"], pt)
    de = pe - zoh(tt, tgt["east"], pt)
    du = pu - zoh(tt, tgt["up"], pt)
    dxy = np.hypot(dn, de)

    at = att["TimeUS"] / 1e6 - t0
    ah = att["Yaw"] % 360.0
    s2 = (at >= 0.0) & (at <= t_end)
    at, ah = at[s2], ah[s2]
    # 헤딩 오차는 원형이라 최단거리로 뺀다
    dh = (ah - zoh(tt, tgt["hdg"], at) + 180.0) % 360.0 - 180.0

    print()
    print("=" * 64)
    print(f"  state 6 구간 {t_end:.1f}s   목표 {tgt['t'].size}개 / "
          f"현재 {pt.size}개")
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
        description="TDCN 비행 로그 분석 — 목표 vs 현재, 제어값, 궤적",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?", help="로그 파일 (생략 시 가장 최근 .BIN)")
    ap.add_argument("--save", metavar="FILE",
                    help="플롯을 창 대신 파일로 저장 (FILE_1.png 등)")
    ap.add_argument("--sr-position", type=float, metavar="HZ",
                    help="GLOBAL_POSITION_INT 주기를 직접 지정 "
                         "(기본: 로그의 SR*_POSITION)")
    args = ap.parse_args(argv)

    path = find_log(args.log)
    data = read_log(path)

    for need in ("MAVC", "POS", "ATT", "TDCC"):
        if need not in data:
            sys.exit(f"{need} 메시지가 없습니다 — TDCN 모드로 비행한 "
                     f"로그가 맞는지 확인하세요.")

    if args.sr_position:
        data["_sr_position"] = args.sr_position

    lat0, lng0 = home_origin(data)
    tgt = extract_target(data)

    # state 6 진입(첫 타겟)을 0 초로 잡는다
    t0 = float(tgt["t"][0])

    print_summary(data, tgt, lat0, lng0, t0)

    figure1(data, tgt, lat0, lng0, t0, args.save)
    figure2(data, t0, args.save)
    figure3(data, tgt, lat0, lng0, t0, args.save)

    if not args.save:
        # 세 장을 한꺼번에 띄운다.  장마다 show() 를 부르면 창을 하나 닫아야
        # 다음 장이 뜬다.  여기서 한 번만 부른다.
        print("[plot] Figure 1~3 을 함께 띄웁니다 — 모두 닫으면 종료합니다 ...")
        setup_mpl().show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
