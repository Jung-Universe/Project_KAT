#!/usr/bin/env python3
"""
TDCN 로그 분석 (v4)  /* Sejong */

TDCN Part 3 로그 (ArduCopter/mode_tdcn_log.cpp) 를 세 장으로 본다.  한꺼번에 띄운다.

    Figure 1   현재 상태 — 위치 / 속도 / 자세                  (TDST)
    Figure 2   목표 — 위치 / 헤딩.  현재값을 겹쳐 그린다      (TDTG, TDST)
    Figure 3   믹서 직전 제어값 — 아두파일럿 vs CLAW          (TDMX)

쓰는 로그 메시지 (같은 루프에서 같은 TimeUS 로 기록된다)
--------------------------------------------------------
    TDST   Mode (비행 모드), St (TDCN state 0~13), Stp (실제 단계 0~11),
           PN/PE/PD (m), VN/VE/VD (m/s), Roll/Pitch/Yaw (deg)
    TDTG   Val (목표 유무), PN/PE/PD (m), Hdg (deg)
    TDMX   Act (CLAW 가 믹서를 몰았는가), AR/AP/AY/AT (아두파일럿),
           CR/CP/CY/CT (CLAW).  roll/pitch/yaw 는 -1~1, throttle 은 0~1

    위치는 EKF origin 기준 NED (m) 다.  PSCN / PSCE / PSCD 와 같은 기준이다.


그리는 규칙
-----------
세 장 모두 같은 뜻에 같은 모양을 쓰고, 범례도 같은 자리 / 같은 순서다.

    목표          검정 점선
    아두파일럿    파랑 실선   (현재값 = 아두파일럿 EKF, 제어값 = 아두파일럿 제어기)
    CLAW          빨강 실선
    회색 점선     TDCN 단계 전환.  맨 위 축 위의 숫자가 새 단계 (Stp),
                  12 / 13 은 그 자리에서 자동 진행 명령을 받았다는 뜻
    회색 음영     TDCN 밖 (Mode != 29).  펌웨어가 이 구간을 0 으로 남긴다
    빨강 음영     CLAW 가 믹서를 몬 구간 (Act = 1)

값은 로그에 남은 그대로 그린다.  예외는 목표 위치 하나다 — Val = 0 (지상 처리
중, 목표 없음) 인 구간은 펌웨어 주석대로 쓸 수 없는 값이라 비운다.  목표 헤딩도
TDCN 밖에서는 비운다.

N/E/D 는 같은 폭으로 맞춘다 (v3 와 같다).  가장 크게 움직인 축을 기준으로 폭을
정하고, 각 축은 자기 데이터 중앙에 그 폭을 씌운다.  속도와 Roll/Pitch 도 같은
규칙이다.  제어값은 정의역 (-1~1, 0~1) 으로 고정하고, ±1 / 0 / 1 에 붙은 값이
테두리에 가리지 않게 5% 여유를 둔다.

Yaw / Heading 은 한 바퀴로 고정하되 범위는 데이터로 고른다.  북쪽 근처를 돌면
0~360 경계에서 선이 위아래 테두리에 붙어 안 보이므로 -180~180 을, 아니면
0~360 을 쓴다.  Figure 1 과 2 는 같은 범위를 쓴다.


그리는 구간 / 시간축
--------------------
로그는 전원 인가부터 남지만, 그림은 TDCN 첫 진입 ~ 마지막 이탈만 그린다.
시간축은 **TDCN 첫 진입을 0** 으로 한 초다.  재진입이 있으면 그 사이 TDCN 밖
구간은 회색 음영으로 남고, 재진입 자리에 단계 숫자 0 이 다시 찍힌다.
부팅 후 시각 (다른 로그 뷰어의 시각) = 진입 기준 시각 + 진입 시각이며, 진입
시각은 부제와 요약에 적는다.


사용 예
-------
    ./tdcn_log_analyze.py                      # 가장 최근 .BIN (logs/, ../logs/)
    ./tdcn_log_analyze.py 00000007.BIN         # 특정 로그
    ./tdcn_log_analyze.py --start 40 --end 70  # 구간 (s, TDCN 진입 기준)
    ./tdcn_log_analyze.py --save out/fig.png   # 창 대신 파일로 (fig_1.png ...)
"""

from __future__ import annotations

import argparse
import glob
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


HERE = os.path.dirname(os.path.abspath(__file__))

# 로그를 찾을 곳.  TDCN/ 은 ardupilot/ 안에 있으므로 ../logs 가 SITL 로그 폴더다.
LOG_DIRS = (
    "logs",
    os.path.join(HERE, "..", "logs"),
)

TDCN_MODE = 29

# 읽을 필드.  Mode 가 없는 초기 로그 (TDCN 안에서만 기록하던 때) 는 29 로 채운다.
FIELDS = {
    "TDST": ("TimeUS", "Mode", "St", "Stp", "PN", "PE", "PD",
             "VN", "VE", "VD", "Roll", "Pitch", "Yaw"),
    "TDTG": ("TimeUS", "Val", "PN", "PE", "PD", "Hdg"),
    "TDMX": ("TimeUS", "Act", "AR", "AP", "AY", "AT", "CR", "CP", "CY", "CT"),
}
DEFAULTS = {"Mode": TDCN_MODE}

# 요약 출력용 비행 모드 이름 (ArduCopter 번호)
MODE_NAMES = {
    0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
    6: "RTL", 9: "LAND", 16: "POSHOLD", 17: "BRAKE", 29: "TDCN",
}


# ---------------------------------------------------------------------------
# 스타일 — 세 장 공통.  같은 뜻이면 같은 색 / 선 / 범례 문구를 쓴다.
# ---------------------------------------------------------------------------

C_TARGET = "#222222"
C_AP = "#1f77b4"
C_CLAW = "#d62728"
C_OUT = "#9e9e9e"
C_STATE = "#7f7f7f"

LINE = {
    "target":  dict(color=C_TARGET, lw=1.6, ls="--", label="목표 (TDCN)"),
    "current": dict(color=C_AP,     lw=1.2, ls="-",  label="현재 (아두파일럿 EKF)"),
    "ap":      dict(color=C_AP,     lw=1.2, ls="-",  label="아두파일럿"),
    "claw":    dict(color=C_CLAW,   lw=1.2, ls="-",  label="CLAW"),
}
STATE_LINE = dict(color=C_STATE, lw=0.8, ls=":")
SHADE_OUT = dict(color=C_OUT, alpha=0.18, lw=0)
SHADE_ACT = dict(color=C_CLAW, alpha=0.10, lw=0)

FIGSIZE_4x1 = (12.5, 10.5)
FIGSIZE_3x3 = (16.0, 10.5)
XLABEL = "t (s, TDCN 진입 기준)"


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
    if name:
        sys.exit(f"{name} 을 찾을 수 없습니다.  찾아본 곳: {', '.join(LOG_DIRS)}")
    if not cands:
        sys.exit(f"로그를 찾을 수 없습니다.  찾아본 곳: {', '.join(LOG_DIRS)}")
    return max(cands, key=os.path.getmtime)


def read_log(path: str) -> dict:
    """TDST / TDTG / TDMX 만 뽑아 필드별 배열로 돌려준다.  시간은 s 로 바꾼다."""
    print(f"[log] 읽는 중: {path}")
    m = mavutil.mavlink_connection(path)
    cols = {k: {f: [] for f in fs} for k, fs in FIELDS.items()}

    while True:
        msg = m.recv_match(type=list(FIELDS))
        if msg is None:
            break
        t = msg.get_type()
        d = cols[t]
        for f in FIELDS[t]:
            d[f].append(getattr(msg, f, DEFAULTS.get(f, np.nan)))

    data = {}
    for k, d in cols.items():
        n = len(d["TimeUS"])
        print(f"[log]   {k}  {n:>7}개")
        if n == 0:
            continue
        arr = {f: np.asarray(v, dtype=float) for f, v in d.items()}
        arr["t"] = arr.pop("TimeUS") / 1e6
        data[k] = arr

    if "TDST" not in data:
        sys.exit("TDST 가 없습니다 — TDCN Part 3 로그 (mode_tdcn_log.cpp) 가 들어간 "
                 "펌웨어의 로그인지, TDCN_LOG_HZ 가 0 이 아닌지 확인하세요.")
    return data


def crop(data: dict, t0: float | None, t1: float | None) -> dict:
    """[t0, t1] 구간만 남긴다."""
    out = {}
    for k, d in data.items():
        sel = np.ones(d["t"].size, dtype=bool)
        if t0 is not None:
            sel &= d["t"] >= t0
        if t1 is not None:
            sel &= d["t"] <= t1
        out[k] = {f: v[sel] for f, v in d.items()}
    if out["TDST"]["t"].size == 0:
        sys.exit("선택한 시간 구간에 TDST 가 없습니다.")
    return out


def tdcn_window(data: dict) -> tuple[float, float]:
    """TDCN 첫 진입 ~ 마지막 이탈 (부팅 후 s).  이탈이 없으면 로그 끝까지."""
    s = data["TDST"]
    on = np.where(s["Mode"] == TDCN_MODE)[0]
    if on.size == 0:
        sys.exit("이 로그에는 TDCN 모드 (Mode 29) 구간이 없습니다.")
    return float(s["t"][on[0]]), float(s["t"][on[-1]])


def shift(data: dict, t_ref: float) -> None:
    """시간축을 t_ref 기준으로 옮긴다 (t_ref 가 0 초)."""
    for d in data.values():
        d["t"] = d["t"] - t_ref


# ---------------------------------------------------------------------------
# 구간 계산
# ---------------------------------------------------------------------------

def at_times(t_src, y_src, t_dst):
    """t_dst 각 시각의 값 (직전 샘플, zero-order hold)."""
    idx = np.searchsorted(t_src, t_dst, side="right") - 1
    idx = np.clip(idx, 0, len(t_src) - 1)
    return np.asarray(y_src)[idx]


def spans(t, mask) -> list[tuple[float, float]]:
    """mask 가 참인 연속 구간.  끝은 다음 샘플 시각까지 덮는다."""
    if t.size == 0:
        return []
    d = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
    starts = np.where(d == 1)[0]
    ends = np.where(d == -1)[0]
    return [(float(t[s]), float(t[min(e, t.size - 1)])) for s, e in zip(starts, ends)]


def context(data: dict) -> dict:
    """세 장이 공통으로 쓰는 배경 — TDCN 밖 / CLAW 구동 구간, 단계 전환."""
    s = data["TDST"]
    t, mode, st, stp = s["t"], s["Mode"], s["St"], s["Stp"]
    active = mode == TDCN_MODE

    # 단계 전환: TDCN 안에서 St 나 Stp 가 바뀐 곳과 TDCN 진입.  이탈은 음영이
    # 보여 준다.
    key_st = np.where(active, st, -1.0)
    key_stp = np.where(active, stp, -1.0)
    idx = np.where((np.diff(key_st) != 0) | (np.diff(key_stp) != 0))[0] + 1
    idx = idx[active[idx]]
    if active[0]:
        idx = np.concatenate([[0], idx]).astype(int)

    # 표시할 숫자는 새 단계 (Stp).  자동 진행 명령을 받은 곳 (St 만 12 / 13 으로
    # 바뀌고 단계는 그대로) 은 12 / 13 을 적는다.
    prev = np.maximum(idx - 1, 0)
    auto_start = (np.isin(st[idx], (12, 13)) & (key_st[prev] != st[idx])
                  & (key_stp[prev] == stp[idx]))
    tr_label = np.where(auto_start, st[idx], stp[idx])

    act_spans = []
    if "TDMX" in data:
        x = data["TDMX"]
        act_spans = spans(x["t"], x["Act"] > 0.5)

    return {
        "t0": float(t[0]), "t1": float(t[-1]),
        "out_spans": spans(t, ~active),
        "act_spans": act_spans,
        "tr_t": t[idx], "tr_st": st[idx], "tr_stp": stp[idx], "tr_label": tr_label,
        "hdg_lim": heading_limits(s["Yaw"][active] if np.any(active) else s["Yaw"]),
    }


def heading_limits(yaw) -> tuple[float, float]:
    """헤딩 축 범위.  평균 헤딩이 북쪽 (±90 도 안) 이면 -180~180, 아니면 0~360.

    0~360 으로 고정하면 북쪽 근처 헤딩이 0 / 360 테두리에 붙어 선이 가린다.
    """
    a = np.radians(np.asarray(yaw, dtype=float))
    a = a[np.isfinite(a)]
    if a.size and np.mean(np.cos(a)) > 0.0:     # 원형 평균이 북쪽 반구
        return -180.0, 180.0
    return 0.0, 360.0


def wrap_heading(y, lim: tuple[float, float]):
    """헤딩을 축 범위로 감고 경계를 넘는 자리를 끊는다."""
    y = np.asarray(y, dtype=float)
    y = (y + 180.0) % 360.0 - 180.0 if lim[0] < 0.0 else y % 360.0
    return break_wrap(y)


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


def legend_handles(lines: list[str]):
    """범례 — 선 (그림마다 다름) 다음에 배경 (세 장 공통) 을 같은 순서로."""
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    h = [Line2D([], [], **LINE[k]) for k in lines]
    h.append(Line2D([], [], label="단계 전환 (위 숫자 = 단계, 12·13 = 자동 진행 시작)",
                    **STATE_LINE))
    h.append(Patch(label="TDCN 밖 (Mode ≠ 29)", **SHADE_OUT))
    h.append(Patch(label="CLAW 구동 (Act = 1)", **SHADE_ACT))
    return h


def new_figure(plt, nrows: int, ncols: int, figsize, title: str, sub: str,
               lines: list[str]):
    fig, axes = plt.subplots(nrows, ncols, figsize=figsize, sharex=True,
                             squeeze=False)
    fig.suptitle(f"{title}\n{sub}", fontsize=12, y=0.985)
    fig.legend(handles=legend_handles(lines), loc="upper center",
               bbox_to_anchor=(0.5, 0.925), ncol=len(lines) + 3,
               fontsize=9, frameon=False)
    # 여백은 세 장이 같게 고정한다 (tight_layout 은 범례 아래에 빈 띠를 남긴다).
    # 3x3 은 열 제목 자리를 조금 더 둔다.
    fig.subplots_adjust(left=0.065, right=0.985, bottom=0.06,
                        top=0.83 if ncols > 1 else 0.865,
                        hspace=0.18, wspace=0.22)
    return fig, axes


def decorate(ax, ctx: dict, label_states: bool) -> None:
    """배경 (TDCN 밖 / CLAW 구동 음영, 단계 전환선) 과 격자."""
    for a, b in ctx["out_spans"]:
        ax.axvspan(a, b, zorder=0, **SHADE_OUT)
    for a, b in ctx["act_spans"]:
        ax.axvspan(a, b, zorder=0, **SHADE_ACT)

    # 숫자끼리 0.25 인치보다 가까우면 한 줄 위로 올려 겹치지 않게 한다
    width_in = ax.get_position().width * ax.figure.get_figwidth()
    min_dt = 0.25 / width_in * (ctx["t1"] - ctx["t0"])
    last_t, level = -np.inf, 0
    for tt, s in zip(ctx["tr_t"], ctx["tr_label"]):
        ax.axvline(tt, zorder=1, **STATE_LINE)
        if label_states:
            level = (level + 1) % 2 if tt - last_t < min_dt else 0
            last_t = tt
            ax.text(tt, 1.01 + 0.055 * level, f"{int(s)}",
                    transform=ax.get_xaxis_transform(),
                    ha="center", va="bottom", fontsize=7.5, color=C_STATE)
    ax.set_xlim(ctx["t0"], ctx["t1"])
    ax.grid(alpha=0.3)


def finish(fig, plt, save: str | None, k: int) -> None:
    """저장 모드면 파일로 쓰고 닫는다.  아니면 열어 둔 채 돌아간다.

    장마다 plt.show() 를 부르면 창을 하나 닫아야 다음 장이 뜬다.  세 장을
    한꺼번에 보려면 마지막에 한 번만 불러야 한다.
    """
    if save:
        root, ext = os.path.splitext(save)
        out = f"{root}_{k}{ext or '.png'}"
        d = os.path.dirname(os.path.abspath(out))
        os.makedirs(d, exist_ok=True)
        fig.savefig(out, dpi=130)
        print(f"[plot] 저장: {out}")
        plt.close(fig)


def equal_span_limits(series_list: list[np.ndarray], pad: float = 0.08):
    """여러 축을 같은 폭으로 맞춘다 (v3 와 같은 규칙).

    가장 변화폭이 큰 축을 기준으로 폭을 정하고, 각 축은 자기 데이터 중앙에
    그 폭을 씌운다.  폭이 같아야 축끼리 움직인 크기를 눈으로 비교할 수 있다.
    """
    ok = [s[np.isfinite(s)] for s in series_list]
    spans_ = [float(s.max() - s.min()) for s in ok if s.size]
    span = max(spans_) if spans_ else 1.0
    if span <= 0:
        span = 1.0
    span *= (1.0 + 2.0 * pad)
    lims = []
    for s in ok:
        mid = 0.5 * (float(s.max()) + float(s.min())) if s.size else 0.0
        lims.append((mid - span / 2.0, mid + span / 2.0))
    return lims, span


def break_wrap(y, limit: float = 180.0):
    """0/360 경계를 넘는 자리를 NaN 으로 끊는다.

    끊지 않으면 359.9 -> 0.1 이 한 점에서 위아래를 잇는 세로선으로 그려져,
    실제로는 없는 급변처럼 보인다.
    """
    y = np.asarray(y, dtype=float).copy()
    d = np.abs(np.diff(y))
    y[:-1][d > limit] = np.nan
    return y


def set_heading_axis(ax, lim: tuple[float, float]) -> None:
    ax.set_ylim(*lim)
    ax.set_yticks(np.linspace(lim[0], lim[1], 5))


def hdg_text(lim: tuple[float, float]) -> str:
    return f"{lim[0]:.0f}~{lim[1]:.0f}"


# ---------------------------------------------------------------------------
# Figure 1 — 현재 상태 (위치 / 속도 / 자세)
# ---------------------------------------------------------------------------

def figure1(data: dict, ctx: dict, sub: str, save: str | None) -> None:
    plt = setup_mpl()
    s = data["TDST"]
    t = s["t"]

    cols = (
        ("위치", (("PN", "North (m)"), ("PE", "East (m)"), ("PD", "Down (m)"))),
        ("속도", (("VN", "VN (m/s)"), ("VE", "VE (m/s)"), ("VD", "VD (m/s)"))),
        ("자세", (("Roll", "Roll (deg)"), ("Pitch", "Pitch (deg)"), ("Yaw", "Yaw (deg)"))),
    )

    pos_lims, pos_span = equal_span_limits([s["PN"], s["PE"], s["PD"]])
    vel_lims, vel_span = equal_span_limits([s["VN"], s["VE"], s["VD"]])
    att_lims, att_span = equal_span_limits([s["Roll"], s["Pitch"]])
    lims = (pos_lims, vel_lims, att_lims + [None])

    fig, axes = new_figure(
        plt, 3, 3, FIGSIZE_3x3,
        "[Figure 1] 현재 상태 — 위치 · 속도 · 자세 (TDST)",
        f"{sub}   |   N/E/D 폭 {pos_span:.1f} m, 속도 폭 {vel_span:.1f} m/s, "
        f"Roll/Pitch 폭 {att_span:.1f} deg 로 통일, Yaw 는 {hdg_text(ctx['hdg_lim'])} 고정",
        ["current"])

    for c, (col_title, rows) in enumerate(cols):
        for r, (key, ylabel) in enumerate(rows):
            ax = axes[r][c]
            decorate(ax, ctx, label_states=(r == 0))
            y = s[key]
            if key == "Yaw":
                y = wrap_heading(y, ctx["hdg_lim"])
            ax.plot(t, y, **LINE["current"])
            ax.set_ylabel(ylabel, fontsize=10)
            if key == "Yaw":
                set_heading_axis(ax, ctx["hdg_lim"])
            else:
                ax.set_ylim(*lims[c][r])
        axes[0][c].set_title(col_title, fontsize=11, pad=26)   # 단계 숫자 두 줄 위
        axes[-1][c].set_xlabel(XLABEL)

    finish(fig, plt, save, 1)


# ---------------------------------------------------------------------------
# Figure 2 — 목표 (위치 / 헤딩) + 현재
# ---------------------------------------------------------------------------

def figure2(data: dict, ctx: dict, sub: str, save: str | None) -> None:
    plt = setup_mpl()
    s = data["TDST"]
    g = data.get("TDTG")
    if g is None:
        print("[plot] TDTG 가 없어 Figure 2 를 건너뜁니다.")
        return

    # 목표 위치는 Val = 1 인 곳만, 목표 헤딩은 TDCN 안에서만 그린다
    valid = g["Val"] > 0.5
    in_tdcn = at_times(s["t"], s["Mode"], g["t"]) == TDCN_MODE
    tgt = {k: np.where(valid, g[k], np.nan) for k in ("PN", "PE", "PD")}
    tgt["Hdg"] = np.where(in_tdcn, g["Hdg"], np.nan)

    lims, span = equal_span_limits([
        np.concatenate([tgt["PN"], s["PN"]]),
        np.concatenate([tgt["PE"], s["PE"]]),
        np.concatenate([tgt["PD"], s["PD"]]),
    ])

    rows = (
        ("North (m)",     "PN",  "PN",  lims[0]),
        ("East (m)",      "PE",  "PE",  lims[1]),
        ("Down (m)",      "PD",  "PD",  lims[2]),
        ("Heading (deg)", "Hdg", "Yaw", None),
    )

    fig, axes = new_figure(
        plt, 4, 1, FIGSIZE_4x1,
        "[Figure 2] 목표 — 위치 · 헤딩 (TDTG), 현재값 (TDST) 겹침",
        f"{sub}   |   목표 위치는 Val = 1 (목표 있음) 인 구간만, "
        f"N/E/D 는 같은 폭 {span:.1f} m, Heading 은 {hdg_text(ctx['hdg_lim'])} 고정",
        ["target", "current"])

    for ax, (ylabel, tk, ck, ylim) in zip(axes[:, 0], rows):
        decorate(ax, ctx, label_states=(ax is axes[0][0]))
        yc, yt = s[ck], tgt[tk]
        if ylim is None:
            yc, yt = wrap_heading(yc, ctx["hdg_lim"]), wrap_heading(yt, ctx["hdg_lim"])
        ax.plot(s["t"], yc, **LINE["current"])
        ax.plot(g["t"], yt, **LINE["target"])
        ax.set_ylabel(ylabel, fontsize=10)
        if ylim is None:
            set_heading_axis(ax, ctx["hdg_lim"])
        else:
            ax.set_ylim(*ylim)

    axes[-1][0].set_xlabel(XLABEL)
    finish(fig, plt, save, 2)


# ---------------------------------------------------------------------------
# Figure 3 — 믹서 직전 제어값 (아두파일럿 vs CLAW)
# ---------------------------------------------------------------------------

def range_text(ax, y, color: str, name: str, row: int) -> None:
    """실제 사용 범위를 숫자로 적는다 (축은 정의역 고정이라 좁아 보인다)."""
    y = y[np.isfinite(y)]
    txt = f"{name}  {y.min():+.3f} ~ {y.max():+.3f}" if y.size else f"{name}  -"
    ax.text(0.995, 0.05 + 0.09 * row, txt, transform=ax.transAxes,
            ha="right", va="bottom", fontsize=8, color=color,
            bbox=dict(facecolor="white", edgecolor="none", alpha=0.7, pad=1.0))


def figure3(data: dict, ctx: dict, sub: str, save: str | None) -> None:
    plt = setup_mpl()
    x = data.get("TDMX")
    if x is None:
        print("[plot] TDMX 가 없어 Figure 3 을 건너뜁니다.")
        return
    t = x["t"]

    rows = (
        ("Roll",     "AR", "CR", (-1.0, 1.0)),
        ("Pitch",    "AP", "CP", (-1.0, 1.0)),
        ("Yaw",      "AY", "CY", (-1.0, 1.0)),
        ("Throttle", "AT", "CT", (0.0, 1.0)),
    )

    fig, axes = new_figure(
        plt, 4, 1, FIGSIZE_4x1,
        "[Figure 3] 믹서 직전 제어값 — 아두파일럿 vs CLAW (TDMX)",
        f"{sub}   |   y 축은 정의역으로 고정: Roll/Pitch/Yaw = -1~1, "
        f"Throttle = 0~1.  CLAW 는 state 6 밖에서 0",
        ["ap", "claw"])

    for ax, (name, ak, ck, ylim) in zip(axes[:, 0], rows):
        decorate(ax, ctx, label_states=(ax is axes[0][0]))
        ax.axhline(0.0, color="#888888", lw=0.8, ls=":", zorder=1)
        ax.plot(t, x[ck], **LINE["claw"])
        ax.plot(t, x[ak], **LINE["ap"])
        ax.set_ylabel(name, fontsize=10)
        pad = 0.05 * (ylim[1] - ylim[0]) / 2.0     # 정의역 끝 (±1 포화) 이 테두리에 가리지 않게
        ax.set_ylim(ylim[0] - pad, ylim[1] + pad)
        ax.set_yticks(np.linspace(ylim[0], ylim[1], 5))
        range_text(ax, x[ak], C_AP, LINE["ap"]["label"], 1)
        range_text(ax, x[ck], C_CLAW, LINE["claw"]["label"], 0)

    axes[-1][0].set_xlabel(XLABEL)
    finish(fig, plt, save, 3)


# ---------------------------------------------------------------------------
# 요약
# ---------------------------------------------------------------------------

def print_summary(path: str, data: dict, ctx: dict) -> None:
    s = data["TDST"]
    t, mode = s["t"], s["Mode"]
    dt = np.diff(t)
    hz = 1.0 / float(np.median(dt)) if dt.size else 0.0

    print()
    print("=" * 66)
    print(f"  {os.path.basename(path)}   {t[0]:.1f} ~ {t[-1]:.1f} s   "
          f"(TDST {t.size}개, {hz:.0f} Hz)")
    print(f"  시각은 TDCN 첫 진입 기준 (진입 = 부팅 후 {ctx['t_entry']:.1f} s)")
    print("-" * 66)
    print("  비행 모드")
    segs = sorted((a, b, int(m_)) for m_ in np.unique(mode)
                  for a, b in spans(t, mode == m_))
    for a, b, m_ in segs:
        print(f"    {a:7.1f} ~ {b:7.1f} s   Mode {m_:2d} {MODE_NAMES.get(m_, '?')}")

    print("  TDCN 단계 전환   (St = 명령 state, Stp = 실제 단계)")
    for tt, st, stp in zip(ctx["tr_t"], ctx["tr_st"], ctx["tr_stp"]):
        auto = "   (자동 진행)" if st in (12, 13) else ""
        print(f"    {tt:7.1f} s   St {int(st):2d}  Stp {int(stp):2d}{auto}")

    act = sum(b - a for a, b in ctx["act_spans"])
    print(f"  CLAW 구동 (Act = 1)   {act:.1f} s")

    # state 6 추종 오차 — 같은 루프에 기록된 목표와 현재를 뺀다
    g = data.get("TDTG")
    if g is not None:
        stp = at_times(t, s["Stp"], g["t"])
        md = at_times(t, mode, g["t"])
        sel = (g["Val"] > 0.5) & (stp == 6) & (md == TDCN_MODE)
        if np.any(sel):
            print("  state 6 추종 오차 (현재 - 목표)")
            print(f"    {'축':<12}{'RMS':>10}{'평균':>10}{'최대|오차|':>12}")
            tg = g["t"][sel]
            for name, ck, tk in (("North (m)", "PN", "PN"),
                                 ("East (m)", "PE", "PE"),
                                 ("Down (m)", "PD", "PD"),
                                 ("Heading(deg)", "Yaw", "Hdg")):
                d = at_times(t, s[ck], tg) - g[tk][sel]
                if tk == "Hdg":
                    d = (d + 180.0) % 360.0 - 180.0     # 원형이라 최단거리로
                print(f"    {name:<12}{np.sqrt(np.mean(d**2)):>10.3f}"
                      f"{np.mean(d):>10.3f}{np.max(np.abs(d)):>12.3f}")
    print("=" * 66)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="TDCN 로그 분석 (v4) — 현재 상태, 목표, 믹서 직전 제어값")
    ap.add_argument("log", nargs="?", help="로그 파일 (생략 시 가장 최근 .BIN)")
    ap.add_argument("--start", type=float, metavar="S",
                    help="시작 시각 (s, TDCN 진입 기준)")
    ap.add_argument("--end", type=float, metavar="S",
                    help="끝 시각 (s, TDCN 진입 기준)")
    ap.add_argument("--save", metavar="FILE",
                    help="플롯을 창 대신 파일로 저장 (FILE_1.png 등)")
    args = ap.parse_args(argv)

    path = find_log(args.log)
    data = read_log(path)

    # TDCN 첫 진입 ~ 마지막 이탈만, 첫 진입을 0 초로
    t_entry, t_exit = tdcn_window(data)
    data = crop(data, t_entry, t_exit)
    shift(data, t_entry)
    data = crop(data, args.start, args.end)

    ctx = context(data)
    ctx["t_entry"] = t_entry
    sub = (f"{os.path.basename(path)}   {ctx['t0']:.1f} ~ {ctx['t1']:.1f} s "
           f"(TDCN 진입 = 부팅 후 {t_entry:.1f} s)")

    print_summary(path, data, ctx)

    figure1(data, ctx, sub, args.save)
    figure2(data, ctx, sub, args.save)
    figure3(data, ctx, sub, args.save)

    if not args.save:
        print("[plot] Figure 1~3 을 함께 띄웁니다 — 모두 닫으면 종료합니다 ...")
        setup_mpl().show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
