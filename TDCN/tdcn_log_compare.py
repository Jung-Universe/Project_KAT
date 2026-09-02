#!/usr/bin/env python3
"""TDCN 모드 로그 분석.

state 6 (추종 비행) 구간을 대상으로 한다.  그림을 하나씩 늘려 가는 구조다.

  Figure 1  TDCN -> CLAW 인터페이스 검증
            Update_Info_for_CLAW() 가 넘긴 값과 CLAW_step() 이 내부 변수에 넣은
            값을 짝지어 본다.  같은 스텝끼리 비교하도록 펌웨어에서 넘긴 값은
            CLAW_step() 직전에, CLAW 내부값은 직후에 읽어 기록한다.

            CLAW 가 의도적으로 바꾸는 곳이 있어 그것만 차이가 나야 정상이다.
              - 고도    up -> down 부호 반전 (로그에서 이미 맞춰 둠)
              - 헤딩    wrapToPi
              - 타겟    home 을 래치하는 그 한 스텝만 현재 위치로 덮어씀

사용법
    ./tdcn_log_compare.py                     # 최근 로그 자동 탐색
    ./tdcn_log_compare.py logs/00000006.BIN   # 파일 지정
    ./tdcn_log_compare.py --save out/         # PNG 저장 (창 안 띄움)
"""

import argparse
import glob
import os
import sys

import numpy as np

TDCN_C, CLAW_C = "#1f77b4", "#d62728"      # mode_tdcn.cpp 파랑, CLAW 빨강

LOG_DIRS = (
    "logs",
    os.path.expanduser("~/Desktop/KAT/ardupilot/logs"),
)

# 인터페이스 검증 대상.
#   (메시지, 제목, [(TDCN 필드, CLAW 필드, 이름, 단위, 차이 표시 배율, 차이 단위)])
#
# 위경도는 값이 35 / 149 처럼 커서 두 선을 그대로 보면 차이가 안 보인다.
# 그래서 차이는 1e-7 deg (Location 의 int32 한 칸, 약 1.1 cm) 단위로 표시한다.
GROUPS = (
    ("TDCP", "현재 위치   Cur_Pos -> Cur_Lat / Cur_Lon / Cur_Alt", (
        ("TLat", "CLat", "위도",      "deg", 1e7, "1e-7deg"),
        ("TLng", "CLng", "경도",      "deg", 1e7, "1e-7deg"),
        ("TDwn", "CDwn", "고도 Down", "m",   1.0, "m"),
    )),
    ("TDCT", "타겟   Dest_poti_i -> Dest_Lat / Dest_Lon / Dest_Alt,  Ship_heading -> ps_cmd", (
        ("TLat", "CLat", "위도",      "deg", 1e7, "1e-7deg"),
        ("TLng", "CLng", "경도",      "deg", 1e7, "1e-7deg"),
        ("TDwn", "CDwn", "고도 Down", "m",   1.0, "m"),
        ("THdg", "CHdg", "헤딩",      "deg", 1.0, "deg"),
    )),
    ("TDCI", "IMU   p,q,r / Roll,Pitch,DR_heading -> STV[3..8]", (
        ("Tp",   "Cp",   "p 롤각속도",   "rad/s", 1.0, "rad/s"),
        ("Tq",   "Cq",   "q 피치각속도", "rad/s", 1.0, "rad/s"),
        ("Tr",   "Cr",   "r 요각속도",   "rad/s", 1.0, "rad/s"),
        ("TRol", "CRol", "Roll",         "rad",   1.0, "rad"),
        ("TPit", "CPit", "Pitch",        "rad",   1.0, "rad"),
        # DR_heading -> STV[8].  현재 헤딩이라 IMU 가 아니라 현재 상태 열에 놓는다.
        ("TYaw", "CYaw", "현재 헤딩",    "rad",   1.0, "rad"),
    )),
    ("TDCV", "속도   XTV[0..2]  (NED)", (
        ("TVN", "CVN", "북 N", "m/s", 1.0, "m/s"),
        ("TVE", "CVE", "동 E", "m/s", 1.0, "m/s"),
        ("TVD", "CVD", "하 D", "m/s", 1.0, "m/s"),
    )),
)

# Figure 2 - NED 위치 비교.
#   아두파일럿은 EKF 가 낸 NED, CLAW 는 위경도를 받아 Lat2m/Lon2m 으로 다시 만든 NED.
#   원점이 다르다 (아두파일럿 = EKF origin, CLAW = state 6 진입 위치).  제자리
#   이륙 후 넘어오므로 수평은 거의 같고 고도만 진입 고도만큼 오프셋이 있다.
FIG2_MSG = "TDCL"
FIG2_ROWS = (
    ("AN", "CN", "North 북", "m"),
    ("AE", "CE", "East 동",  "m"),
    ("AD", "CD", "Down 하",  "m"),
)

# Figure 3 - 제어값 비교.
#   범위가 다르다.  아두파일럿은 롤/피치/요가 -1~+1, 스로틀만 0~+1 이고,
#   CLAW 는 네 개 모두 -1~+1 이다.  그래서 CLAW 스로틀만 (x+1)/2 로 0~+1 에
#   맞춘다 (-1 -> 0, 0 -> 0.5, +1 -> 1).  나머지 세 축은 손대지 않는다.
#   (아두파일럿 필드, CLAW 필드, 이름, CLAW 배율, CLAW 오프셋)
FIG3_MSG = "TDCC"
FIG3_ROWS = (
    ("MR", "CR", "Roll 롤",       1.0, 0.0),
    ("MP", "CP", "Pitch 피치",    1.0, 0.0),
    ("MY", "CY", "Yaw 요",        1.0, 0.0),
    ("MT", "CH", "Throttle 스로틀", 0.5, 0.5),
)


# ---------------------------------------------------------------------------
# 로그 읽기
# ---------------------------------------------------------------------------

def find_log():
    """로그를 흔한 위치에서 찾는다.

    각 디렉터리에서는 LASTLOG.TXT 를 먼저 본다.  아두파일럿이 마지막 로그 번호를
    적어 두는 파일이라 이게 정확한 기준이다.  수정시각으로 고르면 틀릴 수 있다 -
    실기체 로그는 RTC 가 없어 1980 년으로 찍히고, SD 에서 복사하면 mtime 이
    복사 시각으로 바뀐다."""
    cands = []
    for dirpath in LOG_DIRS:
        if not os.path.isdir(dirpath):
            continue
        chosen = None
        last = os.path.join(dirpath, "LASTLOG.TXT")
        if os.path.isfile(last):
            try:
                with open(last) as f:
                    num = int(f.read().strip())
                path = os.path.join(dirpath, f"{num:08d}.BIN")
                if os.path.isfile(path):
                    chosen = path
            except (ValueError, OSError):
                pass
        if chosen is None:
            bins = glob.glob(os.path.join(dirpath, "*.BIN"))
            if bins:
                chosen = max(bins, key=os.path.getmtime)
        if chosen:
            cands.append(chosen)

    if not cands:
        sys.exit("BIN 로그를 못 찾았다.  경로를 직접 지정할 것.\n"
                 "  찾아본 곳: " + ", ".join(LOG_DIRS))
    return max(cands, key=os.path.getmtime)


def load(path):
    """BIN -> {메시지: {필드: ndarray}}.

    't' 는 그 메시지의 첫 샘플 기준 상대 초다.  TDC* 는 state 6 에서만 기록되므로
    t=0 이 곧 state 6 진입 시점이 된다."""
    try:
        from pymavlink import mavutil
    except ImportError:
        sys.exit("pymavlink 가 없다:  pip install pymavlink")

    names = [g[0] for g in GROUPS] + [FIG2_MSG, FIG3_MSG]
    conn = mavutil.mavlink_connection(path)
    raw = {m: [] for m in names}
    while True:
        msg = conn.recv_match(type=names)
        if msg is None:
            break
        raw[msg.get_type()].append(msg.to_dict())

    out = {}
    for name, rows in raw.items():
        if not rows:
            continue
        d = {k: np.array([r[k] for r in rows], dtype=float) for k in rows[0]
             if k != "mavpackettype"}
        d["t"] = (d["TimeUS"] - d["TimeUS"][0]) / 1e6
        out[name] = d

    if not out:
        sys.exit(f"{path} 에 TDC* 메시지가 없다.\n"
                 "  state 6 (추종 비행) 까지 진행한 로그인지 확인할 것.")

    # 필드 확인.  로그 포맷을 바꾸는 중이라 옛 로그가 섞이기 쉽다.
    for name, _title, rows in GROUPS:
        if name not in out:
            continue
        need = [f for pair in rows for f in pair[:2]]
        missing = [f for f in need if f not in out[name]]
        if missing:
            sys.exit(f"{path}\n"
                     f"  {name} 에 {', '.join(missing)} 가 없다 - 로그 포맷 "
                     f"변경 전 펌웨어로 찍은 로그다.\n"
                     f"  있는 필드: "
                     f"{', '.join(k for k in out[name] if k != 't')}\n"
                     f"  ./waf copter 로 다시 빌드한 뒤 새로 비행할 것.")
    return out


# ---------------------------------------------------------------------------
# Figure 1 - 인터페이스 검증
# ---------------------------------------------------------------------------

def setup_mpl(save):
    import matplotlib
    if save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager as fm
    for cand in ("NanumGothic", "Malgun Gothic", "Noto Sans CJK KR",
                 "Noto Sans CJK JP"):
        if any(f.name == cand for f in fm.fontManager.ttflist):
            plt.rcParams["font.family"] = cand
            break
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["figure.autolayout"] = True
    return plt


def diff_of(q, tk, ck, mul):
    """두 값의 차이.  헤딩은 ±180 경계를 넘을 수 있어 최단거리로 뺀다."""
    if "Hdg" in tk:
        dh = np.radians(q[tk] - q[ck])
        return np.degrees(np.arctan2(np.sin(dh), np.cos(dh)))
    return (q[tk] - q[ck]) * mul


def fig1(plt, d):
    """Figure 1 - 인터페이스 검증 전체를 한 장에.

    열이 그룹이고 (IMU / 현재 상태 / 목표), 칸마다 두 값을 겹쳐 그린다.
    값 자체는 스케일이 커서 눈으로는 늘 겹쳐 보이므로, 칸 제목에 최대 차이와
    0 이 아닌 샘플 수를 같이 적어 그것으로 판정한다."""
    # 열 구성: (열 제목, [(메시지, TDCN 필드, CLAW 필드, 이름, 단위, 배율, 차이단위)])
    cols = []
    by_name = {g[0]: g for g in GROUPS}

    def take(name, idx=None):
        if name not in d:
            return []
        _n, _t, rows = by_name[name]
        rows = rows if idx is None else [rows[i] for i in idx]
        return [(name,) + r for r in rows]

    # 현재 헤딩(DR_heading -> STV[8])은 IMU 가 아니라 현재 상태로 묶는다.
    cols.append(("IMU   p,q,r / Roll,Pitch  ->  STV[3..7]",
                 take("TDCI", [0, 1, 2, 3, 4])))
    cols.append(("현재 위치 · 헤딩 · 속도   Cur_Pos -> Cur_*,  "
                 "DR_heading -> STV[8],  XTV[0..2]",
                 take("TDCP") + take("TDCI", [5]) + take("TDCV")))
    cols.append(("목표 위치 · 헤딩   Dest_poti_i -> Dest_*,  Ship_heading -> ps_cmd",
                 take("TDCT")))

    nrow = max(len(c[1]) for c in cols)
    ncol = len(cols)
    fig, axs = plt.subplots(nrow, ncol, figsize=(6.0 * ncol, 2.1 * nrow + 1.2),
                            squeeze=False)
    fig.suptitle("Figure 1   TDCN 이 넘긴 값  vs  CLAW 가 받은 값"
                 "   (state 6 구간)", fontsize=14)

    for c, (ctitle, items) in enumerate(cols):
        for r in range(nrow):
            ax = axs[r][c]
            if r >= len(items):
                ax.axis("off")
                continue
            msg, tk, ck, nm, unit, mul, dunit = items[r]
            q = d[msg]
            # CLAW 를 먼저(아래) 그리고 TDCN 을 위에 올린다.  CLAW 값이 커서
            # 위에 그리면 TDCN 값을 덮어 가린다.
            ax.plot(q["t"], q[ck], color=CLAW_C, lw=2.4, alpha=0.55,
                    label="CLAW 가 받은 값")
            ax.plot(q["t"], q[tk], color=TDCN_C, lw=1.0, ls="--",
                    label="TDCN 이 넘긴 값")

            # 차이가 있을 때만 적는다.  없으면 아무 표시도 하지 않는다.
            dif = diff_of(q, tk, ck, mul)
            nz = int(np.count_nonzero(dif))
            tag = "" if nz == 0 else \
                f"   차이 {nz}샘플 (최대 {np.abs(dif).max():.3g} {dunit})"
            ax.set_title(f"{nm}  [{unit}]{tag}", fontsize=9.5)
            ax.set_ylabel(unit, fontsize=8)
            ax.grid(alpha=0.3)
            ax.tick_params(labelsize=8)
            if r == 0:
                ax.legend(fontsize=7.5, loc="upper right")
            if r == len(items) - 1:
                ax.set_xlabel("state 6 진입 후 시간 [s]", fontsize=8)

        # 열 제목은 그 열 첫 칸 위에 붙인다
        axs[0][c].annotate(ctitle, xy=(0.5, 1.32), xycoords="axes fraction",
                           ha="center", va="bottom", fontsize=10.5,
                           fontweight="bold")
    return fig


def fig2(plt, d):
    """Figure 2 - NED 위치.  아두파일럿 EKF 계산 vs CLAW 위경도 환산.

    왼쪽에 두 값을 겹쳐 그리고 오른쪽에 차이를 그린다.  원점이 달라서 (고도는
    state 6 진입 고도만큼) 차이에 상수 오프셋이 얹혀 있다.  변환식 차이는
    거리에 비례해 커지므로, 상수분과 기울기를 나눠 제목에 적는다."""
    q = d[FIG2_MSG]
    fig, axs = plt.subplots(3, 2, figsize=(14, 8), sharex=True,
                            gridspec_kw={"width_ratios": [1.6, 1]})
    fig.suptitle("Figure 2   NED 위치   아두파일럿(EKF) vs CLAW(위경도 -> Lat2m/Lon2m)"
                 "   [원점 다름 - 고도는 진입 고도만큼 오프셋]", fontsize=12)

    for i, (ak, ck, nm, unit) in enumerate(FIG2_ROWS):
        ax = axs[i][0]
        # CLAW 를 먼저(아래), 아두파일럿을 위에 올린다.
        ax.plot(q["t"], q[ck], color=CLAW_C, lw=2.0, alpha=0.65, label="CLAW")
        ax.plot(q["t"], q[ak], color=TDCN_C, lw=1.0, ls="--", label="아두파일럿")
        ax.set_title(f"{nm}  [{unit}]", fontsize=10)
        ax.set_ylabel(unit, fontsize=9)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")

        ax = axs[i][1]
        dif = q[ak] - q[ck]
        ax.plot(q["t"], dif, color="k", lw=0.9)
        ax.axhline(0, color="gray", lw=0.8)
        # 상수 오프셋(원점 차이)과 거리 비례분(변환식 차이)을 나눈다
        if q[ck].std() > 1e-6:
            slope, const = np.polyfit(q[ck], dif, 1)
            tag = f"오프셋 {const:+.3f}{unit},  기울기 {slope*100:+.3f}%"
        else:
            tag = f"평균 {dif.mean():+.3f}{unit}"
        ax.set_title(f"차이 (아두파일럿 - CLAW)   {tag}", fontsize=9)
        ax.set_ylabel(unit, fontsize=9)
        ax.grid(alpha=0.3)

    for ax in axs[-1]:
        ax.set_xlabel("state 6 진입 후 시간 [s]")
    return fig


def fig2_summary(d):
    if FIG2_MSG not in d:
        return
    q = d[FIG2_MSG]
    print(f"\n{'='*78}\nFigure 2 - NED 위치  아두파일럿(EKF) vs CLAW(위경도 환산)\n{'='*78}")
    print(f"  {'축':8s} {'AP 평균':>11s} {'CLAW 평균':>11s} "
          f"{'오프셋':>11s} {'기울기':>10s} {'오프셋 뺀 잔차':>14s}")
    for ak, ck, nm, unit in FIG2_ROWS:
        dif = q[ak] - q[ck]
        if q[ck].std() > 1e-6:
            slope, const = np.polyfit(q[ck], dif, 1)
            resid = dif - (slope * q[ck] + const)
        else:
            slope, const, resid = 0.0, dif.mean(), dif - dif.mean()
        print(f"  {nm:8s} {q[ak].mean():+11.3f} {q[ck].mean():+11.3f} "
              f"{const:+10.3f}m {slope*100:+9.3f}% {np.abs(resid).max():13.4f}m")
    print("\n  오프셋 = 원점 차이.  고도는 state 6 진입 고도만큼 나오는 게 정상이다.")
    print("  기울기 = 변환식 차이.  CLAW 는 WGS84 급수식, 아두파일럿은 구면 근사라")
    print("           위도 -0.3%, 경도 +0.1% 수준이면 정상이다.")
    print("  잔차   = 그 둘로 설명 안 되는 부분.  0 에 가까워야 한다.")


def fig3(plt, d):
    """Figure 3 - 제어값 4개.  아두파일럿 출력 vs CLAW 출력.

    CLAW 스로틀만 (x+1)/2 로 0~1 에 맞춰 아두파일럿과 같은 축에 놓는다.
    나머지 세 축은 양쪽 다 -1~+1 이라 그대로 겹친다.

    [주의] 두 제어기는 제어 법칙이 다르므로 값이 같을 이유가 없다.  Figure 1,2
    처럼 '일치해야 정상' 인 비교가 아니라, 방향과 크기가 납득되는지를 본다."""
    q = d[FIG3_MSG]
    fig, axs = plt.subplots(4, 2, figsize=(14, 10), sharex=True,
                            gridspec_kw={"width_ratios": [1.6, 1]})
    fig.suptitle("Figure 3   제어값   아두파일럿 vs CLAW"
                 "   [CLAW 스로틀은 (x+1)/2 로 0~1 환산]", fontsize=12)

    for i, (ak, ck, nm, sc, off) in enumerate(FIG3_ROWS):
        claw = q[ck] * sc + off
        rng = "0 ~ 1" if off else "-1 ~ +1"

        ax = axs[i][0]
        # CLAW 를 먼저(아래), 아두파일럿을 위에 올린다.
        ax.plot(q["t"], claw, color=CLAW_C, lw=2.0, alpha=0.55, label="CLAW")
        ax.plot(q["t"], q[ak], color=TDCN_C, lw=1.0, ls="--", label="아두파일럿")
        ax.set_title(f"{nm}   [{rng}]", fontsize=10)
        ax.set_ylabel("정규화", fontsize=9)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")

        ax = axs[i][1]
        dif = q[ak] - claw
        ax.plot(q["t"], dif, color="k", lw=0.8)
        ax.axhline(0, color="gray", lw=0.8)
        ax.set_title(f"차이 (아두파일럿 - CLAW)   평균 {dif.mean():+.4f}, "
                     f"최대 |{np.abs(dif).max():.3f}|", fontsize=9)
        ax.set_ylabel("정규화", fontsize=9)
        ax.grid(alpha=0.3)

    for ax in axs[-1]:
        ax.set_xlabel("state 6 진입 후 시간 [s]")
    return fig


def fig3_summary(d):
    if FIG3_MSG not in d:
        return
    q = d[FIG3_MSG]
    print(f"\n{'='*78}\nFigure 3 - 제어값  아두파일럿 vs CLAW\n{'='*78}")
    print(f"  {'축':14s} {'AP 평균':>10s} {'CLAW 평균':>10s} "
          f"{'AP 피크':>9s} {'CLAW 피크':>10s} {'상관':>7s}")
    for ak, ck, nm, sc, off in FIG3_ROWS:
        claw = q[ck] * sc + off
        r = (np.corrcoef(q[ak], claw)[0, 1]
             if q[ak].std() > 1e-9 and claw.std() > 1e-9 else float("nan"))
        print(f"  {nm:14s} {q[ak].mean():+10.4f} {claw.mean():+10.4f} "
              f"{np.abs(q[ak]).max():9.3f} {np.abs(claw).max():10.3f} {r:+7.3f}")
    print("\n  두 제어기는 제어 법칙이 다르므로 값이 같을 이유가 없다.")
    print("  일치가 아니라 방향(상관)과 크기(피크)가 납득되는지를 본다.")
    print("  CLAW 스로틀은 (x+1)/2 환산이라 무입력이 0.5 에 해당한다.")


def summary(d):
    """차이를 수치로 낸다.  그림에서 겹쳐 보이는 것과 실제로 0 인 것은 다르다.

    0 아닌 샘플 수를 같이 낸다.  CLAW 는 home 을 래치하는 한 스텝에서만 타겟을
    덮어쓰므로, 그 항목만 '1 개' 로 나와야 정상이다."""
    print(f"\n{'='*78}\nFigure 1 - TDCN -> CLAW 인터페이스 검증\n{'='*78}")
    for name, title, rows in GROUPS:
        if name not in d:
            print(f"\n[{name}] 로그에 없음")
            continue
        q = d[name]
        print(f"\n[{name}]  {title}")
        print(f"  {'항목':12s} {'TDCN 평균':>18s} {'CLAW 평균':>18s} "
              f"{'최대차이':>12s} {'0아닌샘플':>9s}")
        for tk, ck, nm, unit, mul, dunit in rows:
            dif = diff_of(q, tk, ck, mul)
            nz = int(np.count_nonzero(dif))
            mark = "" if nz == 0 else ("  <- 래치 스텝" if nz == 1 else "  <- 확인")
            print(f"  {nm:12s} {q[tk].mean():+18.7f} {q[ck].mean():+18.7f} "
                  f"{np.abs(dif).max():11.6f} {nz:9d}{mark}")

    print("\n  대입만 하는 값은 모든 스텝에서 정확히 0 이어야 한다.")
    print("  타겟만 예외 - CLAW 가 home 을 래치하는 한 스텝에서 Dest_poti_i 를")
    print("  현재 위치로 덮어쓴다 (CLAW.c L187-189).  그래서 1 개는 정상이다.")
    print("  1e-7 deg 는 Location 의 int32 한 칸으로 약 1.1 cm 다.")


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="TDCN 로그 분석")
    ap.add_argument("log", nargs="?", help="BIN 파일 (생략하면 최근 것 자동 탐색)")
    ap.add_argument("--save", metavar="DIR", help="PNG 로 저장하고 창은 안 띄운다")
    args = ap.parse_args()

    path = args.log or find_log()
    print(f"로그: {path}")
    d = load(path)
    print("메시지: " + ", ".join(f"{k}({len(v['t'])}행)" for k, v in d.items()))
    any_msg = next(iter(d.values()))
    print(f"state 6 구간: {any_msg['t'][-1]:.1f} 초")

    summary(d)
    fig2_summary(d)
    fig3_summary(d)

    plt = setup_mpl(args.save)
    figs = [("fig1", fig1(plt, d))]
    if FIG2_MSG in d:
        figs.append(("fig2", fig2(plt, d)))
    if FIG3_MSG in d:
        figs.append(("fig3", fig3(plt, d)))

    if args.save:
        os.makedirs(args.save, exist_ok=True)
        for name, fig in figs:
            out = os.path.join(args.save, f"tdcn_{name}.png")
            fig.savefig(out, dpi=120, bbox_inches="tight")
            print(f"저장: {out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
