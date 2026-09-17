#!/usr/bin/env python3
"""
CLAW 제어값 로그 간 비교  /* Sejong */

두 로그에서 CLAW 의 제어 출력 4개(TDCC 의 CR/CP/CY/CH)만 뽑아 나란히 그린다.
게인이나 TDCN_CLAW_ON_OFF 를 하나만 바꿔 두 번 비행했을 때, CLAW 출력이
어떻게 달라졌는지 보기 위한 것이다.

tdcn_log_compare.py 의 Figure 3 은 [한 로그 안에서 아두파일럿 vs CLAW] 를
비교한다.  이 스크립트는 [두 로그의 CLAW vs CLAW] 를 비교한다.

시간축은 각 로그의 TDCC 첫 샘플 기준 상대 초다.  TDC* 는 state 6 에서만
기록되므로 t=0 이 곧 state 6 진입 시점이고, 두 로그를 그 시점에 맞춰 겹친다.

스로틀 환산은 Figure 3 과 같게 (x+1)/2 를 쓴다.  펌웨어의
ModeTDCN::output_to_motors() 도 같은 식이므로, 그려지는 값이 곧 믹서 입력이다.

사용 예
-------
    ./tdcn_claw_compare.py 12 13
    ./tdcn_claw_compare.py 12 13 --save out/
    ./tdcn_claw_compare.py ../logs/00000012.BIN ../logs/00000013.BIN
"""

import argparse
import os
import sys

import numpy as np

# 로그 A 파랑, 로그 B 주황.  tdcn_log_compare.py 의 파랑/빨강과 구분해
# "같은 CLAW 를 두 조건에서 본 것" 임을 색으로도 드러낸다.
A_C, B_C = "#1f77b4", "#ff7f0e"

LOG_DIRS = (
    os.path.expanduser("~/Desktop/KAT/ardupilot/logs"),
)

MSG = "TDCC"

# (CLAW 필드, 이름, 배율, 오프셋, 표시 범위)
ROWS = (
    ("CR", "Roll 롤",        1.0, 0.0, "-1 ~ +1"),
    ("CP", "Pitch 피치",     1.0, 0.0, "-1 ~ +1"),
    ("CY", "Yaw 요",         1.0, 0.0, "-1 ~ +1"),
    ("CH", "Throttle 스로틀", 0.5, 0.5, "0 ~ 1"),
)

# 참고로 함께 읽는 파라미터.  제목에 찍어 무엇이 달랐는지 남긴다.
PARM_KEYS = ("TDCN_CLAW_ON_OFF", "CLAW_SCALE_TH", "CLAW_SCALE_R",
             "CLAW_SCALE_P", "CLAW_SCALE_Y")


def resolve(token):
    """'12' 나 '00000012' 는 로그 번호로, 그 외는 경로로 본다."""
    if os.path.isfile(token):
        return token
    if token.isdigit():
        name = f"{int(token):08d}.BIN"
        for dirpath in LOG_DIRS:
            path = os.path.join(dirpath, name)
            if os.path.isfile(path):
                return path
        sys.exit(f"로그 {token} 을 못 찾았다.\n"
                 f"  찾아본 곳: " + ", ".join(LOG_DIRS))
    sys.exit(f"{token} 은 파일도 아니고 로그 번호도 아니다.")


def load(path):
    """TDCC 와 PARM 만 읽는다."""
    try:
        from pymavlink import mavutil
    except ImportError:
        sys.exit("pymavlink 가 없다:  pip install pymavlink")

    conn = mavutil.mavlink_connection(path)
    rows, parm = [], {}
    while True:
        msg = conn.recv_match(type=[MSG, "PARM"])
        if msg is None:
            break
        if msg.get_type() == "PARM":
            parm.setdefault(msg.Name, msg.Value)   # 첫 값 = 로그 시작 시점
        else:
            rows.append(msg.to_dict())

    if not rows:
        sys.exit(f"{path} 에 {MSG} 가 없다.\n"
                 "  state 6 (추종 비행) 까지 진행한 로그인지 확인할 것.")

    d = {k: np.array([r[k] for r in rows], dtype=float) for k in rows[0]
         if k != "mavpackettype"}
    d["t"] = (d["TimeUS"] - d["TimeUS"][0]) / 1e6

    missing = [f for f, *_ in ROWS if f not in d]
    if missing:
        sys.exit(f"{path}\n  {MSG} 에 {', '.join(missing)} 가 없다 - "
                 "로그 포맷 변경 전 펌웨어로 찍은 로그다.")
    return d, parm


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


def label_of(tag, parm):
    """범례에 쓸 이름.  ON_OFF 는 해석에 결정적이므로 항상 붙인다."""
    onoff = parm.get("TDCN_CLAW_ON_OFF")
    if onoff is None:
        return f"log {tag}"
    return f"log {tag}  (ON_OFF={int(onoff)})"


def parm_diff_line(pa, pb, ta, tb):
    """두 로그에서 다른 파라미터만 한 줄로."""
    diffs = []
    for k in PARM_KEYS:
        va, vb = pa.get(k), pb.get(k)
        if va is None or vb is None:
            continue
        if abs(va - vb) > 1e-9:
            diffs.append(f"{k}: {va:g} → {vb:g}")
    if not diffs:
        return f"log {ta} 과 log {tb} 의 CLAW 파라미터는 동일"
    return f"다른 파라미터  " + "   |   ".join(diffs)


def stats(v):
    return (float(np.mean(v)), float(np.std(v)),
            float(np.max(np.abs(v))),
            100.0 * float(np.count_nonzero(np.abs(v) >= 0.999)) / len(v))


def main():
    ap = argparse.ArgumentParser(
        description="두 로그의 CLAW 제어값 비교 (TDCC 의 CR/CP/CY/CH)")
    ap.add_argument("log_a", help="로그 번호 또는 BIN 경로")
    ap.add_argument("log_b", help="로그 번호 또는 BIN 경로")
    ap.add_argument("--save", metavar="DIR",
                    help="PNG 로 저장 (창을 띄우지 않는다)")
    ap.add_argument("--act-only", action="store_true",
                    help="ACT=1 (CLAW 가 실제로 모터를 몰던) 구간만 그린다")
    args = ap.parse_args()

    pa_path, pb_path = resolve(args.log_a), resolve(args.log_b)
    da, parm_a = load(pa_path)
    db, parm_b = load(pb_path)
    ta = os.path.basename(pa_path).replace(".BIN", "").lstrip("0") or "0"
    tb = os.path.basename(pb_path).replace(".BIN", "").lstrip("0") or "0"

    if args.act_only:
        for d, tag in ((da, ta), (db, tb)):
            if "ACT" not in d:
                sys.exit(f"log {tag} 에 ACT 필드가 없다.")
            keep = d["ACT"] > 0.5
            if not keep.any():
                sys.exit(f"log {tag} 에 ACT=1 구간이 없다 (인계가 없었다).")
            for k in list(d):
                d[k] = d[k][keep]
            d["t"] = d["t"] - d["t"][0]

    plt = setup_mpl(args.save)
    fig, axs = plt.subplots(len(ROWS), 1, figsize=(13, 11), sharex=True)

    head = "CLAW 제어값 비교   " + label_of(ta, parm_a) + "  vs  " + label_of(tb, parm_b)
    if args.act_only:
        head += "   [ACT=1 구간만]"
    fig.suptitle(head + "\n" + parm_diff_line(parm_a, parm_b, ta, tb),
                 fontsize=12)

    for i, (fld, nm, sc, off, rng) in enumerate(ROWS):
        ax = axs[i]
        va = da[fld] * sc + off
        vb = db[fld] * sc + off
        # 값이 큰 쪽을 아래(굵고 반투명), 작은 쪽을 위(얇은 점선)에 둔다.
        # 겹칠 때 위 선이 아래 선을 가리지 않게 한다.
        if np.std(va) >= np.std(vb):
            ax.plot(da["t"], va, color=A_C, lw=2.0, alpha=0.55,
                    label=label_of(ta, parm_a))
            ax.plot(db["t"], vb, color=B_C, lw=1.0, ls="--",
                    label=label_of(tb, parm_b))
        else:
            ax.plot(db["t"], vb, color=B_C, lw=2.0, alpha=0.55,
                    label=label_of(tb, parm_b))
            ax.plot(da["t"], va, color=A_C, lw=1.0, ls="--",
                    label=label_of(ta, parm_a))

        sa, sb = stats(va), stats(vb)
        ax.set_title(
            f"{nm}   [{rng}]      "
            f"log {ta}: 평균 {sa[0]:+.3f}  σ {sa[1]:.3f}  피크 {sa[2]:.3f}  포화 {sa[3]:.0f}%"
            f"      log {tb}: 평균 {sb[0]:+.3f}  σ {sb[1]:.3f}  피크 {sb[2]:.3f}  포화 {sb[3]:.0f}%",
            fontsize=9.5)
        ax.set_ylabel("정규화", fontsize=9)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="upper right")
        if off == 0.0:
            ax.axhline(0.0, color="k", lw=0.5, alpha=0.3)

    axs[-1].set_xlabel("state 6 진입 후 시간 [s]", fontsize=9)

    # 표로도 남긴다.  그래프에서 눈으로 재기 어려운 크기 비교용.
    print(f"\nlog {ta}  {pa_path}   {len(da['t'])} 샘플, {da['t'][-1]:.1f}s")
    print(f"log {tb}  {pb_path}   {len(db['t'])} 샘플, {db['t'][-1]:.1f}s")
    print(f"\n{parm_diff_line(parm_a, parm_b, ta, tb)}\n")
    print(f"  {'축':<14}{'':>4}{'평균':>9}{'표준편차':>10}{'피크':>9}{'포화':>7}")
    for fld, nm, sc, off, _rng in ROWS:
        for tag, d in ((ta, da), (tb, db)):
            s = stats(d[fld] * sc + off)
            print(f"  {nm:<14}{tag:>4}{s[0]:+9.4f}{s[1]:10.4f}{s[2]:9.3f}{s[3]:6.0f}%")
        print()

    if args.save:
        os.makedirs(args.save, exist_ok=True)
        out = os.path.join(args.save, f"claw_{ta}_vs_{tb}.png")
        fig.savefig(out, dpi=130)
        print("저장:", out)
    else:
        plt.show()


if __name__ == "__main__":
    main()
