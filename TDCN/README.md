# TDCN 모드 — CLAW 제어기 통합

세종대 CLAW 제어기를 ArduCopter 비행모드(`TDCN`, mode 29)로 통합한 것이다.
GCS 가 `MAV_CMD_USER_1` 로 시나리오 상태 1~11 을 보내면 기체가 그에 맞춰
격납함 개폐 대기 → 이륙 → 선박 추종 → 착륙까지 진행한다.

```
ArduCopter/mode_tdcn.cpp                    모드 본체
ArduCopter/mode.h                           ModeTDCN / CLAW_Gains 클래스 선언
ArduCopter/mode_tdcn_CLAW_IBSC_ship_Fianl_NED.c   CLAW 제어기 (외부 제공)
ArduCopter/mode_tdcn_CLAW_data_0729.c       CLAW 게인 초기값 (외부 제공)
TDCN/tdcn_log_compare.py                    로그 분석 스크립트
```

---

## v1 → v2 변경 요약

| 항목 | v1 | v2 |
|---|---|---|
| CLAW 출력 | 로그로만 기록, 기체에 미적용 | `TDCN_CLAW_ON_OFF` 로 믹서 직결 가능 |
| 이륙/착륙 고도·속도 | 소스에 상수 | 파라미터 4개 |
| CLAW 게인 | `data_0729.c` 에 상수 | 파라미터 23개, 실시간 반영 |
| 로그 | 출력 몇 개 | 계층별 8개 메시지 |
| state 9 착륙 | 정밀착륙으로 분기 가능 | 일반 착륙 고정 |

---

## 1. 파라미터 (총 28개)

미션플래너 **Config → Full Parameter List** 에서 `TDCN` / `CLAW` 로 검색된다.
값을 쓰면 즉시 저장되고 재부팅 후에도 유지된다.

### 시나리오 (`TDCN_`)

| 파라미터 | 의미 | 기본값 | 단위 | 반영 시점 |
|---|---|---|---|---|
| `TDCN_TKO_ALT` | state 4 이륙 목표 고도 | 1000 | cm | 다음 이륙 |
| `TDCN_TKO_SPD` | state 4 상승 속도 | 100 | cm/s | 다음 이륙 |
| `TDCN_LND_ALT` | state 8 착륙 동기 고도 | 1000 | cm | 다음 착륙 |
| `TDCN_LND_SPD` | state 8 하강 속도 | 100 | cm/s | 다음 착륙 |
| `TDCN_CLAW_ON_OFF` | 0 = 아두파일럿 조종 / 1 = CLAW 조종 | 0 | — | 즉시 |

### CLAW 게인 (`CLAW_`) — 값은 `data_0729.c` 원본과 동일

| 파라미터 | 기본값 | 파라미터 | 기본값 |
|---|---|---|---|
| `CLAW_SCALE_TH` | 3.2 | `CLAW_OMEGA_XX` | 0.3 |
| `CLAW_SCALE_R` | 0.25 | `CLAW_OMEGA_YY` | 0.3 |
| `CLAW_SCALE_P` | 0.20 | `CLAW_OMEGA_ZZ` | 2.0 |
| `CLAW_SCALE_Y` | 0.13 | `CLAW_OMEGA_PH` | 9.3 |
| `CLAW_K_POS_P` | 0.3 | `CLAW_OMEGA_TH` | 12.0 |
| `CLAW_K_POS_I` | 0.01 | `CLAW_OMEGA_PS` | 3.0 |
| `CLAW_K_VEL_P` | 0.4 | `CLAW_ZETA_XX` | 1.015 |
| `CLAW_K_VEL_I` | 0.05 | `CLAW_ZETA_YY` | 1.015 |
| `CLAW_AWU_LIMIT` | 1.0 | `CLAW_ZETA_ZZ` | 0.75 |
| `CLAW_TAU_HDOT` | 0.164297 | `CLAW_ZETA_PH` | 0.98 |
| `CLAW_TAU_R` | 0.150985 | `CLAW_ZETA_TH` | 0.98 |
| | | `CLAW_ZETA_PS` | 0.9 |

`BSC_B_mat`(제어효과 행렬 48개) 은 파라미터화하지 않았다. 기체 제원에서 나오는
값이라 현장 조정 대상이 아니다.

`CLAW_Gains::apply()` 가 `CLAW_step()` 직전에 `CLAW_P` 로 복사하므로 **값을 바꾸면
다음 제어 스텝부터 반영된다.** 재부팅도 모드 재진입도 필요 없다.

### 이미 있는 ArduPilot 파라미터를 쓰는 것

| 대상 | 파라미터 | 기본값 |
|---|---|---|
| state 5~8 수평 이동 | `WPNAV_SPEED` / `WPNAV_ACCEL` | 1000 cm/s / 250 cm/s² |
| state 5~7 수직 이동 | `WPNAV_SPEED_UP` / `_DN` / `WPNAV_ACCEL_Z` | 250 / 150 / 100 |
| state 6 추종 지연 | `PSC_JERK_XY` | 5 m/s³ |
| state 6 요 변화율 | `ATC_SLEW_YAW` | 6000 cd/s (60°/s) |
| state 9 착륙 속도 | `LAND_SPEED` / `LAND_SPEED_HIGH` / `LAND_ALT_LOW` | 50 / 0 / 1000 |

---

## 2. CLAW 출력 인계 (`TDCN_CLAW_ON_OFF = 1`)

### 훅 지점

`Copter` 의 fast loop 는 `run_rate_controller`(7번) 로 제어값을 만들어 `motors` 에
넣어두고, `motors_output`(15번) 이 그 값으로 모터를 돌린다. `motors_output` 이
`flightmode->output_to_motors()` 를 부르므로 **계산은 끝났고 아직 모터로 안 나간**
그 자리에서 값을 갈아끼운다.

`run_rate_controller` 자체를 건너뛰지 않는다. 각속도 PID 의 적분항과 필터가 계속
갱신되어야 CLAW 를 껐을 때 튀지 않기 때문이다.

### 적용 조건 (모두 참일 때만)

```
TDCN_CLAW_ON_OFF == 1     조작자가 명시적으로 켰다
state == 6 (TRACKING)     CLAW 가 도는 유일한 state
home_init                 CLAW 가 home 을 잡은 뒤
armed && !landed          비행 중
```

state 7 로 넘어가면 두 번째 조건이 깨져 **자동으로 아두파일럿 제어로 복귀**한다.
로그의 `TDCC.ACT` 가 1 인 구간이 실제로 CLAW 가 몰던 구간이다.

### 스로틀 환산

범위와 기준점이 둘 다 다르다.

| | 범위 | 호버 |
|---|---|---|
| CLAW `cmd_height` | −1 ~ +1 | **0** |
| 믹서 throttle | 0 ~ 1 | `MOT_THST_HOVER` (기체마다 다름) |

CLAW 의 0 이 호버라는 것은 로그로 확인했다. 아두파일럿이 고도를 잡고 있던 7101
샘플에서 `cmd_height` 평균이 +0.0009(표준편차 0.018), 같은 구간 실제 스로틀이
0.332 였다. 즉 `cmd_height` 는 절대 스로틀이 아니라 **호버 기준 증분**이다.

그래서 0 을 그 기체의 실제 호버값에 맞춘다. 중앙에서 위아래 여유가 다르므로
꺾인 직선이 된다.

```cpp
const float hover = motors->get_throttle_hover();
const float thr = is_negative(ch) ? hover * (1.0f + ch)          // -1 -> 0
                                  : hover + ch * (1.0f - hover); // +1 -> 1
```

`(x+1)/2` 로 두면 CLAW 의 0 이 0.5 가 되어, 호버 0.334 인 기체에서 인계 순간
**+0.17 짜리 스로틀 계단**이 주입된다. 실제로 그 상태로 켰을 때 20 m 에서
고도를 잃었다.

### 인수인계 안전장치

CLAW 가 모는 동안 아두파일럿 제어기는 자기 명령이 반영되지 않으므로 목표와 실제가
계속 벌어진다. 그대로 두면 state 7 인계 순간 그 간극만큼 튄다. 그래서 state 6 에서
CLAW 가 몰 때 매 루프:

```cpp
attitude_control->reset_target_and_rate(false);   // 목표 자세 = 현재 자세
attitude_control->reset_rate_controller_I_terms();
```

`reset_rate = false` 로 두어 각속도 제어기는 계속 돌게 한다.

### 5 → 6 진입 시 요 초기화

CLAW 는 home 을 잡는 스텝에서 궤적 생성기를 전부 0 으로 리셋한다. `Xtraj[3]` 은
요 목표이고 **0 = 진북**이라, 기수가 북쪽이 아닌 채로 진입하면 그 헤딩이 통째로
자세 오차가 된다. 실기체 로그의 진입 헤딩은 121° 였다.

vendor 파일을 고치지 않고 `Run_CLAW()` 에서 래치 직후 되돌린다.

```cpp
if (!home_was_init && home_init) {
    Xtraj[3]  = STV[8];     // 요 목표 = 현재 헤딩
    dXtraj[3] = 0.0;
}
```

---

## 3. CLAW vendor 파일 수정 목록

CLAW 코드는 외부(Simulink 생성) 제공물이다. 새 버전을 받으면 아래 수정을 다시
적용해야 한다. 모든 수정 위치에 `/* Sejong: */` 주석으로 **원본 코드와 근거**를
남겨 두었다.

### 버그 수정

| 위치 | 내용 |
|---|---|
| `CLAW.c:233`, `363` | `Lon2m(Home_Lon)` → `Lon2m(Home_Lat)`. `Lon2m` 은 "그 위도에서 경도 1도가 몇 m 인가" 이므로 인자가 위도여야 한다. 경도를 넣으면 한국 경도(127°)에서 배율이 −0.76 이 되어 부호까지 뒤집힌다. SITL 에서 동쪽 +2.80 m 를 −2.92 m 로 계산하는 것을 확인했다. |
| `CLAW.c:184` | home 래치 스텝의 `Dest_poti_i` 대입에서 x(위도)/y(경도) 가 뒤바뀌어 있었다. 그 한 스텝 동안 타겟이 엉뚱한 곳이 되어 큰 스파이크가 난다. |
| `CLAW.c:410` | `uv_des = CTML_inv * pos_dot_des` → `CTML`. `pos_dot_des` 는 NED, `uv_des` 는 body 인데 `CTML_inv` 는 body→NED 라 −ψ 회전이 된다. 헤딩 0 에서는 두 행렬이 같아 SITL 에서 안 드러났고, 실기체 로그(헤딩 −11°~+122°)에서 잔차 비교로 확인했다 (`CTML` 0.66 vs `CTML_inv` 0.0007). |
| `CLAW.c:530`, `562` | `cmd_roll` 부호 반전(`1.0` → `-1.0`). 모멘트 명령은 자세 오차를 줄이는 방향이어야 하므로 `corr(cmd, z1) < 0` 이어야 하는데 롤만 +0.910 이었다(피치 −0.946, 요 −0.665). 수정 후 −0.888 로 뒤집혔다. 폐루프에서 롤 축 발산 → 전복의 직접 원인으로 보인다. |
| `CLAW.c:61`, `210`, `309` | `STV_OLD` 유효성 플래그 추가. 첫 스텝에 이전 속도가 없어 가속도가 `속도/DT` 만큼 튀었다. |

### 통합을 위한 변경 (계산 결과는 동일)

| 위치 | 내용 |
|---|---|
| `types.h:50` | `Dest_poti` 를 `real32_T` → `real_T`(float → double). float32 는 위도 0.42 m / 경도 0.675 m 로 양자화되어 cm 급 착함에 그대로 오차가 된다. |
| `CLAW.c:146` | `home_init` 의 `static` 제거. 모드 재진입 시 TDCN 이 `false` 로 되돌려 home 을 다시 잡게 한다. |
| `CLAW.c:149`, `227` | `Cur_Lat/Lon/Alt`, `Dest_Lat/Lon/Alt` 를 지역변수 → 파일 전역. TDCN 이 넘긴 값과 CLAW 내부값을 짝지어 비교하기 위함이다. |
| `CLAW.c:250` | `XTV[0..2] = 0` 세 줄을 주석 처리. TDCN 이 EKF 속도를 채우는데 원본이 그것을 지워 속도 피드백이 무효화되어 있었다. |
| `CLAW.c:571` | `cur_poti` 대입의 `real32_T` 캐스팅 제거 (double 승격에 맞춤). |

---

## 4. 로그

state 6 에서만 기록된다. 50 Hz(400 Hz 의 8분주).

| 메시지 | 내용 |
|---|---|
| `TDCP` | 현재 위치 — TDCN 이 넘긴 값 vs CLAW 내부값 |
| `TDCT` | 타겟 — 위경도/고도/헤딩, 양쪽 비교 |
| `TDCI` | IMU — `p,q,r`, `Roll,Pitch,DR_heading` vs `STV[3..8]` |
| `TDCV` | 속도 — `XTV[0..2]` 양쪽 비교 |
| `TDCL` | NED 위치 — 아두파일럿 EKF vs CLAW 위경도 환산 |
| `TDCA` | 목표 자세 — CLAW `Xtraj[1..3]` vs 아두파일럿 |
| `TDCR` | 각속도 명령 — CLAW `alpha[1..3]` vs 아두파일럿 |
| `TDCC` | 제어 출력 — CLAW `v_cmd` vs 믹서 입력, `ACT` 플래그 포함 |
| `TDCE` | CLAW 내부 — 오차, 적분기, 속도명령, 궤적 |

### 분석 스크립트

```bash
cd TDCN
./tdcn_log_compare.py                  # 최근 로그 자동 탐색 (LASTLOG.TXT 우선)
./tdcn_log_compare.py logs/00000013.BIN
./tdcn_log_compare.py --save out/      # PNG 저장
```

- **Figure 1** TDCN → CLAW 인터페이스 검증 (전달 무손실 확인)
- **Figure 2** NED 위치 — 아두파일럿 EKF vs CLAW 위경도 환산
- **Figure 3** 제어값 4개 — 아두파일럿 믹서 입력 vs CLAW 출력

---

## 5. 검증 결과

### 확인된 것

| 항목 | 결과 |
|---|---|
| 입력 전달 | TDCN 이 넘긴 값 = CLAW 내부값. 4577 샘플 중 어긋난 것은 home 래치 스텝 1샘플뿐(설계된 동작) |
| 좌표 변환 | 아두파일럿 EKF NED vs CLAW 위경도 환산. 오프셋(원점 차이) 과 기울기(지구 모델 차이, 위도 −0.3% / 경도 +0.1%) 를 빼면 잔차 1~2 cm |
| body 속도 회전 | `CTML` 수정 후 아두파일럿과 최대차이 0.0000 m/s |
| 제어 부호 | 롤 부호 수정 후 세 축 모두 자세 오차를 줄이는 방향 |
| CLAW 폐루프 비행 | 로그 13 에서 49초간 CLAW 가 기체를 몰았고 고도 유지 (−0.0 m → −0.2 m) |
| 인계 안전성 | `TDCN_CLAW_ON_OFF` 를 1 → 0 으로 되돌리면 아두파일럿이 깨끗하게 회복. 발진 중에도 복구 확인 |

### 미해결

| 항목 | 상태 |
|---|---|
| 요 진동 | `CLAW_SCALE_Y` 0.13 → 0.03 으로 포화는 사라졌으나 10 Hz 진동이 남아 있다. 실제 자세가 롤 31° / 피치 32° 까지 간다 |
| 기체 제원 불일치 | CLAW 게인은 실기체 기준이고 SITL 기체는 더 가볍다. `BSC_B_mat` 이 기체 제원에서 나오는 값이라 SITL 로는 게인 적정성·제어 성능 크기를 판정할 수 없다. 실기체 제원으로 SITL 프레임 JSON 을 만들면(`sim_vehicle.py --model json:파일`) 범위가 넓어진다 |
| 목표 스텝 크기 | CLAW 는 위치 오차를 shaping 없이 그대로 받는다. `K_POS_P = 0.3`, `max_vel = 5.0` 이므로 **오차 16.7 m 이상이면 속도명령이 포화**한다. state 6 진입 시 선박이 그보다 멀면 진입 순간 포화가 일어난다 |

---

## 6. 운용 시 주의

- **state 6 → LOITER 전환 시 조종기 스로틀 스틱을 중앙에 두어야 한다.** LOITER 는
  스로틀 스틱으로 상승률을 명령한다. 스틱이 0 이면 최대 하강률이 명령된다
  (실기체에서 이것으로 추락한 적이 있다). 데드존은 `THR_DZ` 기본 100(±10%).
- **state 9 착륙 중 롤/피치 스틱 입력은 착륙 지점을 옮긴다** (`LAND_REPOSITION`
  기본 1). 완전 자동으로 두려면 0 으로 설정한다.
- **`TDCN_CLAW_ON_OFF = 1` 은 게인이 그 기체에 맞는지 확인한 뒤에만 켠다.**
  이상하면 0 으로 되돌리면 즉시 아두파일럿으로 복귀한다.

---

## 빌드

```bash
./waf configure --board Pixhawk4   # 또는 fmuv5, sitl
./waf copter
# 펌웨어: build/<보드>/bin/arducopter.apj
```
