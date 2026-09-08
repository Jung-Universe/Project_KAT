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

## 버전 변경 요약

| 항목 | v1 | v2 | **v3-yb (현재)** |
|---|---|---|---|
| 구조 | 병렬 (CLAW 는 로그만) | 병렬 + 믹서 직결 선택 | **직렬** |
| CLAW 출력이 가는 곳 | 없음 | 믹서 (`TDCN_CLAW_ON_OFF`) | **LOITER 의 스틱 입력 자리** |
| 아두파일럿 PID | CLAW 와 무관하게 돌음 | CLAW 켜면 무시됨 | **전부 살아서 쓰임** |
| 아두파일럿이 타겟을 아는가 | 안다 (같은 타겟 추종) | 안다 | **모른다 (CLAW 만 안다)** |
| 이륙/착륙 고도·속도 | 소스 상수 | 파라미터 4개 | 파라미터 4개 |
| CLAW 게인 | `data_0729.c` 상수 | 파라미터 23개 | 파라미터 23개 |
| 스케일 상수 | — | — | **파라미터 4개 (`TDCN_SC_*`)** |
| state 9 착륙 | 정밀착륙 분기 가능 | 일반 착륙 고정 | 일반 착륙 고정 |

v2 까지는 두 제어기가 **나란히** 돌았다. v3-yb 는 한 줄로 잇는다.

```
GCS 타겟 -> CLAW -> 스케일 -> LOITER 입력 -> 아두파일럿 제어기 -> 모터
```

---

## 1. 파라미터 (총 31개)

미션플래너 **Config → Full Parameter List** 에서 `TDCN` / `CLAW` 로 검색된다.
값을 쓰면 즉시 저장되고 재부팅 후에도 유지된다.

### 시나리오 (`TDCN_`)

| 파라미터 | 의미 | 기본값 | 단위 | 반영 시점 |
|---|---|---|---|---|
| `TDCN_TKO_ALT` | state 4 이륙 목표 고도 | 1000 | cm | 다음 이륙 |
| `TDCN_TKO_SPD` | state 4 상승 속도 | 100 | cm/s | 다음 이륙 |
| `TDCN_LND_ALT` | state 8 착륙 동기 고도 | 1000 | cm | 다음 착륙 |
| `TDCN_LND_SPD` | state 8 하강 속도 | 100 | cm/s | 다음 착륙 |
| `TDCN_SC_ROLL` | CLAW 롤 출력 → 가상 스틱 PWM 오프셋 | 250 | PWM | 즉시 |
| `TDCN_SC_PITCH` | CLAW 피치 출력 → 〃 | 250 | PWM | 즉시 |
| `TDCN_SC_YAW` | CLAW 요 출력 → 〃 | 80 | PWM | 즉시 |
| `TDCN_SC_THR` | CLAW 스로틀 출력 → 〃 | 250 | PWM | 즉시 |

`TDCN_CLAW_ON_OFF` 는 v3-yb 에서 없어졌다. 병렬 구조가 아니므로 켜고 끌 대상이
없다. state 6 에 있으면 CLAW 가 항상 조종한다.

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

## 2. 직렬 구조 — CLAW 출력을 LOITER 스틱 입력으로

### 왜 이렇게 하나

원래 하드웨어는 이렇게 묶여 있다.

```
조종기 ))) 수신기 ──SBUS──> 외부컴퓨터(CLAW) ──SBUS──> FC(RCIN) ──> LOITER ──> PID ──> 모터
```

외부컴퓨터가 **수신기 자리를 그대로 대신한다.** FC 입장에서는 수신기가 붙어 있는
것과 구분되지 않는다. CLAW 게인은 이 사슬 전체를 통과한 상태로 튜닝되었다.

v3-yb 는 이 사슬을 FC 안에서 재현한다. CLAW 출력을 모터에 직접 넣지 않고,
**조종기 스틱이 들어가던 자리**에 넣는다.

```
CLAW 출력 (-1 ~ +1)
   |
   v  가상 PWM = RCn_TRIM + 출력 x TDCN_SC_*        (외부컴퓨터가 하던 계산)
   v  pwm_to_unit()   트림 / 데드존 / 상하 비대칭    (SBUS 수신부)
   v  rc_input_to_roll_pitch / PILOT_Y_RATE / get_pilot_desired_climb_rate
   v  LOITER  (ModeLoiter::run 의 Flying 경로와 동일)
   v  아두파일럿 P/PID 전체
   v  모터
```

물리량을 직접 곱하지 않기 때문에 `RCn_MIN/TRIM/MAX`, `RCn_DZ`, `THR_DZ`,
`ANGLE_MAX`, `PILOT_Y_RATE`, `PILOT_SPEED_UP/DN` 이 실기체와 똑같이 작용한다.
**실기체 파라미터를 SITL 에 넣으면 코드 수정 없이 그대로 재현된다.**

### 기본 스케일에서 나오는 값

기본 RC 캘리브레이션(1000/1500/2000, DZ 20/30) 기준.

| 축 | 스케일 | CLAW ±1 일 때 |
|---|---|---|
| 롤 / 피치 | 250 | ±14.4° 자세각 |
| 요 | 80 | ±25.3 °/s |
| 스로틀 | 250 | +99.6 / −87.5 cm/s |

### 데드존을 CLAW 값에도 적용한다

CLAW 값도 조종기와 똑같이 `RC_Channel` 의 데드존을 통과한다. 외부컴퓨터가 보낸
PWM 을 FC 가 받을 때 실기체에서도 그랬기 때문이다.

그 결과 축마다 **위치 불감대**가 생긴다.

```
불감대  =  데드존  /  스케일  /  CLAW 이득
```

| 축 | 데드존(CLAW 단위) | 계산 불감대 | SITL 실측 정착오차 |
|---|---|---|---|
| 롤 / 피치 | 0.08 | ≈1.0 m | 0.77 ~ 1.06 m |
| 스로틀 | 0.39 | ≈1.2 m | 1.01 m |

**이 오차를 CLAW 성능으로 판정하면 안 된다.** 정착 구간에서 CLAW 는 피치 −0.079,
고도 0.346 을 계속 내고 있었고 문턱(0.08 / 0.39) 바로 아래에서 막혔다.

조종자 스틱에만 데드존을 두고 CLAW 값은 통과시키는 우회를 만들면 이 오차는
사라진다(최대 권한을 유지한 채 가능). **일부러 넣지 않았다.** 실기체에서 데드존
파라미터를 수정했을 수 있고, 그러면 지금 구조가 그 차이를 그대로 반영해 주기
때문이다. 실기체 `RC1~4_DZ` / `THR_DZ` 를 확인한 뒤에 정할 일이다.

### 조종기 우선

원래는 외부컴퓨터가 조종기와 CLAW 를 섞었다. FC 안에는 그 지점이 없으므로
`claw_to_loiter_input()` 에서 정한다. **스틱이 데드존을 벗어난 축은 조종자가
이긴다.**

- 롤·피치는 한 쌍 — `rc_input_to_roll_pitch()` 가 원형 제한으로 묶는다
- 요·스로틀은 독립
- 로그 `TDCK.STK` 비트로 어느 축을 조종자가 잡았는지 보인다

**전제: state 6 에 들어갈 때 스로틀 스틱은 중앙에 둔다.** 스로틀은 스프링으로
중립에 돌아오지 않으므로 이것은 코드가 아니라 운용으로 지킨다. 내려둔 채
들어가면 그 자리가 하강 명령으로 읽혀 CLAW 스로틀이 무시된다.
SITL 에서는 state 6 전에 `rc 3 1500` 을 준다.

### v2 에서 없어진 것

| 없어진 것 | 이유 |
|---|---|
| `output_to_motors()` 오버라이드 | 믹서를 덮어쓰지 않는다 |
| `claw_output_active()` | 켜고 끌 대상이 없다 |
| `TDCN_CLAW_ON_OFF` | 〃 |
| 스로틀 `MOT_THST_HOVER` 환산 | 믹서에 직접 넣지 않으므로 불필요 |
| `reset_target_and_rate()` 인계 보정 | 아두파일럿이 계속 몰고 있어 간극이 안 생긴다 |

### LOITER 원본과 다른 점

`ModeLoiter::run()` 의 **Flying 경로**를 그대로 쓰되 아래는 빠져 있다.

| 빠진 것 | 영향 |
|---|---|
| `get_alt_hold_state()` 상태기계 | state 6 은 이미 비행 중이라 Flying 만 필요 |
| `set_max_speed_accel_z` 매 루프 | 진입 시 1회. 비행 중 `PILOT_SPEED_UP` 변경 미반영 |
| `get_avoidance_adjusted_climbrate()` | 펜스·회피가 상승률을 못 깎음 |
| `soften_for_landing()` | 접지 직전 로이터 완화 없음 |
| `surface_tracking` | 레인지파인더 지형 추종 없음 |
| `update_simple_mode()` | CLAW 가 NED 기준이라 적용하면 안 된다 |
| 무선 페일세이프 시 `clear_pilot_desired_acceleration()` | 조종자 입력만 무시하고 CLAW 는 계속 쓴다 |

CLAW 가 펌웨어 안에 있으므로 무선 두절은 CLAW 동작과 무관하다. 실제로 의미 있는
두절은 **GCS 타겟 명령이 끊기는 경우**이고, 현재 그 처리는 없다.

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
| `TDCC` | 제어 출력 — CLAW `v_cmd` vs 믹서 입력, `STK` 플래그 포함 |
| `TDCE` | CLAW 내부 — 오차, 적분기, 속도명령, 궤적 |
| `TDCK` | **CLAW → LOITER 입력** — 가상 스틱 PWM, LOITER 에 넣은 물리 명령, `STK` |

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

`TDCK` 는 v3-yb 에서 새로 추가되었다. 직렬 구조에서 가장 중요한 관찰점이다.

| 필드 | 내용 |
|---|---|
| `SR/SP/SY/ST` | CLAW 출력으로 만든 가상 스틱 PWM |
| `LR/LP` | LOITER 에 넣은 자세각 (cd) |
| `LY` | 요 각속도 (cds) |
| `LT` | 상승률 (cm/s) |
| `STK` | 조종자가 잡은 축. bit0 롤·피치, bit1 요, bit2 스로틀 |

**`STK` 가 0 이 아닌 구간은 CLAW 성능 판정에서 빼야 한다.**

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
| **직렬 구조 비행 (v3-yb)** | 로그 2 에서 **161초** 연속. `STK` 전 구간 0 (네 축 모두 CLAW). 목표 5회 변경 모두 추종 |
| **발진 해소 (v3-yb)** | 롤 RMS 0.39° / 피치 0.60°, 실제 자세 롤 −1.1~3.5° / 피치 −5.1~4.0°. 아두파일럿 각속도 PID 가 살아난 효과 |
| **출력 여유 (v3-yb)** | 롤 최대 0.47, 피치 0.67 — 포화 0%. 요 2.3%, 스로틀 10.5% |

### 미해결

| 항목 | 상태 |
|---|---|
| 요 진동 | `CLAW_SCALE_Y` 0.13 → 0.03 으로 포화는 사라졌으나 10 Hz 진동이 남아 있다. 실제 자세가 롤 31° / 피치 32° 까지 간다 |
| 기체 제원 불일치 | CLAW 게인은 실기체 기준이고 SITL 기체는 더 가볍다. `BSC_B_mat` 이 기체 제원에서 나오는 값이라 SITL 로는 게인 적정성·제어 성능 크기를 판정할 수 없다. 실기체 제원으로 SITL 프레임 JSON 을 만들면(`sim_vehicle.py --model json:파일`) 범위가 넓어진다 |
| **데드존 불감대** | 축마다 1 m 근처에서 수렴이 멈춘다. 원인은 CLAW 가 아니라 RC 데드존(2절 참고). **실기체 `RC1~4_DZ` / `THR_DZ` 를 받기 전까지 게인 판정 보류** |
| **외부컴퓨터 채널 생성 방식** | 덮어쓰기(`1500 + CLAW x SC`) 로 가정했다. 조종기 값에 **더하는** 방식이었다면 조종자 우선 로직 자체를 바꿔야 한다. 확인 필요 |
| **GCS 명령 두절 처리** | 없다. CLAW 의 유일한 외부 입력이 GCS 타겟인데 끊겼을 때 동작이 정의되어 있지 않다 |
| 하강 경로 미검증 | 로그 2 에서 `cmd_height` 최소가 −0.045 로 데드밴드 안이라 하강 명령이 한 번도 나가지 않았다 |
| 목표 스텝 크기 | CLAW 는 위치 오차를 shaping 없이 그대로 받는다. `K_POS_P = 0.3`, `max_vel = 5.0` 이므로 **오차 16.7 m 이상이면 속도명령이 포화**한다. state 6 진입 시 선박이 그보다 멀면 진입 순간 포화가 일어난다 |

---

## 6. 운용 시 주의

- **state 6 → LOITER 전환 시 조종기 스로틀 스틱을 중앙에 두어야 한다.** LOITER 는
  스로틀 스틱으로 상승률을 명령한다. 스틱이 0 이면 최대 하강률이 명령된다
  (실기체에서 이것으로 추락한 적이 있다). 데드존은 `THR_DZ` 기본 100(±10%).
- **state 9 착륙 중 롤/피치 스틱 입력은 착륙 지점을 옮긴다** (`LAND_REPOSITION`
  기본 1). 완전 자동으로 두려면 0 으로 설정한다.
- **state 6 진입 시 스로틀 스틱을 중앙에 둔다.** 데드존 밖이면 조종자 우선으로
  판정되어 CLAW 스로틀이 무시된다. SITL 은 RC3 이 1000 에서 시작하므로
  `rc 3 1500` 을 먼저 준다 (안 하면 최대 하강이 걸려 수 초 만에 착지한다).
- **`TDCN_SC_*` 는 게인이다.** 값을 키우면 CLAW 의 유효 이득이 올라간다.
  외부컴퓨터가 쓰던 값(250/250/80/250) 을 그대로 두는 것이 원칙이다.

---

## 빌드

```bash
./waf configure --board Pixhawk4   # 또는 fmuv5, sitl
./waf copter
# 펌웨어: build/<보드>/bin/arducopter.apj
```
