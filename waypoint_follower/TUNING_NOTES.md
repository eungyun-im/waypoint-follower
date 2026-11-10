# waypoint_follower 튜닝 노트

> 2026-09-29 분석 기록. 대상: `src/new_path_tracking.cpp`, `src/new_process.cpp`,
> `include/waypoint_follower/waypointFollower.h`, `config/tracking_params.yaml`
>
> **주의: 이 문서의 수식/코드 분석은 검증된 것이고, "권장값"은 전부 미검증 추정입니다.**
> 실차 bag 없이 작성되었습니다.

---

## 0. 한 줄 요약

- Stanley·PP 구현 **로직에 치명적 버그 없음** (부호 규약·인덱스 경계·NaN 전파 전부 확인)
- 고칠 가치 있는 코드 이슈 **2개** (§3.1, §3.2), 나머지는 죽은 코드
- **모든 파라미터가 미측정 시뮬레이션 값**. 실차 bag 1회로 7개가 자동 확정됨 (§4.1)
- 제일 위험한 값: **`speed.a_lat_max: 3.0`** (타이어 그립, 측정 이력 없음, 주석 경고도 없음)

---

## 1. 현재 구조

### 1.1 Ld 파이프라인 (매 사이클 3단계)

```cpp
// new_process.cpp:14-19  ← 새 /odom 이 온 사이클에만 실행
double cap = ldStateCapM(state);       // ① 상태별 캡 = 0.0 (꺼져 있음)
const double curv_cap = curvatureLdCap();  // ② 곡률 캡
if (curv_cap > 0.0) cap = (cap > 0.0) ? min(cap, curv_cap) : curv_cap;
ld_state_cap_ = rateLimitCap(cap);     // ③ 슬루 제한 후 멤버에 저장

// new_path_tracking.cpp:24  ← pure_pursuit() 안에서
lookahead_dist_ = adaptiveLookahead(); // ④ 속도 스케줄 ∧ ld_state_cap_
```

| 단계 | 함수 | 식 |
|---|---|---|
| ② | `curvatureLdCap()` :706 | `cap = clamp(2.3·sqrt(R), 3.0, 11.0)`, 최근 5 m 최솟값 유지 |
| ③ | `rateLimitCap()` :734 | `cap = clamp(target, prev − 4.0·dt, prev + 2.0·dt)` |
| ④ | `adaptiveLookahead()` :619 | `Ld = clamp(1.6·v + 4, 4, 11)` → `min(Ld, cap)` |

**주의:** ④에서 clamp 먼저, min 나중이라 **하한 4.0이 뚫립니다.**
실질 Ld 하한은 `curv_min_m = 3.0`.

차선 변경 시 Ld 를 `18/(1+e^(-d)) − 2.5` 로 덮어쓰던 블록은 제거됨 — 차선 변경 중에도 위 파이프라인 그대로.

### 1.2 속도 파이프라인

```
spd_state_ (uniform_kmh 18.0)
  ↓ ACC          acc_index >200 통과 / 100~200 시그모이드 / ≤100 정지
  ↓ applySpeedLimits()  ┬ min(곡률 프로파일)
  ↓                     ├ × cteSpeedFactor()     |cte| 0.25→1.2 m 에서 1.0→0.5
  ↓                     └ × oscillationFactor()  20 s 내 3플립 → 0.6까지
  ↓ INS 불량 → 7 km/h 캡 / odom·경로 이상 → 0
  ↓ rampSpeed()  가속만 1.5 m/s² (=5.4 km/h/s) 제한, 감속 무제한
input_speed_
```

곡률 프로파일 `curvatureSpeedLimitKmh()` :308 — 점별 `v=sqrt(3.0/k)` → 역방향 패스(`a_dec 1.0`) → `lag_comp 0.55 s` 앞 지점 선택 → `exit_hold 5 m` 최솟값 유지.

### 1.3 Stanley (`stanly()` :216)

```
δ = k_ang·ψ_e + atan(k_dist·e / (k_damp·v + k_soft)) + k_rate·(r_meas − r_path) + k_steer·(δ_cmd − δ_meas)
```

- `e` : `projectOnPath()` 세그먼트 수직 투영 (:182)
- `ψ_e` : `tangent_half_window_m: 1.0` 앞뒤 1 m 현 기준
- `r_meas` `r_path` : :253-255, 요레이트 damping (D항)
- `k_rate`/`k_steer` 항은 **`isFeedbackFresh()` 참일 때만** 동작 (:252, :261)

PP와의 블렌드는 `hybrid_control()` :777-782, 속도로 가중 (`fade_lo 1.5` → `fade_hi 5.0` m/s, 최대 `max_ratio 0.7`).

---

## 2. 수학적으로 확정된 관계식 (측정 불필요)

### 2.1 Pure Pursuit

직선 선형화 → `ë + (2v/Ld)·ė + (2v²/Ld²)·e = 0`

| 항목 | 식 | 현재값 |
|---|---|---|
| 감쇠비 | `ζ = 1/sqrt(2) = 0.707` | **Ld·v 와 무관하게 항상 일정** |
| 고유진동수 | `ωn = sqrt(2)·v / Ld` | 25 km/h, Ld 11 → 0.14 Hz |
| **정착 거리** | **`4·Ld`** | Ld 11 → **44 m** |
| 코너 대역폭 | `ωn = sqrt(6)/curv_coef` | **반경 무관**, 2.3 → 0.17 Hz |

정착 거리가 속도와 무관하게 `4·Ld`인 것이 핵심. Ld 11 m는 외란 하나 수습에 44 m를 씁니다.

### 2.2 곡률 캡이 속도 스케줄을 항상 이깁니다

코너 한계 주행 시 `v = sqrt(a_lat·R) = sqrt(3R)` 이므로

```
스케줄 = 1.6·sqrt(3R) + 4 = 2.77·sqrt(R) + 4
캡     = 2.3·sqrt(R)
차이   = 0.47·sqrt(R) + 4  >  0   (모든 R)
```

`1.6·sqrt(3) = 2.77 > 2.3` 이라 **`min()`에서 항상 캡이 이깁니다.**
→ `lookahead.gain: 1.6` 은 **저속(0~15.75 km/h) + 완만한 경로**에서만 작동합니다.

### 2.3 25 km/h 상한에서의 작동 구간

| 반경 | 속도 제한 | Ld 캡 |
|---|---|---|
| R > 22.9 m | 없음 (25 km/h) | 없음 (11 m) |
| 16.1 ~ 22.9 m | 없음 (25 km/h) | **9.2 ~ 11 m — Ld만 작동** |
| R < 16.1 m | 발동 | 발동 |

`a_lat_max 3.0` 기준 `sqrt(3R) = 6.94` → R = 16.1 m / `2.3·sqrt(R) = 11` → R = 22.9 m

### 2.4 Stanley 횡오차 수렴

```
ė ≈ −v·k_dist·e / (k_damp·v + k_soft)

τ  = (k_damp·v + k_soft) / (k_dist·v)
τ∞ = k_damp / k_dist              ← 고속 극한. k_dist 와 k_damp 는 중복 손잡이
정착 거리 ≈ 4·τ·v
```

현재값 (`k_dist 0.8`, `k_damp 1.0`, `k_soft 1.0`):

| 속도 | τ | 정착 거리 |
|---|---|---|
| 7 km/h | 1.88 s | 15 m |
| 14 km/h | 1.56 s | 25 m |
| **25 km/h** | **1.43 s** | **40 m** |

PP(44 m)와 큰 차이 없음 → `k_dist` 상향 여지 있음 (단 §4.3 지연 제약).

### 2.5 PP 코너 컷

`컷 ≈ Ld²/(8R)`, `Ld = c·sqrt(R)` → `컷 = c²/8` (반경 무관 상수)

| `curv_coef` | 컷 | R 4.6 m Ld | R 8 m Ld |
|---|---|---|---|
| 2.3 (현재) | 0.66 m | 4.93 m | 6.51 m |
| 1.8 | 0.41 m | 3.86 m | 5.09 m |
| 1.5 | 0.28 m | 3.22 m | 4.24 m |

> **정정 기록:** 이 컷은 정상상태 바이어스가 **아니라 코너 진입 과도응답**입니다.
> 접현각 정리로 PP는 원 위에서 정확히 `κ = 1/R` 을 냅니다 (`sin α = Ld/2R` → `2 sin α/Ld = 1/R`).
> 따라서 **`atan(L·κ)` 피드포워드를 추가하면 조향이 2배가 되어 오히려 악화됩니다. 넣지 말 것.**

### 2.6 코너 진입 타이밍 마진 (25 km/h → R 4.6 m)

```
감속 시작      d = (6.94² − 3.71²)/(2·1.0) = 17.2 m 전
preview 진입   11 m 전, 그 시점 속도 v = 5.98 m/s
남은 시간      t = (5.98 − 3.71)/1.0 = 2.27 s
Ld 하강 소요   (11.0 − 4.93)/4.0 = 1.52 s
여유           0.75 s  →  §3.1 이슈로 0.58 s
```

**마진 충분.** `rate_down: 4.0` 이나 preview 분리는 25 km/h 상한에서 불필요.
단 25 km/h 초과 시 이 마진이 먼저 깨집니다.

---

## 3. 코드 이슈

### 3.1 [✅ 수정 완료] 곡률 캡이 지연 보상 전 pose를 사용

```cpp
// new_process.cpp
15:   const double curv_cap = curvatureLdCap();   // find_cloestindex(cur_pose_back) — 보상 前
38:   if (delay_comp_sec_ > 0.0) { ... }          // 여기서 pose 투영
55:   input_steer_ = hybrid_control();            // 타겟 선택 — 보상 後
```

같은 조향 기하인데 기준 위치가 **1.04 m** 어긋남 (25 km/h × 0.15 s).
→ 곡률 캡이 코너를 1 m 늦게 인식, 진입 여유 0.75 → 0.58 s.

**수정 완료:** 캡 계산 블록을 지연 보상 뒤 (`new_process.cpp:49-64`, `hybrid_control()` 직전)로 이동.

부수 효과: 블록이 `is_pose_ && is_lane_` (odom 주기) 에서 `is_control_` (20 Hz) 으로 옮겨져
캡이 이제 조향 명령과 같은 주기로 갱신됩니다. `rateLimitCap` 은 `prev ± rate·dt` 라 총 변화율은
보존되고, `holdMin` 은 `odo_m_` 이 안 움직이는 사이클에서 5 cm 병합으로 흡수됩니다.

### 3.2 [✅ 수정 완료] `holdMin` 이력이 차선 변경 시 초기화되지 않음

```cpp
// new_path_tracking.cpp:658-662 (updateOdometer)
if (step > 2.0) { hold_hist_.clear(); ld_hold_hist_.clear(); }
```

odometer 점프(>2 m)에서만 clear. **차선 변경은 경로가 바뀌어도 odometer 연속** → 이전 경로의 곡률 한계가 5 m 유지됨.
`holdMin`이 최솟값이라 방향은 보수적(안전)이나 논리 오류.

**수정 완료:** `StateCallback()` 의 차선 변경 감지 지점 (`waypointFollower.h:838-849`) 에서
`hold_hist_` / `ld_hold_hist_` clear. `lane_change_stamp_` 를 찍는 바로 그 자리라 **1회성**으로
동작합니다 (`laneChangeActive()` 는 hold_sec 동안 계속 참이라 부적합).

### 3.3 [정리] `projectOnPath()` 매 사이클 2회 중복 호출

`hybrid_control():773` 과 `stanly():225` 가 같은 pose로 같은 O(N) 계산 수행, `last_cte_` 도 두 번 대입.

### 3.4 [정리] `pure_pursuit()` 각도 wrap 블록이 no-op

```cpp
84-96:  if (temp_theta * cur_course_ < 0) { ... ±360 ... cout << "case 0"; }
103:    ... sin(alpha) ...      // sin 은 360° 주기 → 보정이 결과를 안 바꿈
```

블록 전체가 결과에 영향 없음. 매 사이클 `cout` 스팸만 발생.
`atan2(sin(Δ), cos(Δ))` 로 대체하거나 삭제.

### 3.5 [정리] 죽은 코드 5개

| 위치 | 내용 | 원인 |
|---|---|---|
| `waypointFollower.h:706` | `ldStateCapM()` 16칸 테이블 | `state_cap_m` 전부 `0.0` |
| `waypointFollower.h:662` | `missionSpeedKmh()` 16칸 테이블 | `uniform_kmh: 25.0` 이 먼저 리턴 |
| `waypointFollower.h:850` | `PathCallback()` | 구독 자체가 없음 |
| `new_path_tracking.cpp:597` | `cross_error()` | 호출처 0 |

`speed.anticipate_mission: true` 도 `uniform_kmh` 가 상태 테이블을 평탄화해 **무의미** (중복 클램프).

### 3.6 [✅ 추가 완료] CTE 디버그 토픽

발행자 전체 (`waypointFollower.h:410-417`): `front_pose`, `back_pose`, `target_pose`×2, `lane_number_msg_`, `course_path`, `/wf_debug/lookahead`

**CTE(`last_cte_`)는 계산·사용은 정상이나 발행되지 않음** → 실시간 튜닝 불가, 매번 bag 오프라인 분석 필요.

**추가 완료:** `/wf_debug/cte` (`std_msgs/Float64`, 부호 있음, **+ = 경로가 오른쪽**).

- 선언 `waypointFollower.h:190-194`
- 생성 `waypointFollower.h:423-425`
- 발행 `new_path_tracking.cpp:776-780` — `hybrid_control()` 에서 매 사이클 (20 Hz)

NaN 도 그대로 발행합니다. 플롯의 끊김이 곧 `projectOnPath()` 실패이고, 그게 `process():70` 의
정지 조건이기도 합니다.

```bash
rqt_plot /wf_debug/cte/data /wf_debug/lookahead/data   # 주행 중 실시간
ros2 topic echo /wf_debug/cte
```

`record_fit_bag.sh` 는 `/wf_debug/lookahead` 만 기록하므로, bag 에 CTE 도 남기려면
그 스크립트에 `/wf_debug/cte` 를 추가해야 합니다 (미적용).

### 3.7 검증했으나 문제 없던 항목

| 항목 | 결과 |
|---|---|
| Stanley 부호 규약 4항 (`cte`, `ψ_e`, `rate`, `lag`) | 전부 일관 |
| `segmentLength()` 호출처 5곳 인덱스 경계 | 전부 `j+1 < n` 보장 |
| `pathCurvature(seg+1)` 경계 | `arcWindow` 가 `c < n-1` 가드 |
| NaN 전파 → 정지 경로 | `process():70` 에서 차단 정상 |
| `waypoints_size_` vs `waypoints_.size()` | 항상 일치 |
| 콜백 경쟁 | `spin_some()` 단일 스레드, race 없음 |
| `rateLimitCap` 초기 상태 | 첫 사이클만 점프, 이후 정상 |
| `holdMin` 이력 증가 | 5 cm 병합으로 유한 |

---

## 4. 파라미터 분류와 피팅

### 4.1 bag 1회로 자동 확정 (7개) — `fit_from_bag.py:477-508`

| 파일 | 파라미터 | 산출 |
|---|---|---|
| `tracking_params` | `delay_comp_sec` | 측정 위치 지연 |
| `tracking_params` | `speed.a_dec_max` | 실측 감속 × 0.8 |
| `tracking_params` | `speed.lag_comp_sec` | `v_dead + v_tau + 0.1` |
| `waypoint_follower` | `WHEEL_BASE` | 요레이트 vs `v·tanδ` 역산 |
| `serial_bridge` | `steering_offset_deg` | 직진 시 조향 편차 |
| `odom_information` | `yaw_lpf_tau_sec` | IMU 지연 |
| `aeb_params` | `decel_capability_mps2` | 실측 제동 × 0.8 |

같은 실행에서 **`steer_tau`(조향 액추에이터 시상수)** 도 출력됨 → `k_steer`, `curv_coef` 하한 결정에 필수.

> `steer.rate_limit_deg_s: 90.0` 은 측정은 되나 **자동 제안 목록에 없음.** 출력의 `rate limit ~N deg/s` 를 수동 반영.

### 4.2 시험주행으로 결정

| 파라미터 | 시험 | 합격 기준 |
|---|---|---|
| `stanley.k_dist` | 직선에 0.5 m 평행 오프셋 진입 | 오버슈트 없이 수렴 |
| `stanley.k_soft` | 저속 3~5 km/h | 정지 근처 조향 안 튐 |
| `stanley.k_rate` | `k_dist` 상향 후 고주파 떨림 | 떨림 소멸 |
| `stanley.k_steer` | `steer_tau` 확정 후 | 명령−실측 차이 감소 |
| `lookahead.max_m` | 직선 오프셋 복귀 | 실측이 `4·Ld` 와 일치 |
| `lookahead.curv_coef` | 코너 안쪽 파고든 양 | 허용 오차 이내 |
| `osc.*` | 실제 weave 주파수 측정 | — |

**건드리지 말 것:** `stanley.k_ang: 1.0` (Hoffmann 원식의 이론값), `stanley.k_damp: 1.0` (`k_dist` 와 중복).

### 4.3 `curv_coef` 하한은 지연이 정함

PP 위상여유 약 65°(ζ=0.707), 총 지연 `τ_d` 가 `ω·τ_d` 만큼 잠식.
여유 45° 유지 조건: **`curv_coef ≥ 7·τ_d`**

| `τ_d` | 최소 `curv_coef` |
|---|---|
| 0.3 s | 2.1 |
| 0.2 s | 1.4 |
| 0.15 s | 1.05 |

**`steer_tau` 측정 전에는 `curv_coef` 를 내리면 안 됩니다.**
(이 표의 `τ_d` 는 전부 추정치. §5 참조)

### 4.4 측정 대상이 아닌 설계 선택

| 파라미터 | 결정 내용 |
|---|---|
| **`speed.a_lat_max: 3.0`** | 코너에서 감수할 횡가속도 — **가장 위험** |
| `mission.uniform_kmh: 25.0` | 목표 속도 |
| `cte.deadband_m / full_m / min_factor` | 허용 오차 |
| `stanley.max_ratio / fade_*` | PP vs Stanley 비중 |
| `lookahead.curv_min_m: 3.0` | Ld 안전 하한 |

### 4.5 피팅 순서 (의존관계 있음, 순서 바꾸면 안 됨)

```
1. 플랜트     WHEEL_BASE, steering_offset_deg
      ↓       틀리면 조향식·요레이트 전부 틀림
2. 지연       delay_comp_sec
      ↓       틀리면 CTE 계산 자체가 틀려 k_dist 피팅이 무의미
3. 종방향     a_dec_max, a_acc_max, lag_comp_sec
      ↓       틀리면 코너 진입 속도가 달라 시험 조건 불일치
4. 그립       a_lat_max  (1.5 → 2.0 → 2.5 → 3.0 단계 탐색)
      ↓
5. Stanley    k_dist → k_soft → k_rate → k_steer
      ↓
6. PP         curv_coef, max_m
      ↓
7. 블렌드     max_ratio, fade_lo/hi
```

**2단계가 핵심.** `delay_comp_sec` 이 틀린 채 `k_dist` 를 맞추면 지연 오차를 게인으로 보상한 값이 나와, 나중에 지연을 고치면 전부 재작업해야 합니다.

### 4.6 1차 bag 수집 절차

Stanley만 단독 피팅하려면 PP 가중치를 0으로:
```yaml
stanley: { max_ratio: 1.0, fade_lo_mps: 0.0, fade_hi_mps: 0.01 }   # 순수 Stanley
```
PP만 쓰려면 `max_ratio: 0.0` (구 버전과 동일 조건, 1차 bag 권장).

안전 설정으로 저속 주행:
```yaml
mission:  { uniform_kmh: 10.0 }
speed:    { a_lat_max: 1.5 }
stanley:  { max_ratio: 0.0, k_rate: 0.0, k_steer: 0.0 }
```

```bash
bash waypoint_follower/test/record_fit_bag.sh fit_01   # 오토 모드, 스택 정지 1회 포함
python3 waypoint_follower/test/fit_from_bag.py fit_01
```

---

## 5. 미검증 가정값 목록

### 5.1 주석이 미측정을 명시한 것

| 파라미터 | 주석 |
|---|---|
| `steer.rate_limit_deg_s: 90.0` | "an **untested**, conservative starting point -- MEASURE" |
| `delay_comp_sec: 0.15` | "**MEASURE** on the vehicle" |
| `speed.a_dec_max: 1.0` | "**MEASURE** the vehicle's real braking deceleration" |
| `mission.uniform_kmh` | "The plant is **not fitted yet**: raise it in steps, 15 → 20 → 25" |

> **[✅ 수정 완료] `uniform_kmh: 25.0` → `18.0`.**
> 주석이 15부터 단계적으로 올리라는데 값이 최대치인 25로 박혀 있었습니다.
> 실차 bag + `fit_from_bag.py` 이후에만 25 방향으로 올릴 것.

### 5.2 주석 없이 박혀 있는 것

| 파라미터 | 비고 |
|---|---|
| **`speed.a_lat_max: 3.0`** | **타이어 그립. 측정 이력 없음. 경고 주석도 없음. 틀리면 코너 이탈** |
| `speed.a_acc_max: 1.5` | 실측 없음 |
| `speed.lag_comp_sec: 0.55` | 시뮬 근거만 |
| `lookahead.curv_coef/min_m/hold_m` | 시뮬 근거만 |
| `lookahead.rate_up_mps / rate_down_mps` | 근거 기록 없음 |
| `stanley.*` 6개 | 시뮬 + 논문 기본값 |
| `osc.*` 6개 | 주석: "Re-tune against ... **real weave data once available**" |

### 5.3 yaml에 없어 헤더 기본값이 쓰이는 것

```
cte.deadband_m         0.25
cte.full_m             1.2
cte.min_factor         0.5
lane_change.hold_sec   4.0
lane_change.allowance_m 3.5
```

### 5.4 분석 과정에서 사용된 추정치 (근거 없음)

| 값 | 출처 | 신뢰도 |
|---|---|---|
| `τ_d ≈ 0.3 s` | `curv_coef=2.3` 에서 역산 | 순환논리 |
| `steer_tau ≈ 0.2~0.3 s` | 추측 | 미측정 |
| `ωn_max ≈ 1.67 rad/s` | "액추에이터 극점의 1/3" 일반 경험칙 | 이 차량 근거 아님 |
| `curv_coef ≥ 7·τ_d` | 위상여유 45° 기준 자체 유도 | 유도는 타당하나 `τ_d` 미상 |
| 목표 정착거리 15~25 m | 임의 설정 | 코스 요구사항 아님 |

### 5.5 검증 인프라 현황

- 워크스페이스에 waypoint_follower **rosbag 없음** (grid_map 데모 bag만 존재)
- `fit_from_bag.py` 는 **도구만** 존재, 실행 산출물 없음
- `wf_scenarios.py` 는 시나리오 단위 테스트로 **차량 플랜트 모델(lag/tau) 없음**
- config 주석의 "closed-loop simulation, 22 laps / 36 laps" 에 해당하는 **시뮬레이터가 워크스페이스에 없음**

---

## 6. 구 버전 대비 (`~/Downloads/src_은균이에게`)

### 6.1 구 버전 Stanley는 꺼져 있었음

```cpp
// 구 new_path_tracking.cpp:195-207
double ratio = 0.15;
double ratio_gain = abs(waypoints_[min_idx].rk);
(void)ratio_gain;                                   // 계산 후 미사용
ratio = 0.0;                                        // ★ 덮어씀
steer = (1 - ratio) * pure_pursuit() + ratio * stanly();
```

**조향 100% PP.** 실차 검증 이력은 PP에만 존재하고 **Stanley는 주행 이력 0**.

### 6.2 Stanley 구현 차이

| 항목 | 구 | 신 |
|---|---|---|
| 횡오차 게인 | `abs(waypoints_[i].rk)` — **곡률** | `k_dist = 0.8` 상수 |
| → 결과 | **직선에서 rk≈0 → 보정항 소멸** | 정상 동작 |
| 분모 | `speed_mps` 단독 (v→0 에서 ±90° 발산) | `k_damp·v + k_soft` |
| 횡오차 계산 | 최근접 **웨이포인트까지 거리** | **세그먼트 수직 투영** |
| 헤딩 기준 | `waypoints_[i].ryaw` (1.7° 지터) | 앞뒤 1 m 현 |
| 추가 항 | 없음 | `k_rate` (요레이트 D항), `k_steer` (지연 보상) |
| 속도 소스 | `ego_cur_speed_/3.6` | `speedMps()` (엔코더 우선) |

### 6.3 Ld 차이

구 버전은 `new_process.cpp:31~` 에서 미션 상태별 **하드코딩 switch** (`11/4.75/6/6/5.25/...`), 속도 무관 고정값.
이 테이블이 현재 `ldStateCapM()` 의 `kTable` 로 남아 있고 **죽은 코드** (§3.5).

### 6.4 규모

| | 구 | 신 |
|---|---|---|
| `new_path_tracking.cpp` | 208줄 | 787줄 |
| `waypointFollower.h` | 417줄 | 907줄 |
| 함수 수 | 5 | 21 |
| config yaml | **없음 (하드코딩)** | `tracking_params.yaml` |
| 곡률 속도 프로파일 | 없음 | 있음 |
| 지연 보상 | 없음 | `delay_comp_sec` |
| 조향 슬루 제한 | 없음 | `rate_limit_deg_s` |
| odom/path 워치독 | 없음 | 있음 |

---

## 7. 다음 할 일

### 코드
- [x] §3.1 `curvatureLdCap()` 호출을 지연 보상 이후로 이동
- [x] §3.2 차선 변경 시 `hold_hist_` / `ld_hold_hist_` clear
- [x] §3.6 `/wf_debug/cte` 발행 추가
- [x] `mission.uniform_kmh` 25.0 → **18.0**
- [ ] **빌드 검증 — 아직 안 됨** (`colcon build --packages-select waypoint_follower`)
- [ ] `record_fit_bag.sh` 에 `/wf_debug/cte` 추가 (bag 에 CTE 남기려면)
- [ ] §3.3~3.5 중복·죽은 코드 정리 (선택)

### 측정
- [ ] §4.6 안전 설정으로 1차 bag 수집 → `fit_from_bag.py` → §4.1 7개 반영
- [ ] `steer_tau` 확인 → §4.3 으로 `curv_coef` 하한 결정
- [ ] `a_lat_max` 단계 탐색 (1.5 → 3.0)

### 튜닝
- [ ] §4.5 순서대로 1~7단계
