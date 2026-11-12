# 제어 코드 및 실험 설정 / Control Code and Experiment Setup

[한국어 README](../README.md) · [English README](../README_EN.md)

## 구성 개요

이 저장소는 **C 기반 DISCON 추가 제어기**, OpenFAST/ROSCO 연결 설정, 시간별 제어 경로 JSON, 저장된 72개 실험 결과 및 시각화 스크립트를 함께 제공한다.

- **[controller/discon_offset_reference.c](../controller/discon_offset_reference.c)**: 기존 ROSCO의 `DISCON`을 호출한 후 토크·피치 오프셋, 명령 포화·변화율 제한, 요각 참조 추종 및 출력 기록 적용
- **[configs/historical](../configs/historical/)**: Baseline/Controlled의 `ServoDyn.dat`, `DISCON.IN` 설정 입력. 두 `DISCON.IN`은 동일하며 `ServoDyn.dat`는 `DLL_InFile` 참조명이 다르다
- **[configs](../configs/)**: 풍황 전환과 대표 최고/최저 경로의 시점별 오프셋 설정 JSON
- **[data/openfast_72](../data/openfast_72/)**: 72개 경로 결과 및 대표 시계열
- **[scripts](../scripts/)**: 입력 생성·결과 검증·그래프 생성 Python 코드

## C DISCON 래퍼의 구조

```text
OpenFAST turbine states -> original ROSCO DISCON
                                     |
                                base commands
                                     v
CONTROL_SCHEDULE CSV --> torque / pitch offsets + rate/limit checks
YAW_REFERENCE_CSV ----> yaw-angle reference + yaw-rate correction
                                     |
                                     v
                              ServoDyn commands
```

| 환경변수 | 의미 |
|:--|:--|
| `ROSCO_REAL_LIBRARY` | 기존 ROSCO 공유 라이브러리 경로 |
| `CONTROL_SCHEDULE` | 시점별 제어 이벤트 CSV 경로 |
| `YAW_REFERENCE_CSV` | 선택적 기준 요각 CSV 경로 (설정되면 보간 사용) |
| `CONTROL_LOG` | 기본/수정 제어 명령 기록 CSV 경로 |
| `OFFSET_TORQUE_MAX_NM` | 토크 상한 (기본 47,402.91 N·m) |
| `OFFSET_TORQUE_RATE_NM_S` | 토크 변화율 제한 (기본 15,000 N·m/s) |
| `OFFSET_PITCH_MAX_DEG` | 피치 상한 (기본 90°) |
| `OFFSET_PITCH_RATE_DEG_S` | 피치 변화율 제한 (기본 8°/s) |
| `OFFSET_YAW_RATE_DEG_S` | 요속도 제한 (기본 0.3°/s) |
| `OFFSET_YAW_KP_S_INV` | 요각 추종 이득 (기본 0.2 s⁻¹) |

**`CONTROL_SCHEDULE` CSV 형식**

```csv
time_s,torque_offset_nm,pitch_offset_deg,yaw_offset_deg
30,2500,-1,-3
100,0,0,0
```

각 이벤트부터 다음 이벤트 전까지 오프셋 값이 유지되며, 해제 이벤트에서 0으로 복귀한다. `YAW_REFERENCE_CSV`는 `time_s,yaw_angle_deg` 형식이며, 지정 시 기준 실행의 요각을 시간 보간해서 요속도 보정에 사용한다. CSV를 지정하지 않으면 코드에 구현된 가상 요각 기준 모드로 동작한다.

## 결과 확인

```bash
python -m pip install -r requirements.txt
python scripts/check_openfast72_data.py
python scripts/plot_openfast72_from_csv.py
python scripts/prepare_inputs.py --events configs/openfast72_best_schedule.json --out build
```

이 명령들은 저장된 72개 결과를 검사하고 시각화하거나 풍황·제어 이벤트 입력을 생성한다. **OpenFAST 자체의 시뮬레이션 실행**에는 공식 NREL 5-MW 모델 입력, OpenFAST와 ROSCO 설치 및 연결 설정이 추가로 필요하다.

## 결과 해석

- 평가 대상: 한 풍황 전환 조건 아래 **72개 이산 제어 경로**
- 비교 지표: 발전기 출력 `GenPwr`를 적분한 **발전기 전기에너지**
- 기준 경로와의 비교: 추가 명령뿐 아니라 기준 요각 추종 보정을 포함한 대안 제어 구조 비교
- 한계: 일반적인 최적 제어기 도출, 구동기 소비전력 차감 및 예측 오차 평가는 연구 범위 밖

---

## English summary

This repository showcases a **custom C-based DISCON control wrapper** on top of the original ROSCO feedback controller. Its implementation includes scheduled torque/pitch offsets, rate and saturation limits, yaw-reference tracking, and command logging. Historical ServoDyn/ROSCO configuration inputs and 72-case simulation results are supplied alongside Python analysis scripts.

Use `CONTROL_SCHEDULE` for the time-stamped CSV above; `YAW_REFERENCE_CSV` optionally supplies `time_s,yaw_angle_deg` baseline data. The Python commands analyze archived data and prepare inputs. Running OpenFAST itself requires an appropriate NREL 5-MW model and installed OpenFAST/ROSCO environment.
