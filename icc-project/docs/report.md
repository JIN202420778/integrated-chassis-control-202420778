# [202420778-진우정] ICC 제어기 설계 보고서

**과목**: C050-2자동제어 
**제출일**: 2026-06-23
**팀**: 개인-진우정(202420778)

---

## 1. 설계 개요 (1 페이지)

이 과제의 목표는 14DOF차량 동역학 시스템 내에서 6개의 표준 주행 시나리오에서 안정적으로 동작하고 규정된 KPI를 만족하는 integrated-chassis 제어 시스템을 설계하는 것이다. 
특히, 동적 한계 내에서 타이어의 lock-up을 방지하고 조향 및 제동 상황에서의 횡방향 안정성과 경로 추종 성능을 최적화하여 차량의 주행 안전성을 극대화하고자 하였다.
Rajamani(2012)의 Vehicle Dynamics and Control에 따르면 SMC, LQR 등의 기법은 불확실성이 높은 환경에서 파라미터 튜닝이 어렵고 overshoot 발생 위험이 크기 때문에 
초기에는 동적 응답을 체계적으로 조절할 수 있으며 시스템을 시나리오 별로 최적화하여 안정성을 우선적으로 확보할 수 있는 PID 기법을 채택하였으나, 
시뮬레이션 결과 적분항과 미분항이 일부 시나리오에서 진동과 과도응답을 증가시키는 것을 확인하였다.
따라서 최종 설계에서는 응답 속도와 진동 억제 사이의 trade-off를 단순화한 P 제어 기반의 Robust 제어 전략을 채택하여 
제어 입력을 물리적인 조향 한계 및 제동 토크 limit 내로 제한하여 복잡한 제어 로직 간의 간섭을 최소화하고 모든 시나리오에서의 완주 가능성을 높이는 데에 주력하였다.

각 제어기 한 줄 요약:
- **ctrl_lateral**: P 제어 기반 yaw rate 추종 및 조향각 제한을 통한 횡방향 안정성 확보
- **ctrl_longitudinal**: P 제어 기반 종방향 속도 추종 및 타이어 락업 방지를 위한 최대 제동 토크 제한
- **ctrl_vertical**: 고정 게인 기반의 패시브 댐핑 제어
- **ctrl_coordinator**: 총 제동력의 4륜 균등 분배 방식

---

## 2. 수학적 모델링 (1-2 페이지)

### 2.1 사용한 plant 단순화
본 과제의 최종 검증은 14DOF 차량 모델에서 수행되고 14DOF 모델은 차량의 종방향, yaw, roll, pitch, 각 바퀴의 운동까지 포함하므로 실제 차량 거동을 비교적 정확하게 모사할 수 있지만
제어기 설계 단계에서는 보다 단순한 선형 bicycle 모델을 사용하였다. 
횡방향 거동을 결정하는 주요 변수는 차량의 lateral velocity와 yaw rate이고 AFS 설계 역시 이러한 변수만으로 차량의 기본적인 선회 특성을 설명할 수 있다.
또한 본 설계에서는 복잡한 상태 추정기나 최적 제어기를 사용하지 않고 yaw rate의 오차를 기반으로하는 단순 피드백 구조를 채택하였으므로 bicycle model이 적절하다고 판단하였다.


### 2.2 State-space 표현

횡방향 차량 거동은 다음 상태변수로 표현하였다.
x = [vy, r]^T
vy: lateral velocity, r: yaw rate

입력은 전륜 조향각 δ로 정의하였다.
u = δ

출력은 yaw rate로 정의하였다.
y = r

선형 Bicycle Model의 상태방정식은 다음과 같다.
x_dot = Ax + Bu

vy_dot = -((Cf + Cr)/(mVx))·vy + (((lr·Cr - lf·Cf)/(mVx)) - Vx)·r + (Cf/m)·δ
r_dot = ((lr·Cr - lf·Cf)/(Iz·Vx))·vy - ((lf²·Cf + lr²·Cr)/(Iz·Vx))·r + (lf·Cf/Iz)·δ

A =
[ -(Cf+Cr)/(mVx)       (lrCr-lfCf)/(mVx)-Vx ]
[ (lrCr-lfCf)/(IzVx)  -(lf²Cf+lr²Cr)/(IzVx) ]
B =
[ Cf/m ]
[ lfCf/Iz ]

상태 행렬 A : 차량의 물리적 파라미터와 현재 주행 속도에 의해 결정된다.
입력 행렬 B : 조향 입력이 횡방향 가속도 및 yaw momnent에 미치는 영향을 나타낸다. 

m: 차량 질량
Iz : yaw 관성모멘트
lf : CG에서 전축까지 거리, lr : CG에서 후축까지 거리
Cf : 전륜 코너링 강성, Cr : 후륜 코너링 강성
Vx : 종방향 속도

따라서 상태공간 모델은 차량의 횡속도(vy)와 yaw rate(r)의 변화를 조향 입력 δ에 대해 선형 근사적으로 나타낸다.

본 설계는 실제 상태 공간 제어기를 구현하지는 않았으나
yaw rate 추종을 위한 횡방향 동역학을 설명하기 위해 bicycle model을 사용하였다.

실제 구현된 제어기는 yaw rate 오차
e_r = r_ref - r
를 이용하여 P 제어 기반의 AFS 보조 조향 방식으로 구성하였다.

### 2.3 가정 + 한계
1. 제어기 설계 시 종방향 속도는 일정하다고 가정
2. 타이언ㄴ 소슬립 영역에서 동작하며 선형 코너링 강성을 가진다고 가정
3. 좌우 타이어 특성이 동일하다고 가정
4. 노면 마찰계수가 일정하다고 가정
5. 차량의 roll 및 pitch 운동은 제어기 설계 단계에서 고려하지 않음

이러한 가정은 bicycle 모델의 단순화를 가능하게 하지만
실제 차량의 비선형 거동과 타이어의 포화 현상을 완전히 반영하지는 못한다.
따라서 고속 급조향이나 강제동 등의 상황에서 오차가 발생할 수 있다. 

## 3. 제어기 설계 (3-4 페이지)

### 3.1 ctrl_lateral 
**설계 목표**:
-yaw rate 추종을 통한 조향 안정성 확보
-조향 한계 설정을 통한 경로 추종의 정밀성 및 횡방향 안정성 확보

**선택 기법**: P 제어 기반의 Robust control
초기 설계 단계에서 PID 제어를 검토하였으나 
시뮬레이션 결과 미분항에 의한 노이즈 민감도와 적분항에 의한 과도 응답 지연이 고속 주행 시 차량 거동을 불안정하게 함을 확인하고
응답성을 단순화하고 안정성을 극대화하기 위한 P control 기법을 채택하였다.

**Gain 계산 과정**:
yaw rate 오차:
e_r = r_ref − r
(r_ref: 목표 yaw rate, r: 실제 yaw rate)

보조 조향각:
δ_AFS = K_r · e_r

조향각 한계 5도 설정:
max_afs = deg2rad(5.0);

**최종 게인 + 정당화**:
k_r=1.0

시뮬레이션 결과 A3 시나리오에서 
Overshoot = 2.41%
Rise Time = 0.057s
Settling Time =0.633s
를 달성하여 모든 KPI를 만족하였다.

### 3.2 ctrl_longitudinal 
**설계 목표**:
-속도 추종
-제동 시 차량 안정성 확보
-타이어 lock-up 최소화

**선택 기법**:
-속도 오차 기반 P 제어

**Gain 계산 과정**:
속도 오차: 
e_v = v_ref − v_x

종방향 힘:
F_x = K_v · e_v

급제동 시 과도한 wheel slip을 방지한 최대 제동력 제한(차량 중량 기반 제동력의 약 30% 수준)
Fx = -1500 * 9.81 * 0.3

**최종 게인 + 정당화**:
K_v = 1500


### 3.3 ctrl_vertical 
**설계 목표**: 
-차체 수직 진동 억제 및 타이어 접지력 유지
-다른 제어기와의 간섭 최소화

**선택 기법**:
시스템 복잡도를 낮추고 안정적인 접지력을 확보하기 위해 
모든 휠에 동일한 감쇠계수를 적용하는 고정 gain 방식의 패시브 댐핑 방식을 사용하였다. 


**Gain 계산 과정**:
dampingCmd = 1500 * ones(4,1);

**최종 게인 + 정당화**:
Damping Coefficient = 1500 Ns/m
별도의 gain tuning은 수행하지 않았으며 기본 감쇠 특성을 유지하는 방향으로 설계하여
수직 방향 성능 개선보다 횡방향 안정성과 제동 성능 확보를 우선적으로 고려하였다. 

### 3.4 ctrl_coordinator 
**설계 목표**:
-상위 제어기에서 생성한 종방향 힘Fx을 실제 브레이크 토크로 변환
-각 휠에 안정적으로 제동력 분배
-과도한 wheel slip 및 lock-up 방지
-단순하고 안정적인 actuator allocation 구현

**선택 기법**:
본 설계에서는 ESC 기반 차동 제동이나 최적화 기반 actuator allocation 대신
단순한 균등 토크 분배 방식을 사용하였다. 
초기 시뮬레이션 결과 복잡한 차동 제동이 일부 시나리오에서 차량 거동을 불안정하게 하여 
모든 휠에 동일한 제동 토크를 분배하는 구조를 채택하였다. 

또한 wheel slip을 줄이기 위해 휠 당 최대 브레이크 토크를 제한하였다.

**Gain 계산 과정**:
종방향 제어기에서 생성된 종방향 힘을 브레이크 토크로 변환:
T = |Fx| * rw / 4
Fx: 총 종방향 제동력
rw: 타이어 유효 반경

계산된 브레이크 토크를 모든 휠에 동일하게 분배:
TFL = TFR = TRL = TRR = T

wheel lock을 방지한 각 휠의 최대 브레이크 토크 제한:
Ti ≤ 400 Nm
시뮬레이션에서 400Nm 이상의 토크를 사용할 경우 absSlipRMS가 크게 증가하고 지나치게 낮은 토크는 제동 거리를 증가시키는 것을 확인하고
400Nm을 최종 제한값으로 선택하였다.

**최종 게인 + 정당화**:
Wheel Radius = 0.31m 
Max Brake Torque = 400Nm 
Allocation Method = Equal Distribution

복잡한 ESC 기반 토크 벡터링은 적용하지 않았지만 
단순한 구조로도 안정적인 동작을 확보할 수 있었으며 과도한 타이어 slip을 억제할 수 있었다.

## 4. 시뮬레이션 결과 (2-3 페이지)

### 4.1 P1 시나리오 benchmark — 베이스라인 vs 본인 설계

| 시나리오 | KPI | OFF | ON (본인) | Δ% |
|---|---|---|---|---|
| A1 DLC | sideSlipMax [°] | 3.02 | 2.60 | -13.8% | --score 6/6
| A1 | LTR_max | 0.864 | 0.752 | -12.9% | --score 3.73/5
| A3 step | yawRateOvershoot [%] | 2.70 | 2.41 | -10.6% | --score 4/4
| A4 SS | understeerGradient | -- | 0.000748 | -- | --score 5/5
| A7 BIT | sideSlipMax [°] | 30.48 | 1.90 | -93.8% | --score 8/8
| A7 | LTR_max | 0.68 | 0.33 | -52.2% | --score 7/7
| B1 brake | stoppingDistance [m] | 72.30 | 72.30 | -0.0% | --score 0/5
| D1 통합 | sideSlipMax [°] | 4.91 | 3.16 | -35.7% | --score 4/4

본 설계의 정량적 성능은 총점 52.23점/70점으로 평가되었다. 
가장 큰 성능 향상은 A7 break in turn 시나리오에서 sideSlipMax가 93.8% 개선으로 나타났고 LTR_max 역시 52.2% 감소하였다.
이는 AFS 기반 yaw rate feedback 제어가 제동 중 차량의 횡방향 안정성을 효과적으로 향상시켰음을 의미한다. 
4륜 균등 제동 분배를 통해 타이어의 횡방향 마찰력을 끝까지 유지시킬 수 있다.

A1, D1 시나리오에서도 SideSlipAngle과 Load Transfer Ration를 유의미하게 낮춤으로써 
차량의 전복 위험성을 줄이고 경로 추종 중 차체 거동의 안정성을 확보하였다.

A3 시나리오에서 yaw rate overshoot이 10.6%로 억제되었고
이는 P control 기반의 조향 한계 5도 설정이 차량의 과도한 조향을 물리적으로 방지하고 yaw rate를 안정적으로 추종하도록 설계되었음을 의미한다.

B1 시나리오에서는 제동 거리가 베이스라인과 동일한 결과를 보이며 제동 효율 측면에서의 개선이 이루어지지 않았다.
시스템의 안정적인 완주와 lock-up 방지를 위해 선택한 제동력 제한(최대 제동 토크 400Nm)이 제동 거리 단축과 trade-off 관계에 있어 성능 제약이 생겼다.


### 4.2 핵심 plot — A1 DLC

![A1 trajectory comparison](figures/a1_trajectory.png)
*Figure 4.1 — A1 ISO 3888-1 DLC, 차량 trajectory (off vs on) vs reference path.*
```matlab
[r_off, ~] = run_icc_scenario('A1','14dof','Controller','off','SavePlot',false);
[r_on , ~] = run_icc_scenario('A1','14dof','Controller','on' ,'SavePlot',false);

figure;
plot(r_off.x_pos,r_off.y_pos,'r--','LineWidth',1.5); hold on;
plot(r_on.x_pos ,r_on.y_pos ,'b-' ,'LineWidth',1.5);

if isfield(r_off.scenario,'refPath')
    plot(r_off.scenario.refPath(:,1), ...
         r_off.scenario.refPath(:,2), ...
         'k:','LineWidth',2);
    legend('Controller OFF','Controller ON','Reference Path');
else
    legend('Controller OFF','Controller ON');
end

grid on;
axis equal;
xlabel('X Position [m]');
ylabel('Y Position [m]');
title('A1 DLC Trajectory Comparison');

saveas(gcf,'a1_trajectory.png');
```
controller OFF상태와 ON 상태의 주행 궤적을 비교하였다.
두 경우 모두 기준 경로를 전반적으로 추종하였으나 controller 사용 시 차량 궤족이 기준 경로에 더욱 근접하게 유지된다.
다만 궤적 차이가 크지는 않았으며 이는 본 설계가 ESC의 개입보다는 AFS를 중심으로 설계되었기 때문이다.
KPI의 결과에서는 sideSlipMax는 3도 이하로 유지되어 차량 안정성은 확보되었으나 lateral deviation의 개선 효과는 제한적이었다.

![A1 yaw rate](figures/a1_yawrate.png)
*Figure 4.2 — A1 yaw rate 응답: reference (driver bicycle model), off (controller off), on (본인 설계).*
```matlab
[r_off, ~] = run_icc_scenario('A1','14dof','Controller','off','SavePlot',false);
[r_on , ~] = run_icc_scenario('A1','14dof','Controller','on' ,'SavePlot',false);

figure;

plot(r_off.t,r_off.yawRate,'r--','LineWidth',1.5); hold on;
plot(r_on.t ,r_on.yawRate ,'b-' ,'LineWidth',1.5);
plot(r_on.t ,r_on.yawRateRef,'k:','LineWidth',2);

grid on;
xlabel('Time [s]');
ylabel('Yaw Rate [rad/s]');
legend('OFF','ON','Reference');
title('A1 DLC Yaw Rate Response');

saveas(gcf,'a1_yawrate.png');
```
기준 yaw rate와 차량의 실제 yaw rate 응답을 비교하였다. 
controller OFF 상태에서는 기준 응답 대비 진폭 오차와 위상 지연이 일부 존재하였는데
controller 사용 시 P 기반 조향 보조 제어기가 yaw rate 오차에 비례한 추가 조향각을 생성하여 실제 yaw rate가 기준 응답에 더욱 근접하게 동작한다.
특히 1, 2번째 조향 구간에서 응답 추종성이 향상되었고 과도한 overshoot없이 안정적으로 수렴하게 되었다. 

### 4.3 한 시나리오 deep dive — A7 

A7 brake-in-turn 의 핵심:
선회 중 제동이 동시에 발생하는 복합 시나리오로 차량 안정성 평가에 중요한 조건이다.
- 베이스라인 sideSlipMax 30.48°
- 본인 설계: 1.90° 
- 핵심 요인: yaw rate 오차를 이용한 AFS 보조 조향의 영향으로 해석할 수 있다. 
            제어기는 목표 yaw rate와 실제 yaw rate의 차이를 지속적으로 보상하여 차량이 의도된 궤적을 유지하도록 설계하였다.

controller OFF 상태에서 3초 이후 slip angle이 급격하게 증가하여 sideSlipMax는 30.48°까지 증가하였으며 
이는 차량이 횡방향 안정성을 잃고 심한 언더스티어 또는 스핀에 가까운 거동을 보임을 의미한다.
controller 사용 시 slip angle이 전 구간에서 2%이내로 유지되었으며 제동이 시작되는 구간에서도 slip angle의 추가적인 증가가 발생하지 않고 점진적으로 0도 부근으로 수렴한다.
이는 본 설계에서 적용한 AFS yaw rate오차를 감소시키고 
제한된 제동 토크가 타이어 포화 및 급격한 횡력 손실을 방지하여 차량 자세 안정화에 기여하였기 때문이다. 
---

## 5. 분석 + 한계 (1-2 페이지)

### 5.1 가장 성공적이었던 시나리오
가장 우수한 성능을 보인 시나리오는 A7 Break-In-Turn 이다. 
해당 시나리오에서 sideSlipMax는 1.90°를 기록하였고 LTR_max는 0.33로 목표에 달성하였다.
A7의 선회와 제동이 동시에 발생하는 복합 상황에서 
yaw rate 오차에 비례한 추가 조향 AFS를 적용하여 차량 자세를 안정화하였고
제동 토크를 제한하여 과도한 타이어 slip을 방지하였다.

### 5.2 가장 부족했던 시나리오
가장 성능이 부족했던 시나리오는 B1 Braking이다.
stopdistance는 72.30m로 목표값인 40m를 크게 초과하였고 absSlipRMS 역시 0.73으로 목표값 0.1을 만족하지 못했다. 
- 가설 1: 제동력 제한에 따른 제동 성능 감소
본 설계에서는 타이어의 lock-up 방지를 위해 제동력을 차량 최대 성능의 30%수준으로 제한하였기 때문에
타이어 slip은 일부 감소하나 차량 감속 성능이 크게 저하되고 결과적으로 제동거리가 증가한다.
- 가설 2: ABS 알고리즘 미구현
ABS 제어를 사용하지 않아 wheel slip을 능동적으로 제어하지 못하였고 absSlipRMS가 높은 수준으로 유지되었다. 


### 5.3 만약 더 시간이 있었다면
-slip ratio 기반 ABS 제어기 구현
-PI/PID 기반 종방향 제어기 적용
-slip angle 기반 ESC yaw moment 제어 추가
-yaw moment를 이용한 차동 제동 분배 알고리즘
-속도에 따른 gain scheduling
---

## 6. 참고문헌

[1] ISO 3888-1:2018 — Passenger cars — Test track for a severe lane-change manoeuvre.
[2] ISO 4138:2021 — Steady-state circular driving behaviour.
[3] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer 2012. §2.5 (yaw rate response), §8 (ESC).
[4] J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley 2008.
[5] Ackermann, J., "Robust Control Prevents Car Skidding," IEEE Control Systems Magazine, vol. 17, no. 3, 1997.

---

## 부록 A — 사용한 AI 도구

'Gemini used for discussion of controller concepts and gain tuning. ChatGPT used for MATLAB code development and plotting.';
Gemini를 횡방향 및 종방향 제어기의 구조에 대한 아이디어 검토 및 제어기 gain 조정 방향에 대해 논의하는 데에 활용하였다.
ChatGGPT를 코드 오류 수정 방안 논의 및 plot 코드 작성에 활용하였다.

---

## 부록 B — 본인 sim_params.m 변경사항

```matlab
% 변경 전:
%   CTRL.LAT.Kp = 1.0
%   CTRL.LAT.Ki = 0.1

% 변경 후:
%초기 설계 단계에서는 CTRL.LAT을 활용한 PID제어 구조를 고려하였으나 최종 구현 과정에서 단순 P 제어 구조를 사용하였다
따라서 실제 제어기는 sim_params.m의 gain 을 참조하지 않고 코드 내부에서 직접 정의한 gain을 사용하였다.
% ctrl_lateral.m
steer_req = 1.0 * e_r;
max_afs = deg2rad(5.0);

% ctrl_longitudinal.m
Fx = 1500 * e;
Fx_brake_limit = -1500 * 9.81 * 0.3;

% ctrl_coordinator.m
brakeTorqueLimit = 400;   % Nm
% ...
```
