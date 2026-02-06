# ZMP Preview Control 상세 설명 (Kajita et al. 2003)

## 📚 참고 논문
- **Kajita et al. 2003**: "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point"
- 이 파일의 핵심은 이 논문의 알고리즘을 구현한 것입니다.

---

## 🎯 ZMP Preview Control의 핵심 개념

### 1. LIPM (Linear Inverted Pendulum Model)
```
로봇의 동역학을 단순화한 모델

실제 로봇 (복잡함)
    ↓
LIPM (단순함)
    ↓
제어 가능!
```

### 2. ZMP (Zero Moment Point)
```
로봇이 넘어지지 않기 위한 조건:
ZMP가 지지 다각형(Support Polygon) 내에 있어야 함

ZMP = 발이 지면에 미치는 모멘트가 0인 점
```

---

## 📝 lib_ZMPctrl.py의 주요 함수 설명

### 1. `selectRobot()` 함수
**목적**: 로봇 선택 및 초기화

```python
def selectRobot(num, vel, step_len, stairs_height, spno):
    """
    로봇 선택 및 시뮬레이션 환경 설정
    
    Parameters:
    -----------
    num : int
        1 = Kondo khr3hv (소형 휴머노이드)
        2 = Unitree G1 (대형 휴머노이드)
    
    vel : float
        원하는 보행 속도 (m/s)
    
    step_len : float
        한 발 디딤의 길이 (m)
    
    stairs_height : float
        계단 높이 (m)
    
    spno : int
        1 = SSP (Single Support Phase) - 한 발로 서있는 상태
        2 = DSP (Double Support Phase) - 두 발로 서있는 상태
    
    Returns:
    --------
    humn : myRobot 객체
        로봇의 모든 정보 (질량, 관절, 발 위치 등)
    
    model : MuJoCo model
        시뮬레이션 모델
    
    data : MuJoCo data
        시뮬레이션 데이터
    """
    
    # 제어 게인 설정 (PD 제어)
    Kp = np.zeros(model.nu)  # 비례 게인 (Position gain)
    Kv = np.zeros(model.nu)  # 미분 게인 (Velocity gain)
    Ki = np.zeros(model.nu)  # 적분 게인 (Integral gain)
    
    # 다리 관절: 높은 게인 (정확한 제어 필요)
    Kp[0:12] = 10
    Kv[0:12] = 0.1
    
    # 팔/상체 관절: 낮은 게인
    Kp[12:] = 1
    Kv[12:] = 0.01
```

---

### 2. `myRobot` 클래스
**목적**: 로봇의 모든 정보를 관리

```python
class myRobot:
    def __init__(self, ub_jnts, left_legjnts, right_legjnts, foot_size, vel):
        """
        로봇 객체 초기화
        
        Attributes:
        -----------
        ub_jnts : array
            상체(Upper Body) 관절 인덱스
        
        left_legjnts : array
            왼쪽 다리 관절 인덱스
        
        right_legjnts : array
            오른쪽 다리 관절 인덱스
        
        foot_size : array [length, width]
            발의 크기 (m)
        
        vel : float
            보행 속도 (m/s)
        
        Control Parameters:
        -------------------
        ZMPctrl : float
            ZMP 제어 가중치
        
        posCTRL : bool
            True = 위치 제어 (Position Control)
            False = 토크 제어 (Torque Control)
        
        KINctrl : bool
            순기구학 기반 상태 추정 및 궤적 보정
        """
        pass
    
    def mj2humn(self, model, data):
        """
        MuJoCo 데이터 → 로봇 파라미터 변환
        
        계산 내용:
        ---------
        1. 순기구학 (Forward Kinematics)
           - 모든 링크의 위치 계산
        
        2. 질량 중심 (COM: Center of Mass)
           self.m = 전체 질량
           self.r_com = COM 위치 [x, y, z]
           self.dr_com = COM 속도 [vx, vy, vz]
        
        3. 발 위치
           self.o_left = 왼쪽 발 위치
           self.o_right = 오른쪽 발 위치
        
        4. 에너지
           self.E_init = 초기 에너지
                       = (1/2)*m*||v_com||² + m*g*z_com
        """
        mj.mj_fwdPosition(model, data)  # 순기구학 계산
        mj.mj_comVel(model, data)       # COM 속도 계산
        
        self.m = mj.mj_getTotalmass(model)           # 질량
        self.r_com = data.subtree_com[0].copy()      # COM 위치
        self.dr_com = data.subtree_linvel[0].copy()  # COM 속도
        self.o_left = data.site('left_foot_site').xpos.copy()   # 왼쪽 발
        self.o_right = data.site('right_foot_site').xpos.copy()  # 오른쪽 발
```

---

### 3. `lipm2humn()` 함수 - LIPM 기반 보행
**목적**: LIPM 동역학으로 보행 궤적 생성

#### LIPM 동역학 방정식:

**SSP (Single Support Phase) - 한 발 지지:**
```
x(t) = 1/2*(x(0) + Ts*ẋ(0))*exp(t/Ts) + 1/2*(x(0) - Ts*ẋ(0))*exp(-t/Ts)

여기서:
  x(t) = COM 위치
  Ts = √(h/g) = 특성 시간 (h: COM 높이, g: 중력)
  
이것은 쌍곡선 함수로 표현 가능:
  x(t) = x_zmp + (x(0) - x_zmp)*cosh(t/Ts) + Ts*ẋ(0)*sinh(t/Ts)
```

**DSP (Double Support Phase) - 양발 지지:**
```
x(t) = x_zmp + (x(0) - x_zmp)*cos(t/Td) + Td*ẋ(0)*sin(t/Td)

여기서:
  Td = √(h/g) = 특성 시간
  
이것은 원형 운동 (circular motion)
```

```python
def lipm2humn(self, dt, Tf, sspbydsp, vis):
    """
    LIPM 기반 보행 궤적 생성
    
    Parameters:
    -----------
    dt : float
        시뮬레이션 시간 스텝 (s)
    
    Tf : float
        전체 시뮬레이션 시간 (s)
    
    sspbydsp : int
        SSP 대 DSP의 비율
        예: 2 = SSP가 DSP의 2배 길이
    
    vis : int
        1 = 시각화 (그래프 표시)
        0 = 시각화 없음
    
    Algorithm:
    ----------
    1. 초기 조건 설정
       - COM 위치: rcm0
       - COM 속도: drcm0
       - ZMP 위치: rct (Contact Point)
    
    2. 각 시간 스텝에서:
       a) LIPM 동역학으로 COM 궤적 계산
       b) 발 디딤 조건 확인
       c) 필요시 발 위치 업데이트
    
    3. 결과 저장
       - self.tCMtraj = COM 시간 배열
       - self.oCMtraj = COM 위치 배열
       - self.oLtraj = 왼쪽 발 위치 배열
       - self.oRtraj = 오른쪽 발 위치 배열
    """
    
    # LIPM 특성 시간 계산
    h = rcm0[2] - rct[2]  # COM에서 ZMP까지의 높이
    Ts = np.sqrt(h / 9.81)  # 특성 시간
    
    # SSP에서 COM 궤적 계산
    if spno == 1:  # SSP
        rcm = rct + 1/2*(rcm0 - rct + Ts*drcm0)*np.exp((ti-t0)/Ts) \
                  + 1/2*(rcm0 - rct - Ts*drcm0)*np.exp(-(ti-t0)/Ts)
        drcm = 1/2*(rcm0 - rct + Ts*drcm0)*(1/Ts)*np.exp((ti-t0)/Ts) \
             - 1/2*(rcm0 - rct - Ts*drcm0)*(1/Ts)*np.exp(-(ti-t0)/Ts)
    
    # DSP에서 COM 궤적 계산
    else:  # DSP
        Td = np.sqrt(h / 9.81)
        rcm = rct + (rcm0 - rct)*np.cos((ti-t0)/Td) + drcm0*Td*np.sin((ti-t0)/Td)
        drcm = -(rcm0 - rct)*(1/Td)*np.sin((ti-t0)/Td) + drcm0*np.cos((ti-t0)/Td)
```

---

### 4. `mpc2humn()` 함수 - MPC 기반 ZMP Preview Control ⭐⭐⭐
**목적**: Kajita의 ZMP Preview Control 알고리즘 구현

#### 핵심 알고리즘 (Kajita 2003):

**1. 이산 시간 LIPM 상태 공간 모델:**
```
상태: x = [p, ṗ, p̈]ᵀ  (COM 위치, 속도, 가속도)

상태 방정식:
x(k+1) = A*x(k) + B*u(k)

여기서:
A = [1  dt  dt²/2]
    [0  1   dt   ]
    [0  0   1    ]

B = [dt³/6]
    [dt²/2]
    [dt   ]

출력 방정식:
p_zmp(k) = C*x(k) = [1, 0, -h/g]*x(k)

h = COM 높이
g = 중력 가속도
```

**2. 비용 함수 (Cost Function):**
```
J = Σ(p_zmp(k) - p_ref(k))² + λ*u(k)²

목표:
- ZMP를 기준 궤적(p_ref)에 추종
- 제어 입력(u) 최소화
```

**3. Preview Control Law (Kajita 2003):**
```
u(k) = -Gi*e_sum(k) - Gx*x(k) - Σ(Gd(j)*p_ref(k+j))

여기서:
  Gi = 적분 게인 (Integral gain)
  Gx = 상태 피드백 게인 (State feedback gain)
  Gd(j) = 미리보기 게인 (Preview gain)
  e_sum = 누적 오차 (Integral of error)
  
미리보기 게인 계산:
  Gd(j) = (B^T*P*B + R)^(-1)*B^T*X
  
  여기서 X는 Riccati 방정식의 해
```

```python
def mpc2humn(self, dt, Tf, trn, sspbydsp, step_time, step_len, vis):
    """
    MPC 기반 ZMP Preview Control
    
    Parameters:
    -----------
    dt : float
        시뮬레이션 시간 스텝 (s)
    
    Tf : float
        전체 시뮬레이션 시간 (s)
    
    trn : terrain 객체
        지형 정보
    
    step_time : float
        한 발 디딤 주기 (s)
    
    step_len : float
        한 발 디딤 길이 (m)
    
    vis : int
        시각화 여부
    
    Algorithm:
    ----------
    1. LIPM 파라미터 계산
       h = COM 높이
       Ts = √(h/g) = 특성 시간
    
    2. 이산 시간 상태 공간 모델 구성
       A, B, C 행렬 정의
    
    3. Riccati 방정식 풀이
       P = solve_discrete_are(A_aug, B_aug, Q, R)
       
       이것은 최적 제어 이득을 계산
    
    4. 피드백 및 미리보기 게인 계산
       K = (B^T*P*B + R)^(-1)*B^T*P*A_aug
       Gi = K[0, 0]      # 적분 게인
       Gx = K[0, 1:]     # 상태 피드백 게인
       Gd = [...]        # 미리보기 게인 배열
    
    5. 기준 ZMP 궤적 생성
       - 발 디딤 위치 계산
       - 각 발 디딤 주기마다 ZMP 설정
    
    6. Preview Control 루프
       for k in range(len(zmp_ref) - N):
           # 현재 ZMP 계산
           p = C @ x
           
           # 오차 계산
           e = p - zmp_ref[k]
           e_sum += e
           
           # 미리보기 합 계산 (N 스텝 미리보기)
           preview_sum = Σ(Gd[j]*zmp_ref[k+j+1])
           
           # 제어 입력 계산
           u = -Gi*e_sum - Gx@x - preview_sum
           
           # 상태 업데이트
           x = A@x + B*u
    """
    
    # LIPM 파라미터
    h = rcm0[2] - rct[2]  # COM 높이
    g = 9.81              # 중력
    Ts = np.sqrt(h / g)   # 특성 시간
    
    # 이산 시간 상태 공간 모델
    A = np.array([[1, dt, dt**2/2],
                  [0, 1, dt],
                  [0, 0, 1]])
    
    B = np.array([[dt**3/6],
                  [dt**2/2],
                  [dt]])
    
    C = np.array([[1, 0, -h/g]])  # ZMP = p - (h/g)*p̈
    
    # 비용 함수 가중치
    Qe = 1.0      # ZMP 오차 가중치
    Qx = np.zeros((3, 3))  # 상태 가중치
    R = 1e-6      # 제어 입력 가중치
    
    # Riccati 방정식 풀이
    P = solve_discrete_are(A_aug, B_aug, Q, R)
    
    # 게인 계산
    K = np.linalg.inv(B_aug.T @ P @ B_aug + R) @ (B_aug.T @ P @ A_aug)
    Gi = K[0, 0]      # 적분 게인
    Gx = K[0, 1:]     # 상태 피드백 게인
    
    # 미리보기 게인 계산
    Gd = np.zeros(N)
    AcBK = A_aug - B_aug @ K
    X = -AcBK.T @ P @ np.array([[1], [0], [0], [0]])
    for i in range(N):
        Gd[i] = (np.linalg.inv(B_aug.T @ P @ B_aug + R) @ (B_aug.T @ X)).item()
        X = AcBK.T @ X
```

---

## 🔑 핵심 개념 정리

### ZMP Preview Control의 장점:
1. **미리보기 (Preview)**: 미래의 기준 ZMP를 미리 알고 제어
2. **최적성 (Optimality)**: Riccati 방정식으로 최적 게인 계산
3. **안정성 (Stability)**: ZMP를 지지 다각형 내에 유지

### 제어 구조:
```
기준 ZMP 궤적 (Reference)
    ↓
Preview Control
    ↓
제어 입력 (u = 가속도)
    ↓
LIPM 상태 업데이트
    ↓
COM 궤적
    ↓
역기구학 (IK)
    ↓
관절 각도
    ↓
로봇 움직임
```

### 주요 파라미터:
- **h**: COM 높이 (m)
- **Ts**: 특성 시간 = √(h/g)
- **N**: 미리보기 스텝 수 (보통 1000)
- **dt**: 시뮬레이션 시간 스텝 (보통 0.005s)
- **step_time**: 한 발 디딤 주기 (s)
- **step_len**: 한 발 디딤 길이 (m)

---

## 📊 Kajita 2003 논문의 핵심 수식

### 1. LIPM 동역학 (연속 시간):
```
ẍ = (g/h)*(x - x_zmp)

여기서:
  x = COM 위치
  x_zmp = ZMP 위치
  h = COM 높이
  g = 중력
```

### 2. 이산화 (Discretization):
```
x(k+1) = A*x(k) + B*u(k)
y(k) = C*x(k)

u(k) = 가속도 (제어 입력)
y(k) = ZMP (출력)
```

### 3. Preview Control Law:
```
u(k) = -Gi*∫e(τ)dτ - Gx*x(k) - Σ(Gd(j)*r(k+j))

r(k) = 기준 ZMP
e(k) = y(k) - r(k) = 오차
```

---

## 💡 실제 사용 예시

```python
# 1. 로봇 선택 및 초기화
humn, model, data = selectRobot(
    num=1,              # Kondo khr3hv
    vel=0.1,            # 0.1 m/s
    step_len=0.05,      # 5cm 스텝
    stairs_height=0.01, # 1cm 계단
    spno=1              # SSP 시작
)

# 2. MPC 기반 보행 생성
humn.mpc2humn(
    dt=0.005,           # 5ms
    Tf=10.0,            # 10초
    trn=terrain,        # 지형
    sspbydsp=2,         # SSP:DSP = 2:1
    step_time=0.5,      # 0.5초 주기
    step_len=0.05,      # 5cm
    vis=1               # 그래프 표시
)

# 3. 시뮬레이션 실행
# 로봇이 ZMP Preview Control로 보행!
```

---

## 🎓 학습 순서

1. **LIPM 이해**: 로봇 동역학 단순화
2. **ZMP 개념**: 안정성 조건
3. **상태 공간 모델**: 이산 시간 시스템
4. **Riccati 방정식**: 최적 제어
5. **Preview Control**: 미리보기 기반 제어
6. **구현**: lib_ZMPctrl.py 분석

---

## 📖 참고 자료

- **Kajita et al. 2003**: "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point"
- **Katayama et al. 1985**: "Design of an Optimal Controller for a Discrete-Time System Subject to Previewable Demand"
- **Wieber 2006**: "Trajectory Free Linear Model Predictive Control for Stable Walking in the Presence of Strong Perturbations"
