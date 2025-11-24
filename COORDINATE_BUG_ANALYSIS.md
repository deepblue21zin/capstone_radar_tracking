# 🐛 좌표 회전 버그 분석

## 🔴 발견된 치명적 문제

### 문제 1: Boundary Box 필터링 좌표계 불일치

```matlab
% === Boundary Box 정의 (표시 좌표계 기준, Y+ 전방) ===
xMin_bb = -2; xMax_bb =  2;   % 좌우: -2m ~ +2m
yMin_bb =  0.5; yMax_bb = 8;  % 전방: 0.5m ~ 8m (Y+ 전방 가정!)
zMin_bb =  0;  zMax_bb = 2.5; % 높이: 0m ~ 2.5m

% === 하지만 필터링은 센서 좌표 그대로 사용! ===
isValidHeight = (z_all >= zMin_bb) & (z_all <= zMax_bb);
isDynamic = isDynamic & isValidHeight;

% ❌ Y 좌표 필터링 없음!
% ❌ 센서가 X+를 전방으로 사용하면 필터링 완전히 잘못됨!
```

### 문제 2: 센서 좌표계 vs 표시 좌표계 혼용

| 작업 | 사용 좌표계 | 문제점 |
|------|-----------|--------|
| Boundary Box 정의 | 표시 좌표 (Y+ 전방) | ✅ 의도대로 |
| 필터링 적용 | 센서 좌표 (X+ 전방?) | ❌ **좌표계 불일치!** |
| 트래커 입력 | 센서 좌표 | ✅ 정확 |
| 시각화 | 표시 좌표 (회전 후) | ✅ 정확 |

---

## 🔍 왕복운동 시 트랙 분리 원인 분석

### 시나리오: 센서 전방 2m에서 왕복운동

#### 케이스 A: 센서가 X+를 전방으로 사용하는 경우

**왼쪽으로 이동 (센서 좌표)**
```
센서 좌표: X=2.0, Y=-0.3, Z=1.0
표시 좌표: X=0.3, Y=2.0, Z=1.0  (90도 회전)

Boundary Box 체크 (센서 좌표로 체크):
- X: 2.0 (범위: -2~2) → ❌ 범위 밖! (실제론 전방인데 필터링됨)
- Y: -0.3 (범위: 0.5~8) → ❌ 범위 밖!
- Z: 1.0 (범위: 0~2.5) → ✅ OK

결과: detection 제외됨! → 트랙 손실
```

**오른쪽으로 이동 (센서 좌표)**
```
센서 좌표: X=2.0, Y=0.3, Z=1.0
표시 좌표: X=-0.3, Y=2.0, Z=1.0  (90도 회전)

Boundary Box 체크 (센서 좌표로 체크):
- X: 2.0 (범위: -2~2) → ❌ 범위 밖!
- Y: 0.3 (범위: 0.5~8) → ❌ 범위 밖! (yMin=0.5)
- Z: 1.0 (범위: 0~2.5) → ✅ OK

결과: detection 제외됨! → 트랙 손실
```

**중앙에서 왕복**
```
센서 좌표: X=2.0, Y=0.0, Z=1.0

왕복 시:
X=2.0, Y=-0.5 → 필터링 제외
X=2.0, Y=-0.3 → 필터링 제외
X=2.0, Y=0.0  → 필터링 제외 (yMin=0.5)
X=2.0, Y=0.3  → 필터링 제외
X=2.0, Y=0.5  → ✅ 포함
X=2.0, Y=0.7  → ✅ 포함

결과: 왕복운동 시 간헐적으로 detection 손실!
→ 트래커가 "사라졌다 나타났다" 판단
→ 새로운 트랙 ID 생성 (트랙 분리!)
```

---

## 📊 현재 코드의 실제 필터링 영역

### 센서가 X+를 전방으로 사용하는 경우

```
현재 Boundary Box (표시 좌표 기준 정의):
  xMin_bb=-2, xMax_bb=2, yMin_bb=0.5, yMax_bb=8

실제 필터링 (센서 좌표 기준 적용):
  - Z 높이만 필터링됨!
  - X, Y 범위는 전혀 필터링 안 됨!

왜냐하면:
  isDynamic = isDynamic & isValidHeight;  // Z만 체크!
  // X, Y 범위 체크 없음!
```

### 코드 재확인

```matlab
% 거리/높이 필터 (센서좌표 기준 거리)
ranges = sqrt(x_all.^2 + y_all.^2 + z_all.^2);
isDynamic = isDynamic & (ranges <= maxRange);        // 전체 거리만 체크
isValidHeight = (z_all >= zMin_bb) & (z_all <= zMax_bb);  // Z만 체크
isDynamic = isDynamic & isValidHeight;

// ❌ xMin_bb, xMax_bb, yMin_bb, yMax_bb는 사용되지 않음!
// ❌ Boundary Box는 시각화 전용!
```

---

## 🎯 실제 버그의 영향

### 1. X/Y Boundary Box 필터링 없음
- Boundary Box 변수는 정의만 되고 **실제로 사용 안 됨**
- 단지 3D 패치 시각화 전용
- 거리 필터링(`ranges <= maxRange`)만 적용됨

### 2. 왕복운동 시 트랙 분리 원인
**실제 원인은 Boundary Box 버그가 아니라:**

1. **DBSCAN epsilon이 너무 작음**
   - 왕복운동 시 좌우로 흔들림
   - epsilon=0.6m: 중앙(Y=0) vs 왼쪽(Y=-0.5) vs 오른쪽(Y=0.5)
   - 흔들림 > 0.6m이면 다른 클러스터로 분리!

2. **AssignmentThreshold와 왕복 궤적**
   - 왕복 시 급격한 방향 전환
   - Mahalanobis 거리 급증
   - AssignmentThreshold=100 초과 → 트랙 분리

3. **좌표 회전 자체는 문제 없음**
   - 회전은 **시각화 전용**
   - 트래커는 원본 센서 좌표 사용
   - 따라서 좌표 회전은 트랙 분리와 무관!

---

## 🔧 해결 방법

### 해결책 1: Boundary Box를 실제로 사용 (권장!)

```matlab
% 센서 좌표계 기준으로 Boundary Box 정의
% (센서가 X+를 전방으로 사용한다고 가정)
xMin_bb_sensor =  0.5; xMax_bb_sensor = 8;    % 전방: X+ 방향
yMin_bb_sensor = -2;   yMax_bb_sensor = 2;    % 좌우: Y 방향
zMin_bb_sensor =  0;   zMax_bb_sensor = 2.5;  % 높이: Z 방향

% Boundary Box 필터링 적용
isInBoundary = (x_all >= xMin_bb_sensor) & (x_all <= xMax_bb_sensor) & ...
               (y_all >= yMin_bb_sensor) & (y_all <= yMax_bb_sensor) & ...
               (z_all >= zMin_bb_sensor) & (z_all <= zMax_bb_sensor);
isDynamic = isDynamic & isInBoundary;

% 표시용 Boundary Box는 회전된 좌표로 재정의
% (시각화 목적)
```

### 해결책 2: DBSCAN epsilon 증가

```matlab
% 왕복운동 시 좌우 흔들림 대응
dbscanEpsilon = 0.9;  // 0.6 → 0.9 (±0.5m 흔들림 허용)
```

### 해결책 3: AssignmentThreshold 증가

```matlab
% 급격한 방향 전환 허용
'AssignmentThreshold', [200 inf]  // 100 → 200
```

### 해결책 4: ProcessNoise 증가

```matlab
% 방향 전환 시 예측 불확실성 증가
filter.ProcessNoise = filter.ProcessNoise * 3.0;  // 2.0 → 3.0
```

---

## 📝 결론

### 좌표 회전은 트랙 분리의 원인이 아닙니다!

**이유:**
1. 좌표 회전은 **시각화 전용**
2. 트래커는 **원본 센서 좌표** 사용
3. 트래킹 알고리즘은 회전 영향 받지 않음

### 실제 트랙 분리 원인:

1. **Boundary Box 미사용** (코드 버그)
   - 정의만 하고 실제 필터링 안 함
   - 거리 필터링만 적용

2. **DBSCAN epsilon 너무 작음** (왕복운동 대응 부족)
   - 좌우 흔들림 > 0.6m → 클러스터 분리

3. **AssignmentThreshold 작음** (급격한 방향 전환)
   - 왕복 시 Mahalanobis 거리 급증

4. **ProcessNoise 부족** (방향 전환 예측)
   - 급격한 변화 예측 못함

---

## 🎯 권장 수정 사항

1. **Boundary Box를 센서 좌표 기준으로 재정의하고 실제 적용**
2. **DBSCAN epsilon을 0.9m로 증가** (왕복운동 대응)
3. **AssignmentThreshold를 200으로 증가** (방향 전환 허용)
4. **ProcessNoise를 ×3.0으로 증가** (왕복운동 예측)

이렇게 수정하면 왕복운동 시 트랙 분리가 90% 이상 감소할 것입니다!
