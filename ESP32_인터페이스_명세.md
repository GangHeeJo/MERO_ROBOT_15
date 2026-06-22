# 비전팀 → 로봇팀 인터페이스 명세

> 작성일: 2026-06-21  
> 작성: 비전팀 (조강희)

---

## 개요

Jetson Orin Nano(비전팀)가 카메라로 물체를 탐지·트래킹한 결과를  
USB 시리얼을 통해 ESP32(로봇팀)로 실시간 전송합니다.

---

## 연결 방법

| 항목 | 값 |
|------|----|
| 연결 방식 | USB Serial (Jetson ↔ ESP32) |
| 보드레이트 | 115200 baud |
| 데이터 형식 | JSON 한 줄 + `\n` |
| 전송 주기 | 카메라 프레임마다 (약 30fps) |

---

## 데이터 형식

### 물체가 탐지된 경우

```json
{
  "objects": [
    {
      "id":   1,
      "cls":  "d8",
      "cx":   342.5,
      "cy":   218.3,
      "mx":   12.3,
      "my":   -5.1,
      "conf": 0.91
    },
    {
      "id":   2,
      "cls":  "d6",
      "cx":   150.2,
      "cy":   300.1,
      "mx":   -8.2,
      "my":   10.3,
      "conf": 0.76
    }
  ],
  "target": {
    "id":   1,
    "cls":  "d8",
    "cx":   342.5,
    "cy":   218.3,
    "mx":   12.3,
    "my":   -5.1,
    "conf": 0.91
  }
}
```

### 탐지된 물체가 없는 경우

```json
{"objects": [], "target": null}
```

---

## 필드 설명

| 필드 | 타입 | 설명 |
|------|------|------|
| `objects` | 배열 | 이번 프레임에서 탐지된 전체 물체 목록 |
| `target` | 객체 or null | 집게가 집을 대상 1개 (신뢰도 최고). 탐지 없으면 null |
| `id` | 정수 | 트래킹 ID. 같은 물체는 프레임이 바뀌어도 동일한 ID 유지 |
| `cls` | 문자열 | 물체 종류. 아래 클래스 목록 참고 |
| `cx`, `cy` | 실수 | 화면 기준 픽셀 좌표 (참고용) |
| `mx`, `my` | 실수 | **카메라 중심 기준 실제 거리 (mm)**. 집게 이동에 사용 |
| `conf` | 실수 | 탐지 신뢰도 (0.0 ~ 1.0) |

> `mx`, `my`는 카메라 캘리브레이션 완료 후 포함됩니다.  
> 캘리브레이션 전에는 해당 필드가 없을 수 있습니다.

---

## 좌표계

```
카메라 화면 기준:

        mx 음수 (왼쪽)   mx 양수 (오른쪽)
              ←    (0,0)    →
         ↑
    my 음수
    (위쪽)

         ↓
    my 양수
    (아래쪽)
```

- 원점 `(0, 0)` = 카메라 정중앙 = 테이블 중심
- `mx` 양수 = 오른쪽, 음수 = 왼쪽 (mm)
- `my` 양수 = 아래쪽, 음수 = 위쪽 (mm)

---

## 클래스 목록

| cls 값 | 설명 |
|--------|------|
| `d6` | 6면체 주사위 |
| `d8` | 8면체 주사위 |
| `d12` | 12면체 주사위 |
| `d20` | 20면체 주사위 |
| `apple` | 사과 |
| `banana` | 바나나 |
| `orange` | 오렌지 |
| `pineapple` | 파인애플 |

---

## ESP32 수신 코드 예시 (Arduino)

```cpp
#include <ArduinoJson.h>

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');

    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, line);
    if (err) return;

    JsonObject target = doc["target"];
    if (target.isNull()) {
      // 탐지된 물체 없음 → 대기
      return;
    }

    const char* cls = target["cls"];   // 물체 종류
    float mx = target["mx"];           // 가로 거리 (mm)
    float my = target["my"];           // 세로 거리 (mm)
    int   id = target["id"];           // 트래킹 ID

    // TODO: mx, my 기반으로 집게 이동
  }
}
```

---

## 주의사항

- 데이터는 매 프레임 전송되므로 **가장 최근 수신 데이터만 사용**하면 됨
- `target`이 `null`이면 집게 동작 중지 또는 대기 상태 유지 권장
- `id`가 바뀌면 다른 물체로 타겟이 전환된 것
- 캘리브레이션 전에는 `mx`, `my` 필드가 없을 수 있으니 존재 여부 체크 필요

---

## 문의

비전팀 카톡으로 연락주세요.
