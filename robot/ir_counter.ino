/*
 * ir_counter.ino — KY-032 적외선 장애물 센서로 물체 통과 개수 세기
 * ──────────────────────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 *
 * test_ir_counter.ino(독립 테스트 스케치)에서 검증한 로직을 메인 펌웨어에
 * 통합한 것 — 그리퍼/팔/컨테이너 상태 머신과 완전히 독립적으로 동작한다.
 * safety.ino의 safetyPoll()처럼 loop()에서 매 프레임 irPoll()을 호출하는
 * 폴링 방식. 카운트가 올라갈 때마다 Jetson에 바로 알려준다(요청-응답이
 * 아니라 변화가 생길 때 알아서 보내는 push 방식 — 다른 status들과 형식은
 * 같지만 특정 cmd에 대한 응답이 아니라 언제든 올 수 있음에 주의).
 *
 * KY-032는 VCC/GND/OUT 3핀. OUT은 물체 감지 시 LOW로 떨어지는 모듈이
 * 흔하지만(IR_ACTIVE_LOW=true), 반대인 모듈도 있어 실물 확인 필요.
 */

#define IR_SENSOR_PIN          2     // ⚠️ 실제 연결한 디지털 핀 번호로 바꿀 것
#define IR_DEBOUNCE_MS         50    // ⚠️ 실측 필요 — 신호 흔들림(bounce) 흡수용
#define IR_ACTIVE_LOW          true  // ⚠️ 실물 확인 필요 — OUT이 감지 시 LOW면 true, HIGH면 false
#define IR_RECOUNT_COOLDOWN_MS 2000  // ⚠️ 실측 필요 — 물체 하나가 지나가는 데 걸리는 시간보다
                                     // 조금 길게. 이 시간 안에 다시 감지돼도 같은 물체로 보고 무시

bool     irObjectPresent = false;
uint32_t irLastChangeMs  = 0;
uint32_t irObjectCount   = 0;
bool     irHasCounted    = false;  // 아직 한 번도 카운트 안 했으면 쿨다운 없이 바로 셈
uint32_t irLastCountMs   = 0;

static bool irReadRaw() {
  int v = digitalRead(IR_SENSOR_PIN);
  return IR_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

void irSetup() {
  pinMode(IR_SENSOR_PIN, INPUT);
  Serial.println("[IR카운터] 준비 완료");
}

// 매 loop()마다 호출 — 새로 카운트됐으면 Jetson에 {"status":"ir_count","count":N} 전송
void irPoll() {
  bool raw = irReadRaw();
  uint32_t now = millis();

  if (raw != irObjectPresent && (now - irLastChangeMs) >= IR_DEBOUNCE_MS) {
    irObjectPresent = raw;
    irLastChangeMs  = now;

    if (irObjectPresent) {
      if (!irHasCounted || (now - irLastCountMs) >= IR_RECOUNT_COOLDOWN_MS) {
        irObjectCount++;
        irLastCountMs = now;
        irHasCounted  = true;
        JETSON_SERIAL.print("{\"status\":\"ir_count\",\"count\":");
        JETSON_SERIAL.print(irObjectCount);
        JETSON_SERIAL.println("}");
      }
      // 쿨다운 중 재감지는 같은 물체로 보고 조용히 무시 (Jetson에 안 보냄)
    }
  }
}
