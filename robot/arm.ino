/*
 * arm.ino — 팔 관절 제어 (XL430 × 6)
 * ──────────────────────────────────────
 * TODO: 팔 구성 확정 후 작성
 *
 * 확정 필요 사항:
 *   1. 각 관절 역할 정의 (예시)
 *      ID 1: 베이스 회전 (좌우)
 *      ID 2: 어깨 (상하)
 *      ID 3: 팔꿈치 (상하)
 *      ID 4: 손목 (상하)
 *      ID 5: 손목 회전
 *      ID 6: 집게 베이스
 *   2. 각 관절 동작 범위 (각도)
 *   3. 집기 동작 시퀀스 (어떤 순서로 관절이 움직이는지)
 *   4. 드롭존별 팔 자세 (cls에 따라 다른 자세 필요 여부)
 *
 * 공통 사항 (gripper.ino 참고):
 *   - 프로토콜: Protocol 2.0
 *   - 통신: DXL_SERIAL (Serial2)
 *   - 제어 모드: OP_POSITION (위치 제어)
 *   - 레벨시프터 불필요 (XL430 3.3V TTL)
 */

// TODO: 팔 구성 확정 후 아래 내용 채우기

void armSetup() {
  // TODO
  Serial.println("[팔] arm.ino 미구현 — 팔 구성 확정 후 작성");
}

// 물체 집기 위치로 팔 이동
void armMoveToTarget(float mx, float my) {
  // TODO: mx, my → 각 관절 각도 계산 (역기구학)
}

// 드롭존 위치로 팔 이동
void armMoveToDrop(const char* cls) {
  // TODO: cls별 팔 자세 정의
}

// 홈 포지션 (초기 자세)
void armHome() {
  // TODO: 모든 관절 초기 위치로
}
