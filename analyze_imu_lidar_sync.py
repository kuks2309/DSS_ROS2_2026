#!/usr/bin/env python3
"""
IMU-LiDAR 동기화 분석 스크립트
rosbag2 데이터를 분석하여 IMU/LiDAR 타이밍 문제를 진단합니다.

사용법:
    python3 analyze_imu_lidar_sync.py <rosbag_directory>

예시:
    ros2 bag record /dss/sensor/imu /dss/sensor/lidar -o imu_lidar_test
    python3 analyze_imu_lidar_sync.py imu_lidar_test
"""

import sys
import sqlite3
import struct
from pathlib import Path
from collections import defaultdict

def read_rosbag2_sqlite(bag_path: str):
    """rosbag2 SQLite3 파일에서 메시지 읽기"""

    bag_dir = Path(bag_path)

    # metadata.yaml에서 DB 파일 찾기
    db_files = list(bag_dir.glob("*.db3"))
    if not db_files:
        print(f"ERROR: {bag_path}에서 .db3 파일을 찾을 수 없습니다.")
        return None, None

    db_path = db_files[0]
    print(f"분석 중: {db_path}")

    conn = sqlite3.connect(str(db_path))
    cursor = conn.cursor()

    # 토픽 ID 조회
    cursor.execute("SELECT id, name FROM topics")
    topics = {row[1]: row[0] for row in cursor.fetchall()}
    print(f"발견된 토픽: {list(topics.keys())}")

    imu_times = []
    lidar_times = []

    # IMU 메시지 타임스탬프 추출
    imu_topic = None
    for topic_name in topics.keys():
        if 'imu' in topic_name.lower():
            imu_topic = topic_name
            break

    lidar_topic = None
    for topic_name in topics.keys():
        if 'lidar' in topic_name.lower() or 'point' in topic_name.lower():
            lidar_topic = topic_name
            break

    if imu_topic and imu_topic in topics:
        cursor.execute(
            "SELECT timestamp FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topics[imu_topic],)
        )
        imu_times = [row[0] for row in cursor.fetchall()]
        print(f"IMU 메시지 수: {len(imu_times)} (토픽: {imu_topic})")

    if lidar_topic and lidar_topic in topics:
        cursor.execute(
            "SELECT timestamp FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topics[lidar_topic],)
        )
        lidar_times = [row[0] for row in cursor.fetchall()]
        print(f"LiDAR 메시지 수: {len(lidar_times)} (토픽: {lidar_topic})")

    conn.close()
    return imu_times, lidar_times


def analyze_timing(imu_times: list, lidar_times: list):
    """타이밍 분석 수행"""

    if not imu_times or not lidar_times:
        print("\nERROR: IMU 또는 LiDAR 데이터가 없습니다.")
        return

    print("\n" + "="*60)
    print("타이밍 분석 결과")
    print("="*60)

    # 나노초 -> 초 변환
    imu_sec = [t / 1e9 for t in imu_times]
    lidar_sec = [t / 1e9 for t in lidar_times]

    # 1. 기본 통계
    print("\n[1] 기본 통계")
    print(f"  IMU 메시지 수: {len(imu_times)}")
    print(f"  LiDAR 메시지 수: {len(lidar_times)}")

    imu_duration = imu_sec[-1] - imu_sec[0]
    lidar_duration = lidar_sec[-1] - lidar_sec[0]

    print(f"  IMU 녹화 시간: {imu_duration:.2f}초")
    print(f"  LiDAR 녹화 시간: {lidar_duration:.2f}초")

    if imu_duration > 0:
        imu_hz = len(imu_times) / imu_duration
        print(f"  IMU 평균 Hz: {imu_hz:.2f}")

    if lidar_duration > 0:
        lidar_hz = len(lidar_times) / lidar_duration
        print(f"  LiDAR 평균 Hz: {lidar_hz:.2f}")

    # 2. IMU 간격 분석
    print("\n[2] IMU 메시지 간격 분석")
    imu_intervals = [imu_sec[i+1] - imu_sec[i] for i in range(len(imu_sec)-1)]
    if imu_intervals:
        avg_interval = sum(imu_intervals) / len(imu_intervals)
        max_interval = max(imu_intervals)
        min_interval = min(imu_intervals)

        # 이상치 (평균의 3배 이상)
        anomalies = [(i, interval) for i, interval in enumerate(imu_intervals) if interval > avg_interval * 3]

        print(f"  평균 간격: {avg_interval*1000:.2f}ms")
        print(f"  최소 간격: {min_interval*1000:.2f}ms")
        print(f"  최대 간격: {max_interval*1000:.2f}ms")
        print(f"  이상치 (3x 평균): {len(anomalies)}개")

        if anomalies and len(anomalies) <= 10:
            print("  이상치 상세:")
            for idx, interval in anomalies:
                print(f"    - 인덱스 {idx}: {interval*1000:.2f}ms 간격")

    # 3. LiDAR 간격 분석
    print("\n[3] LiDAR 메시지 간격 분석")
    lidar_intervals = [lidar_sec[i+1] - lidar_sec[i] for i in range(len(lidar_sec)-1)]
    if lidar_intervals:
        avg_interval = sum(lidar_intervals) / len(lidar_intervals)
        max_interval = max(lidar_intervals)
        min_interval = min(lidar_intervals)

        print(f"  평균 간격: {avg_interval*1000:.2f}ms")
        print(f"  최소 간격: {min_interval*1000:.2f}ms")
        print(f"  최대 간격: {max_interval*1000:.2f}ms")

    # 4. 시작 시간 차이 분석 (핵심!)
    print("\n[4] IMU-LiDAR 시작 시간 분석 (핵심)")
    first_imu = imu_sec[0]
    first_lidar = lidar_sec[0]
    start_diff = first_imu - first_lidar

    print(f"  첫 IMU 시간: {first_imu:.6f}초")
    print(f"  첫 LiDAR 시간: {first_lidar:.6f}초")
    print(f"  시작 시간 차이: {start_diff*1000:.2f}ms")

    if start_diff > 0:
        print(f"  >> IMU가 LiDAR보다 {start_diff*1000:.2f}ms 늦게 시작됨!")
        print(f"  >> 이것이 'IMU queue is empty' 원인일 수 있음")
    else:
        print(f"  >> IMU가 LiDAR보다 {-start_diff*1000:.2f}ms 먼저 시작됨 (정상)")

    # 5. 각 LiDAR 프레임 시점의 IMU 가용성
    print("\n[5] LiDAR 프레임별 IMU 데이터 가용성")

    empty_count = 0
    insufficient_count = 0
    ok_count = 0

    for i, lidar_t in enumerate(lidar_sec[:20]):  # 처음 20 프레임만
        # 해당 LiDAR 시간 이전의 IMU 데이터 수
        imu_before = sum(1 for t in imu_sec if t < lidar_t)
        # 해당 LiDAR 시간 근처 (±100ms)의 IMU 데이터 수
        imu_near = sum(1 for t in imu_sec if abs(t - lidar_t) < 0.1)

        status = "OK"
        if imu_before == 0:
            status = "EMPTY!"
            empty_count += 1
        elif imu_before < 5:
            status = "부족"
            insufficient_count += 1
        else:
            ok_count += 1

        if i < 10 or status != "OK":
            print(f"  프레임 {i}: LiDAR={lidar_t:.3f}s, "
                  f"이전 IMU={imu_before}개, 근처 IMU={imu_near}개 [{status}]")

    print(f"\n  요약 (처음 20프레임):")
    print(f"    EMPTY: {empty_count}개")
    print(f"    부족: {insufficient_count}개")
    print(f"    OK: {ok_count}개")

    # 6. 결론 및 권장사항
    print("\n" + "="*60)
    print("진단 결론")
    print("="*60)

    issues = []

    if start_diff > 0.01:  # IMU가 10ms 이상 늦게 시작
        issues.append(f"IMU가 LiDAR보다 {start_diff*1000:.0f}ms 늦게 시작됨")

    if imu_intervals and max(imu_intervals) > 0.1:  # 100ms 이상 간격
        issues.append(f"IMU 메시지 간격 이상치 존재 (최대 {max(imu_intervals)*1000:.0f}ms)")

    if empty_count > 0:
        issues.append(f"처음 20프레임 중 {empty_count}개에서 IMU 데이터 없음")

    if issues:
        print("\n발견된 문제:")
        for i, issue in enumerate(issues, 1):
            print(f"  {i}. {issue}")

        print("\n권장 해결책:")
        if start_diff > 0:
            print("  - DSS 브릿지에서 IMU 노드를 먼저 시작")
            print("  - 또는 LIO-SAM에서 더 긴 warmup 기간 설정")
        if empty_count > 0:
            print("  - imageProjection.cpp의 warmup_frames 값 증가")
            print("  - IMU 구독이 LiDAR보다 먼저 활성화되도록 launch 파일 수정")
    else:
        print("\n타이밍 문제가 발견되지 않았습니다.")
        print("다른 원인 조사 필요:")
        print("  - 콜백 처리 지연")
        print("  - 메시지 큐 크기")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        print("\n실시간 분석 (rosbag 없이):")
        print("  ros2 topic hz /dss/sensor/imu")
        print("  ros2 topic hz /dss/sensor/lidar")
        print("  ros2 topic delay /dss/sensor/imu")
        return 1

    bag_path = sys.argv[1]

    if not Path(bag_path).exists():
        print(f"ERROR: {bag_path} 경로가 존재하지 않습니다.")
        return 1

    imu_times, lidar_times = read_rosbag2_sqlite(bag_path)

    if imu_times is not None and lidar_times is not None:
        analyze_timing(imu_times, lidar_times)

    return 0


if __name__ == "__main__":
    sys.exit(main())
