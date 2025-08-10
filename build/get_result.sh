#!/bin/bash
clear
# 현재 디렉토리 경로 가져오기
current_dir=$(pwd)

echo "코드 컴파일 시작"
make

echo "코드 컴파일 완료"

rm $current_dir/ekf_result.csv
echo "ekf_result.csv 파일 삭제"

echo "ekf 시뮬레이터 실행"
./ekf_localizer
echo "ekf_result.csv 파일 생성"

echo "시각화 시작"
python3 $current_dir/result_visualization.py
