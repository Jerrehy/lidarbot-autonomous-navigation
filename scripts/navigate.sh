#!/bin/bash
# Скрипт для навигации по карте (Уровень 4)

MAP_FILE=$1

if [ -z "$MAP_FILE" ]; then
    echo "Использование: $0 <путь_к_карте.yaml>"
    exit 1
fi

echo "=== Запуск навигации с картой: $MAP_FILE ==="

# Запуск системы с картой
ros2 launch nav2_bringup bringup_launch.py \
    map:=$MAP_FILE \
    params_file:=$(ros2 pkg prefix robot_navigation)/share/robot_navigation/config/nav2_params.yaml \
    use_sim_time:=false

echo "=== Навигация запущена ==="
echo "Используйте RViz для установки начальной позиции и цели"
