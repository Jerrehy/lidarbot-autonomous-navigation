#!/bin/bash
# Скрипт для построения карты (Уровень 4)

echo "=== Запуск системы для построения карты ==="

# Запуск основной системы
ros2 launch robot_navigation level4_full_nav_launch.py use_rviz:=true &
MAIN_PID=$!

# Ожидание запуска
sleep 5

echo "=== Запуск SLAM ==="
# SLAM уже запущен в launch-файле

echo "=== Откройте RViz и управляйте роботом ==="
echo "Используйте джойстик или клавиатуру для движения"
echo "Для телеуправления клавиатурой:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"

# Ожидание завершения
wait $MAIN_PID
