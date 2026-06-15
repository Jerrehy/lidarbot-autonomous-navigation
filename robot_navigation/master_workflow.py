#!/usr/bin/env python3
"""
Мастер-скрипт для дипломной работы.
Последовательно выполняет: 1) Построение карты -> 2) Сохранение -> 3) Навигацию.
"""
import subprocess
import time
import os
import signal
import sys

class MasterWorkflow:
    def __init__(self):
        self.map_name = "diploma_map"
        self.map_dir = "/home/berry/maps"
        self.active_processes = []  # Только текущие активные процессы

    def run_command(self, cmd, description):
        print(f"\n{'='*60}")
        print(f"🚀 ЭТАП: {description}")
        print(f"Команда: {cmd}")
        print(f"{'='*60}")
        proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
        self.active_processes.append(proc)
        return proc

    def stop_all(self):
        print("\n🛑 Остановка всех фоновых процессов ROS...")
        for proc in self.active_processes:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
        subprocess.run("pkill -f 'ros2'", shell=True, stdout=subprocess.DEVNULL)
        time.sleep(2)
        # Очищаем список, так как все процессы убиты
        self.active_processes = []

    def start(self):
        print("🤖 ЗАПУСК МАСТЕР-ПРОЦЕССА ДЛЯ DIPLOMA LIDARBOT")
        
        # ШАГ 1: Запуск железа и SLAM
        self.run_command(
            "ros2 launch robot_navigation basic_robot_launch.py",
            "1. Запуск драйверов (моторы, лидар, IMU)"
        )
        time.sleep(3)
        
        self.run_command(
            "ros2 launch robot_navigation slam_launch.py",
            "2. Запуск SLAM (построение карты)"
        )
        
        print("\n💡 ИНСТРУКЦИЯ:")
        print("1. Откройте RViz2 в новом терминале: rviz2")
        print("2. Установите Fixed Frame: map")
        print("3. Добавьте Display: Map (topic: /map) и LaserScan (topic: /scan)")
        print("4. Управляйте роботом (teleop или remote_commander), чтобы построить карту.")
        input("\n⏳ Нажмите ENTER, когда карта построена и вы готовы сохранить её...")

        # ШАГ 2: Сохранение карты
        print("\n💾 Сохранение карты...")
        os.makedirs(self.map_dir, exist_ok=True)
        map_path = os.path.join(self.map_dir, self.map_name)
        subprocess.run(f"ros2 run nav2_map_server map_saver_cli -f {map_path}", shell=True)
        
        if os.path.exists(f"{map_path}.yaml"):
            print(f"✅ Карта успешно сохранена: {map_path}.yaml")
        else:
            print("❌ Ошибка сохранения карты!")
            self.stop_all()
            sys.exit(1)

        # ШАГ 3: Перезапуск в режим навигации
        print("\n🔄 Переключение в режим НАВИГАЦИИ...")
        self.stop_all()  # Убивает все процессы и очищает active_processes
        time.sleep(2)

        # Запускаем новые процессы для навигации
        self.run_command(
            "ros2 launch robot_navigation basic_robot_launch.py",
            "1. Повторный запуск драйверов"
        )
        time.sleep(2)

        self.run_command(
            f"ros2 launch robot_navigation full_system_launch.py map:={map_path}.yaml",
            f"2. Запуск Nav2 с картой: {self.map_name}"
        )
        
        print("\n" + "="*60)
        print("🎉 ГОТОВО! Система перешла в режим навигации.")
        print("💡 Теперь вы можете:")
        print("   - Дать цель через RViz2 (2D Goal Pose)")
        print("   - Запустить миссию: ros2 run robot_navigation mission_planner ...")
        print("   - Открыть веб-монитор: http://localhost:8080")
        print("="*60)
        print("Для завершения работы нажмите Ctrl+C")

        try:
            # Держим скрипт активным, пока работают фоновые процессы
            while True:
                time.sleep(1)
                # Проверяем только последние два процесса (драйверы + навигация)
                # Если хотя бы один из них жив — всё ок
                if len(self.active_processes) >= 2:
                    # Проверяем только последний запущенный процесс (навигацию)
                    if self.active_processes[-1].poll() is not None:
                        print("\n⚠️ Процесс навигации завершился.")
                        break
                else:
                    print("\n⚠️ Недостаточно активных процессов.")
                    break
        except KeyboardInterrupt:
            print("\n🛑 Завершение работы по запросу пользователя.")
        finally:
            self.stop_all()

if __name__ == '__main__':
    workflow = MasterWorkflow()
    workflow.start()
