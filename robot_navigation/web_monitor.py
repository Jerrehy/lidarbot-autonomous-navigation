#!/usr/bin/env python3
"""
Веб-интерфейс мониторинга LidarBot
Запуск: ros2 run robot_navigation web_monitor
Открыть в браузере: http://<IP_РОБОТА>:8080
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
import json
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import math


class RobotState:
    """Хранилище актуального состояния робота"""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.ranges = []
        self.cpu = 0.0
        self.ram = 0.0
        self.voltage = 12.0

state = RobotState()

HTML_PAGE = """<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>LidarBot Web Monitor</title>
<style>
    body { font-family: system-ui, sans-serif; background: #0f172a; color: #e2e8f0; margin: 0; padding: 20px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 15px; max-width: 1200px; margin: auto; }
    .card { background: #1e293b; padding: 20px; border-radius: 12px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
    h2 { margin: 0 0 15px; color: #38bdf8; font-size: 1.2rem; }
    .row { display: flex; justify-content: space-between; margin: 8px 0; }
    .val { font-weight: bold; color: #f472b6; }
    canvas { background: #000; border-radius: 8px; width: 100%; height: 250px; display: block; }
    .bar { height: 12px; background: #334155; border-radius: 6px; overflow: hidden; margin-top: 5px; }
    .fill { height: 100%; background: linear-gradient(90deg, #22d3ee, #3b82f6); transition: width 0.5s; }
</style>
</head>
<body>
<h1 style="text-align:center;">🤖 LidarBot Real-time Monitor</h1>
<div class="grid">
    <div class="card"><h2>📍 Позиция</h2>
        <div class="row"><span>X:</span><span class="val" id="x">0.00</span></div>
        <div class="row"><span>Y:</span><span class="val" id="y">0.00</span></div>
        <div class="row"><span>Yaw:</span><span class="val" id="yaw">0.0°</span></div>
    </div>
    <div class="card"><h2>🔋 Система</h2>
        <div class="row"><span>Батарея:</span><span class="val" id="voltage">12.0 В</span></div>
        <div class="row"><span>CPU: <span id="cpu_val">0</span>%</span></div>
        <div class="bar"><div class="fill" id="cpu_bar" style="width:0%"></div></div>
        <div class="row"><span>RAM: <span id="ram_val">0</span>%</span></div>
        <div class="bar"><div class="fill" id="ram_bar" style="width:0%"></div></div>
    </div>
    <div class="card"><h2>📡 Лидар (Live)</h2><canvas id="lidar" width="300" height="250"></canvas></div>
</div>
<script>
function update() {
    fetch('/api/state')
    .then(r => r.json())
    .then(d => {
        document.getElementById('x').textContent = d.x.toFixed(2) + ' м';
        document.getElementById('y').textContent = d.y.toFixed(2) + ' м';
        document.getElementById('yaw').textContent = d.yaw.toFixed(1) + '°';
        document.getElementById('voltage').textContent = d.voltage.toFixed(1) + ' В';
        document.getElementById('cpu_val').textContent = d.cpu;
        document.getElementById('cpu_bar').style.width = d.cpu + '%';
        document.getElementById('ram_val').textContent = d.ram;
        document.getElementById('ram_bar').style.width = d.ram + '%';
        
        // Отрисовка лидара
        const c = document.getElementById('lidar'), ctx = c.getContext('2d');
        ctx.fillStyle = '#000'; ctx.fillRect(0,0,c.width,c.height);
        ctx.fillStyle = '#22d3ee';
        const cx = c.width/2, cy = c.height, scale = Math.min(cx, cy)/4.0;
        d.lidar_ranges.forEach((r, i) => {
            if (r > 0 && r < 4.0) {
                const angle = (i / d.lidar_ranges.length) * Math.PI;
                const px = cx + r * scale * Math.cos(angle);
                const py = cy - r * scale * Math.sin(angle);
                ctx.fillRect(px, py, 2, 2);
            }
        });
        ctx.fillStyle = '#f472b6'; ctx.beginPath(); ctx.arc(cx, cy, 4, 0, Math.PI*2); ctx.fill();
    });
}
setInterval(update, 400); // 2.5 Hz обновление
</script>
</body>
</html>"""

class MonitorHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ['/', '/index.html']:
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode())
        elif self.path == '/api/state':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({
                'x': state.x, 'y': state.y, 'yaw': state.yaw,
                'cpu': state.cpu, 'ram': state.ram, 'voltage': state.voltage,
                'lidar_ranges': state.ranges
            }).encode())
        else:
            self.send_error(404)
    def log_message(self, format, *args): pass  # Тихий режим

class WebMonitor(Node):
    def __init__(self):
        super().__init__('web_monitor')
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.diag_cb, 10)
        
        self.server = HTTPServer(('0.0.0.0', 8080), MonitorHandler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info('🌐 Веб-монитор запущен: http://<IP_РОБОТА>:8080')

    def odom_cb(self, msg):
        state.x = msg.pose.pose.position.x
        state.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        state.yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def scan_cb(self, msg):
        state.ranges = [r if 0 < r < 8.0 else 0.0 for r in msg.ranges[::2]]

    def diag_cb(self, msg):
        for s in msg.status:
            if 'cpu' in s.name:
                for kv in s.values:
                    if 'cpu_percent' in kv.key: state.cpu = float(kv.value.replace('%',''))
                    if 'ram_percent' in kv.key: state.ram = float(kv.value.replace('%',''))
            if 'battery' in s.name:
                for kv in s.values:
                    if 'voltage' in kv.key: state.voltage = float(kv.value.replace('V',''))

    def destroy_node(self):
        self.server.shutdown()
        super().destroy_node()

def main():
    rclpy.init()
    node = WebMonitor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
