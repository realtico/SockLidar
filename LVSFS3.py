# Versão fullscreen (SFS) baseada em LidarViewerSocket
# Suporta argumentos de linha de comando: -f (fullscreen), -w XXX, -p /dev/ttyUSB0

import os
import time
import pygame
import math
import socket
import subprocess
import csv
import sys
import argparse
from math import pi, sin, cos

SOCKET_PATH = "/tmp/lidar_socket"
LIDAR_READER_EXEC = "./lidar_reader_socket"

GRID_FONT_SIZE = 10
UI_FONT_SIZE = 16

class LidarViewerSFS:
    def __init__(self, fullscreen=True, window_size=(800, 800), serial_port="/dev/ttyUSB0"):
        pygame.init()
        info = pygame.display.Info()
        if fullscreen:
            self.display_size = (info.current_w, info.current_h)
            self.screen = pygame.display.set_mode(self.display_size, pygame.FULLSCREEN)
        else:
            self.display_size = window_size
            self.screen = pygame.display.set_mode(self.display_size)

        raw_ws = min(self.display_size)
        aligned_ws = raw_ws - (raw_ws % 16)
        self.ws = aligned_ws
        self.hws = self.ws // 2
        self.center = (
            (self.display_size[0] - self.ws) // 2 + self.hws,
            (self.display_size[1] - self.ws) // 2 + self.hws
        )
        self.grid_rect = pygame.Rect(
            self.center[0] - self.hws,
            self.center[1] - self.hws,
            self.ws,
            self.ws
        )

        self.scale = 8000 / self.hws
        self.max_scale = 8000 / self.hws
        self.dot_size = 3
        self.gridstyle = 0
        self.running = True
        self.show_overlay = True

        self.set_dark_mode()

        pygame.display.set_caption("Lidar Viewer SFS")
        pygame.font.init()
        self.grid_font = pygame.font.SysFont('Calibri', GRID_FONT_SIZE)
        self.ui_font = pygame.font.SysFont('Calibri', UI_FONT_SIZE)
        self.clock = pygame.time.Clock()

        self.grid_surface = self.create_grid()
        self.datapack = []
        self.last_delay_ms = -1

        self.reader_proc = subprocess.Popen([LIDAR_READER_EXEC, serial_port])
        try:
            os.sched_setaffinity(self.reader_proc.pid, {1})
        except (AttributeError, OSError):
            pass

        self.total_frames = 0
        self.skipped_frames = 0

    def set_dark_mode(self):
        self.backcolor = (0, 0, 0)
        self.gridcolor = (80, 80, 255)
        self.lessergridcolor = (0, 0, 96)
        self.gridcentercolor = (255, 255, 255)

    def set_light_mode(self):
        self.backcolor = (255, 255, 255)
        self.gridcolor = (0, 0, 128)
        self.lessergridcolor = (192, 192, 255)
        self.gridcentercolor = (0, 0, 0)


    def create_grid(self):
        surface = pygame.Surface(self.display_size).convert()
        surface.fill(self.backcolor)
        major = self.hws // 8
        minor = max(1, major // 5)

        if self.gridstyle == 0:
            for i in range(8):
                pygame.draw.circle(surface, self.gridcolor, self.center, (i + 1) * major, 1)
                for j in range(minor, major, minor):
                    pygame.draw.circle(surface, self.lessergridcolor, self.center, (i + 1) * major - j, 1)
            for i in range(24):
                angle = i * pi / 12
                x = self.center[0] + self.hws * cos(angle)
                y = self.center[1] - self.hws * sin(angle)
                color = self.gridcolor if i % 3 == 0 else self.lessergridcolor
                pygame.draw.line(surface, color, self.center, (int(x), int(y)), 1)
        else:
            for i in range(self.grid_rect.left, self.grid_rect.right, minor):
                pygame.draw.line(surface, self.lessergridcolor, (i, self.grid_rect.top), (i, self.grid_rect.bottom))
            for i in range(self.grid_rect.top, self.grid_rect.bottom, minor):
                pygame.draw.line(surface, self.lessergridcolor, (self.grid_rect.left, i), (self.grid_rect.right, i))
            for i in range(self.grid_rect.left, self.grid_rect.right, major):
                pygame.draw.line(surface, self.gridcolor, (i, self.grid_rect.top), (i, self.grid_rect.bottom))
            for i in range(self.grid_rect.top, self.grid_rect.bottom, major):
                pygame.draw.line(surface, self.gridcolor, (self.grid_rect.left, i), (self.grid_rect.right, i))

        pygame.draw.line(surface, self.gridcentercolor, (self.grid_rect.left, self.center[1]), (self.grid_rect.right, self.center[1]))
        pygame.draw.line(surface, self.gridcentercolor, (self.center[0], self.grid_rect.top), (self.center[0], self.grid_rect.bottom))

        for i in range(8):
            dist = (i + 1) * 1000 * (self.scale / self.max_scale)
            txt = self.grid_font.render(f"{int(dist)} mm", False, self.gridcolor)
            surface.blit(txt, (self.center[0] + 10, self.center[1] - ((i + 1) * major)))
            surface.blit(txt, (self.center[0] + 10, self.center[1] + ((i + 1) * major)))

        return surface

    def polar_to_point(self, polar):
        angle, dist = polar
        ang = angle * pi / 180.0
        dist /= self.scale
        return (self.center[0] - dist * sin(ang), self.center[1] + dist * cos(ang))

    def read_socket(self):
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
                s.connect(SOCKET_PATH)
                s.sendall(b"get")
                data = b""
                while True:
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    data += chunk
        except Exception:
            return

        lines = data.decode().splitlines()
        current_frame = []
        frame_timestamp = None

        for line in lines:
            if line.startswith("NEWFRAME"):
                try:
                    _, timestamp = line.split()
                    frame_timestamp = int(timestamp)
                except:
                    frame_timestamp = None
                break
            try:
                a, d = map(float, line.strip().split(','))
                if 0.0 <= a <= 360.0:
                    current_frame.append((a, d))
            except:
                continue

        if current_frame and frame_timestamp:
            now = int(time.time() * 1000)
            self.last_delay_ms = now - frame_timestamp
            self.datapack = current_frame
            self.total_frames += 1

    def handle_key(self, key):
        if key in [pygame.K_KP_PLUS, pygame.K_EQUALS] and self.scale > 3:
            self.scale /= 2
        elif key in [pygame.K_KP_MINUS, pygame.K_MINUS] and self.scale < self.max_scale:
            self.scale *= 2
        elif key in [pygame.K_1, pygame.K_KP1]: self.scale = self.max_scale / 8
        elif key in [pygame.K_2, pygame.K_KP2]: self.scale = self.max_scale / 4
        elif key in [pygame.K_3, pygame.K_KP3]: self.scale = self.max_scale * 3 / 8
        elif key in [pygame.K_4, pygame.K_KP4]: self.scale = self.max_scale / 2
        elif key in [pygame.K_5, pygame.K_KP5]: self.scale = self.max_scale * 5 / 8
        elif key in [pygame.K_6, pygame.K_KP6]: self.scale = self.max_scale * 3 / 4
        elif key in [pygame.K_7, pygame.K_KP7]: self.scale = self.max_scale * 7 / 8
        elif key in [pygame.K_8, pygame.K_KP8]: self.scale = self.max_scale
        elif key == pygame.K_LEFTBRACKET and self.dot_size > 1:
            self.dot_size -= 1
        elif key == pygame.K_RIGHTBRACKET and self.dot_size < 6:
            self.dot_size += 1
        elif key == pygame.K_g:
            self.gridstyle = (self.gridstyle + 1) % 2
        elif key == pygame.K_d:
            self.set_dark_mode()
        elif key == pygame.K_b:
            self.set_light_mode()
        elif key == pygame.K_f:
            self.save_frame_to_csv()
        elif key == pygame.K_o:
            self.show_overlay = not self.show_overlay
        self.grid_surface = self.create_grid()

    def save_frame_to_csv(self):
        filename = f"frame_{int(time.time())}.csv"
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Angle', 'Distance'])
                for angle, distance in self.datapack:
                    writer.writerow([angle, distance])
            sys.stdout.write("\a")
            sys.stdout.flush()
        except Exception as e:
            print(f"Erro ao salvar frame: {e}")

    def update_screen(self):
        self.screen.fill(self.backcolor)
        self.screen.blit(self.grid_surface, (0, 0))
        for polar in self.datapack:
            pt = self.polar_to_point(polar)
            pygame.draw.circle(self.screen, (255, 0, 0), (int(pt[0]), int(pt[1])), self.dot_size)
        if self.show_overlay:
            if self.last_delay_ms >= 0:
                text = self.ui_font.render(f"Delay: {self.last_delay_ms:.1f} ms", False, (255, 255, 0))
                self.screen.blit(text, (10, self.display_size[1] - 40))
            fps_text = self.ui_font.render(f"FPS: {self.clock.get_fps():.1f}", False, (0, 255, 0))
            self.screen.blit(fps_text, (10, self.display_size[1] - 20))
        pygame.display.update()

    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (
                    event.type == pygame.KEYDOWN and (event.key == pygame.K_ESCAPE or event.key == pygame.K_q)):
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    self.handle_key(event.key)

            self.read_socket()
            self.update_screen()
            self.clock.tick(60)

        self.reader_proc.terminate()
        self.reader_proc.wait()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Lidar Viewer SFS")
    parser.add_argument('-f', '--fullscreen', action='store_true', help='Tela cheia')
    parser.add_argument('-w', '--window', type=int, help='Tamanho da janela (ex: 800)', default=800)
    parser.add_argument('-p', '--port', type=str, help='Porta serial (ex: /dev/ttyUSB0)', default='/dev/ttyUSB0')
    args = parser.parse_args()

    viewer = LidarViewerSFS(
        fullscreen=args.fullscreen,
        window_size=(args.window, args.window),
        serial_port=args.port
    )
    viewer.run()

