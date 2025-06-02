import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple, Dict
from src.models import Drone, DeliveryPoint, NoFlyZone
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math

class DroneFleetVisualizer:
    """Drone filosu görselleştirme sınıfı"""
    
    def __init__(self, drones: List[Drone], delivery_points: List[DeliveryPoint], no_fly_zones: List[NoFlyZone]):
        self.drones = drones
        self.delivery_points = delivery_points
        self.no_fly_zones = no_fly_zones
        self.colors = plt.cm.rainbow(np.linspace(0, 1, max(1, len(drones))))
    
    def plot_environment(self, routes: Dict[int, List[Tuple[float, float]]] = None, title_suffix: str = ""):
        """Drone rotalarını ve çevreyi görselleştirir"""
        plt.figure(figsize=(12, 8))
        ax = plt.gca()
        
        # No-fly zone
        for zone in self.no_fly_zones:
            if hasattr(zone, 'coordinates') and zone.coordinates:
                polygon = plt.Polygon(zone.coordinates, color='red', alpha=0.3, ec='darkred', lw=2, zorder=1)
                ax.add_patch(polygon)
                # Calculate centroid for text label
                center_x = np.mean([c[0] for c in zone.coordinates])
                center_y = np.mean([c[1] for c in zone.coordinates])
                plt.text(center_x, center_y, f'No-Fly Zone {zone.id}', ha='center', va='center', fontsize=12, color='darkred', zorder=2)
            elif hasattr(zone, 'center') and hasattr(zone, 'radius'): # Fallback for old data structure if any
                circle = plt.Circle(zone.center, zone.radius, color='red', alpha=0.3, ec='darkred', lw=2, zorder=1)
                ax.add_patch(circle)
                plt.text(zone.center[0], zone.center[1], f'No-Fly Zone {zone.id}', ha='center', va='center', fontsize=12, color='darkred', zorder=2)

        # Teslimat noktaları
        for delivery in self.delivery_points:
            color = 'green' if delivery.is_delivered else 'red'
            plt.scatter(delivery.position[0], delivery.position[1], color=color, marker='o', s=120, zorder=3, edgecolor='black', label='Delivered' if (color=='green') else 'Undelivered')
            plt.text(delivery.position[0], delivery.position[1]+2, f'D{delivery.id}', ha='center', va='bottom', fontsize=10, color='black', zorder=4)
        
        # Drone başlangıç noktaları
        for i, drone in enumerate(self.drones):
            plt.scatter(drone.position[0], drone.position[1], color='yellow', marker='*', s=250, edgecolor='black', zorder=5, label=f'Drone {drone.id} Start' if i==0 else None)
            plt.text(drone.position[0], drone.position[1]-2, f'Drone {drone.id} Start', ha='center', va='top', fontsize=11, color='orange', zorder=6)
        
        # Rotalar ve oklar
        if routes:
            for i, (drone_id, path) in enumerate(routes.items()):
                if path:
                    path = np.array(path)
                    plt.plot(path[:, 0], path[:, 1], color=self.colors[i], label=f'Drone {drone_id} Route', linewidth=2, alpha=0.7, zorder=7)
                    # Oklar
                    for j in range(len(path)-1):
                        dx = path[j+1, 0] - path[j, 0]
                        dy = path[j+1, 1] - path[j, 1]
                        plt.arrow(path[j, 0], path[j, 1], dx, dy, color=self.colors[i], shape='full', lw=0, length_includes_head=True, head_width=1.5, head_length=2.5, alpha=0.5, zorder=8)
        
        main_title = 'Drone Fleet Routes and Environment'
        if title_suffix:
            main_title += title_suffix
        plt.title(main_title, fontsize=16)
        plt.xlabel('X Coordinate', fontsize=13)
        plt.ylabel('Y Coordinate', fontsize=13)
        plt.grid(True)
        # Legend'i tekrar çizmek için unique label'lar al
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys())
        plt.axis('equal')
        plt.xlim(0, 100)
        plt.ylim(0, 100)
        
        return plt.gcf()
    
    def plot_energy_consumption(self, routes: Dict[int, List[Tuple[float, float]]]):
        """Drone'ların enerji tüketimini görselleştirir"""
        plt.figure(figsize=(10, 6))
        
        energy_consumption = []
        drone_ids = []
        
        for drone_id, path in routes.items():
            if path:
                total_distance = sum(
                    np.sqrt((path[i+1][0] - path[i][0])**2 + 
                           (path[i+1][1] - path[i][1])**2)
                    for i in range(len(path)-1)
                )
                energy_consumption.append(total_distance)
                drone_ids.append(drone_id)
        
        plt.bar(drone_ids, energy_consumption, color=self.colors[:len(drone_ids)])
        plt.title('Energy Consumption per Drone')
        plt.xlabel('Drone ID')
        plt.ylabel('Energy Consumption')
        plt.grid(True, axis='y')
        
        return plt.gcf()
    
    def plot_delivery_statistics(self, routes: Dict[int, List[Tuple[float, float]]]):
        """Teslimat istatistiklerini görselleştirir"""
        plt.figure(figsize=(10, 6))
        
        deliveries_per_drone = []
        drone_ids = []
        
        for drone_id, path in routes.items():
            if path:
                # Teslimat noktalarını say
                deliveries = sum(1 for delivery in self.delivery_points
                               if delivery.is_delivered)
                deliveries_per_drone.append(deliveries)
                drone_ids.append(drone_id)
        
        plt.bar(drone_ids, deliveries_per_drone, color=self.colors[:len(drone_ids)])
        plt.title('Deliveries per Drone')
        plt.xlabel('Drone ID')
        plt.ylabel('Number of Deliveries')
        plt.grid(True, axis='y')
        
        return plt.gcf()

class DroneSimulation3D:
    def __init__(self, width: int = 800, height: int = 600):
        pygame.init()
        self.width = width
        self.height = height
        pygame.display.set_mode((width, height), DOUBLEBUF | OPENGL)
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.15, 0.15, 0.15, 1.0)  # Gri arka plan
        gluPerspective(45, (width/height), 0.1, 1000.0)
        glTranslatef(-50, -30, -180)  # Kamerayı ortala ve biraz yukarıdan bak
        glRotatef(45, 1, 0, 0)        # Hafif yukarıdan bak
        self.rotation_x = 45
        self.rotation_y = 0
        self.zoom = -180

    def draw_grid(self, size: float = 100.0, step: float = 10.0):
        glLineWidth(1.5)
        glBegin(GL_LINES)
        glColor3f(0.7, 0.7, 0.7)
        for i in range(-int(size/2), int(size/2) + 1, int(step)):
            glVertex3f(i, 0, -size/2)
            glVertex3f(i, 0, size/2)
            glVertex3f(-size/2, 0, i)
            glVertex3f(size/2, 0, i)
        glEnd()
        glLineWidth(1)

    def draw_drone(self, position: Tuple[float, float, float], color: Tuple[float, float, float]):
        x, y, z = position
        glPushMatrix()
        glTranslatef(x, y, z)
        glColor3f(*color)
        # Gövde
        glBegin(GL_QUADS)
        glVertex3f(-1.5, 0, -1.5)
        glVertex3f(1.5, 0, -1.5)
        glVertex3f(1.5, 0.7, -1.5)
        glVertex3f(-1.5, 0.7, -1.5)
        glEnd()
        # Pervaneler
        propeller_positions = [(-1.5, 0.7, -1.5), (1.5, 0.7, -1.5), (-1.5, 0.7, 1.5), (1.5, 0.7, 1.5)]
        for px, py, pz in propeller_positions:
            glPushMatrix()
            glTranslatef(px, py, pz)
            glColor3f(0.2, 0.2, 0.2)
            glBegin(GL_TRIANGLE_FAN)
            for i in range(12):
                angle = i * (2 * math.pi / 12)
                glVertex3f(0.5 * math.cos(angle), 0, 0.5 * math.sin(angle))
            glEnd()
            glPopMatrix()
        glPopMatrix()

    def draw_delivery_point(self, position: Tuple[float, float], color: Tuple[float, float, float]):
        x, y = position
        glPushMatrix()
        glTranslatef(x, 0, y)
        glColor3f(*color)
        glBegin(GL_QUADS)
        glVertex3f(-2.5, 0, -2.5)
        glVertex3f(2.5, 0, -2.5)
        glVertex3f(2.5, 0, 2.5)
        glVertex3f(-2.5, 0, 2.5)
        glEnd()
        glPopMatrix()

    def draw_no_fly_zone(self, zone: NoFlyZone): # Changed signature to take NoFlyZone object
        if hasattr(zone, 'coordinates') and zone.coordinates:
            # Draw polygon (assuming it's a flat rectangle on the ground plane)
            glPushMatrix()
            # No translation needed if coordinates are absolute
            glColor4f(1.0, 0.0, 0.0, 0.35) # Red, semi-transparent
            glBegin(GL_POLYGON)
            for coord in zone.coordinates:
                glVertex3f(coord[0], 0.01, coord[1]) # y=0.01 to be slightly above grid
            glEnd()
            glPopMatrix()
        elif hasattr(zone, 'center') and hasattr(zone, 'radius'): # Fallback for old circle-based NFZ
            x, y = zone.center
            radius = zone.radius
            glPushMatrix()
            glTranslatef(x, 0, y)
            glColor4f(1.0, 0.0, 0.0, 0.35)
            glBegin(GL_TRIANGLE_FAN)
            glVertex3f(0, 0.01, 0)
            for i in range(40):
                angle = i * (2 * math.pi / 40)
                glVertex3f(radius * math.cos(angle), 0.01, radius * math.sin(angle))
            glEnd()
            glPopMatrix()

    def draw_path(self, path: List[Tuple[float, float]], color: Tuple[float, float, float]):
        if len(path) < 2:
            return
        glLineWidth(3)
        glBegin(GL_LINE_STRIP)
        glColor3f(*color)
        for point in path:
            x, y = point
            glVertex3f(x, 2, y)
        glEnd()
        glLineWidth(1)

    def update_camera(self):
        glLoadIdentity()
        gluPerspective(45, (self.width/self.height), 0.1, 1000.0)
        glTranslatef(-50, -30, self.zoom)
        glRotatef(self.rotation_x, 1, 0, 0)
        glRotatef(self.rotation_y, 0, 1, 0)

    def handle_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    self.rotation_y -= 5
                elif event.key == pygame.K_RIGHT:
                    self.rotation_y += 5
                elif event.key == pygame.K_UP:
                    self.rotation_x -= 5
                elif event.key == pygame.K_DOWN:
                    self.rotation_x += 5
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    self.zoom += 10
                elif event.key == pygame.K_MINUS:
                    self.zoom -= 10
        return True

    def render(self, drones: List[Drone], delivery_points: List[DeliveryPoint], 
              no_fly_zones: List[NoFlyZone], routes: Dict[int, List[Tuple[float, float]]]):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self.draw_grid()
        for zone in no_fly_zones:
            self.draw_no_fly_zone(zone) # Pass the whole zone object
        for point in delivery_points:
            color = (0, 1, 0) if point.is_delivered else (1, 1, 0)
            self.draw_delivery_point(point.position, color)
        colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (1, 0, 1)]
        for drone_id, route in routes.items():
            if route:
                color = colors[drone_id % len(colors)]
                self.draw_path(route, color)
        for drone in drones:
            color = colors[drone.id % len(colors)]
            self.draw_drone((drone.position[0], 2, drone.position[1]), color)
        pygame.display.flip()
        pygame.time.wait(10)

    def run_simulation(self, drones: List[Drone], delivery_points: List[DeliveryPoint], 
                      no_fly_zones: List[NoFlyZone], routes: Dict[int, List[Tuple[float, float]]]):
        running = True
        while running:
            running = self.handle_input()
            self.update_camera()
            self.render(drones, delivery_points, no_fly_zones, routes)
        pygame.quit() 