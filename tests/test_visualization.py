import pytest
import matplotlib.pyplot as plt
from src.models import Drone, DeliveryPoint, NoFlyZone
from src.visualization import DroneFleetVisualizer

def test_visualizer_creation():
    """Görselleştirici oluşturma testi"""
    drones = [Drone(id=1, max_energy=100.0, current_energy=100.0, max_weight=10.0)]
    deliveries = [DeliveryPoint(id=1, position=(10.0, 10.0), weight=5.0, priority=3)]
    zones = [NoFlyZone(id=1, center=(5.0, 5.0), radius=2.0, start_time=8.0, end_time=18.0)]
    
    visualizer = DroneFleetVisualizer(drones, deliveries, zones)
    assert len(visualizer.colors) == len(drones)

def test_environment_plot():
    """Çevre görselleştirme testi"""
    drones = [Drone(id=1, max_energy=100.0, current_energy=100.0, max_weight=10.0)]
    deliveries = [DeliveryPoint(id=1, position=(10.0, 10.0), weight=5.0, priority=3)]
    zones = [NoFlyZone(id=1, center=(5.0, 5.0), radius=2.0, start_time=8.0, end_time=18.0)]
    
    visualizer = DroneFleetVisualizer(drones, deliveries, zones)
    
    # Rota olmadan çizim
    fig = visualizer.plot_environment()
    assert isinstance(fig, plt.Figure)
    plt.close(fig)
    
    # Rota ile çizim
    routes = {1: [(0.0, 0.0), (5.0, 5.0), (10.0, 10.0)]}
    fig = visualizer.plot_environment(routes)
    assert isinstance(fig, plt.Figure)
    plt.close(fig)

def test_energy_consumption_plot():
    """Enerji tüketimi görselleştirme testi"""
    drones = [Drone(id=1, max_energy=100.0, current_energy=100.0, max_weight=10.0)]
    deliveries = [DeliveryPoint(id=1, position=(10.0, 10.0), weight=5.0, priority=3)]
    zones = [NoFlyZone(id=1, center=(5.0, 5.0), radius=2.0, start_time=8.0, end_time=18.0)]
    
    visualizer = DroneFleetVisualizer(drones, deliveries, zones)
    routes = {1: [(0.0, 0.0), (5.0, 5.0), (10.0, 10.0)]}
    
    fig = visualizer.plot_energy_consumption(routes)
    assert isinstance(fig, plt.Figure)
    plt.close(fig)

def test_delivery_statistics_plot():
    """Teslimat istatistikleri görselleştirme testi"""
    drones = [Drone(id=1, max_energy=100.0, current_energy=100.0, max_weight=10.0)]
    deliveries = [DeliveryPoint(id=1, position=(10.0, 10.0), weight=5.0, priority=3)]
    zones = [NoFlyZone(id=1, center=(5.0, 5.0), radius=2.0, start_time=8.0, end_time=18.0)]
    
    visualizer = DroneFleetVisualizer(drones, deliveries, zones)
    routes = {1: [(0.0, 0.0), (5.0, 5.0), (10.0, 10.0)]}
    
    fig = visualizer.plot_delivery_statistics(routes)
    assert isinstance(fig, plt.Figure)
    plt.close(fig) 