import pytest
from src.models import Drone, DeliveryPoint, NoFlyZone
from src.algorithms import DroneFleetOptimizer, Node

def test_node_creation():
    """Node sınıfı oluşturma testi"""
    node = Node(
        position=(10.0, 20.0),
        g_cost=5.0,
        h_cost=10.0
    )
    assert node.position == (10.0, 20.0)
    assert node.g_cost == 5.0
    assert node.h_cost == 10.0
    assert node.f_cost == 15.0
    assert node.parent is None

def test_euclidean_distance():
    """Öklid mesafesi hesaplama testi"""
    optimizer = DroneFleetOptimizer([], [], [])
    pos1 = (0.0, 0.0)
    pos2 = (3.0, 4.0)
    assert optimizer._euclidean_distance(pos1, pos2) == 5.0

def test_no_fly_zone_check():
    """Uçuşa yasak bölge kontrolü testi"""
    zone = NoFlyZone(
        id=1,
        center=(10.0, 10.0),
        radius=5.0,
        start_time=8.0,
        end_time=18.0
    )
    optimizer = DroneFleetOptimizer([], [], [zone])
    
    # Yasak bölge içindeki nokta
    assert optimizer._is_in_no_fly_zone((10.0, 10.0), 12.0) == True
    # Yasak bölge dışındaki nokta
    assert optimizer._is_in_no_fly_zone((20.0, 20.0), 12.0) == False
    # Yasak bölge zamanı dışındaki nokta
    assert optimizer._is_in_no_fly_zone((10.0, 10.0), 20.0) == False

def test_path_finding():
    """Yol bulma algoritması testi"""
    # Test verilerini oluştur
    drone = Drone(id=1, max_energy=100.0, current_energy=100.0, max_weight=10.0)
    delivery = DeliveryPoint(
        id=1,
        position=(10.0, 10.0),
        weight=5.0,
        priority=3
    )
    zone = NoFlyZone(
        id=1,
        center=(5.0, 5.0),
        radius=2.0,
        start_time=8.0,
        end_time=18.0
    )
    
    optimizer = DroneFleetOptimizer([drone], [delivery], [zone])
    
    # Basit bir yol bulma testi
    path = optimizer.find_path((0.0, 0.0), (10.0, 10.0), 12.0)
    assert path is not None
    assert len(path) > 0
    assert path[0] == (0.0, 0.0)  # Başlangıç noktası
    assert path[-1] == (10.0, 10.0)  # Hedef nokta

def test_optimization():
    """Optimizasyon algoritması testi"""
    # Test verilerini oluştur
    drones = [
        Drone(id=1, max_energy=100.0, current_energy=100.0, max_weight=10.0),
        Drone(id=2, max_energy=100.0, current_energy=100.0, max_weight=10.0)
    ]
    
    deliveries = [
        DeliveryPoint(id=1, position=(10.0, 10.0), weight=5.0, priority=3),
        DeliveryPoint(id=2, position=(20.0, 20.0), weight=3.0, priority=2)
    ]
    
    zones = [
        NoFlyZone(id=1, center=(5.0, 5.0), radius=2.0, start_time=8.0, end_time=18.0)
    ]
    
    optimizer = DroneFleetOptimizer(drones, deliveries, zones)
    routes = optimizer.optimize()
    
    assert isinstance(routes, dict)
    assert len(routes) > 0  # En az bir drone için rota bulunmalı
    
    # Rotaların doğruluğunu kontrol et
    for drone_id, path in routes.items():
        assert len(path) > 0
        # Başlangıç ve bitiş noktaları arasında bağlantı olmalı
        for i in range(len(path) - 1):
            assert optimizer._euclidean_distance(path[i], path[i + 1]) <= optimizer.grid_resolution * 2 