import pytest
from src.models import Drone, DeliveryPoint, NoFlyZone, DataGenerator

def test_drone_creation():
    """Drone oluşturma testi"""
    drone = Drone(id=1, max_energy=100.0, current_energy=100.0, max_weight=10.0)
    assert drone.id == 1
    assert drone.max_energy == 100.0
    assert drone.current_energy == 100.0
    assert drone.max_weight == 10.0
    assert drone.current_weight == 0.0
    assert drone.is_available == True

def test_delivery_point_creation():
    """Teslimat noktası oluşturma testi"""
    delivery = DeliveryPoint(
        id=1,
        position=(10.0, 20.0),
        weight=5.0,
        priority=3
    )
    assert delivery.id == 1
    assert delivery.position == (10.0, 20.0)
    assert delivery.weight == 5.0
    assert delivery.priority == 3
    assert delivery.is_delivered == False

def test_no_fly_zone_creation():
    """Uçuşa yasak bölge oluşturma testi"""
    zone = NoFlyZone(
        id=1,
        center=(15.0, 25.0),
        radius=10.0,
        start_time=8.0,
        end_time=18.0
    )
    assert zone.id == 1
    assert zone.center == (15.0, 25.0)
    assert zone.radius == 10.0
    assert zone.start_time == 8.0
    assert zone.end_time == 18.0

def test_data_generator():
    """Veri üreteci testi"""
    num_drones = 5
    num_deliveries = 10
    num_zones = 3
    
    drones = DataGenerator.generate_drones(num_drones)
    deliveries = DataGenerator.generate_delivery_points(num_deliveries)
    zones = DataGenerator.generate_no_fly_zones(num_zones)
    
    assert len(drones) == num_drones
    assert len(deliveries) == num_deliveries
    assert len(zones) == num_zones
    
    # Drone özelliklerini kontrol et
    for drone in drones:
        assert isinstance(drone, Drone)
        assert drone.max_energy > 0
        assert drone.max_weight > 0
    
    # Teslimat noktası özelliklerini kontrol et
    for delivery in deliveries:
        assert isinstance(delivery, DeliveryPoint)
        assert 0 <= delivery.priority <= 5
        assert delivery.weight > 0
    
    # Uçuşa yasak bölge özelliklerini kontrol et
    for zone in zones:
        assert isinstance(zone, NoFlyZone)
        assert zone.radius > 0
        assert 0 <= zone.start_time <= 24
        assert 0 <= zone.end_time <= 24 