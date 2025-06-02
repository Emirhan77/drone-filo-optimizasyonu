from dataclasses import dataclass
from typing import List, Tuple
import numpy as np

@dataclass
class Drone:
    """Drone sınıfı, drone'un özelliklerini ve durumunu temsil eder."""
    id: int
    max_energy: float  # Maksimum enerji kapasitesi
    current_energy: float  # Mevcut enerji
    max_weight: float  # Maksimum taşıyabileceği ağırlık
    current_weight: float = 0.0  # Şu an taşıdığı ağırlık
    position: Tuple[float, float] = (0.0, 0.0)  # (x, y) koordinatları
    speed: float = 10.0 # Default speed, will be overwritten by data loading
    is_available: bool = True  # Drone'un müsait olup olmadığı

@dataclass
class DeliveryPoint:
    """Teslimat noktası sınıfı, teslimat noktasının özelliklerini temsil eder."""
    id: int
    position: Tuple[float, float]  # (x, y) koordinatları
    weight: float  # Paket ağırlığı
    priority: int  # Teslimat önceliği (1-5 arası)
    time_window_start: float = 0.0 # Teslimat için en erken zaman
    time_window_end: float = float('inf') # Teslimat için en geç zaman
    is_delivered: bool = False  # Teslimatın tamamlanıp tamamlanmadığı
    safe_position: Tuple[float, float] = None  # No-fly zone dışındaki güvenli teslimat noktası

@dataclass
class NoFlyZone:
    """Uçuşa yasak bölge sınıfı, yasaklı bölgenin özelliklerini temsil eder."""
    id: int
    coordinates: List[Tuple[float, float]]  # Poligonun köşe koordinatları
    start_time: float  # Başlangıç zamanı
    end_time: float  # Bitiş zamanı

class DataGenerator:
    """Rastgele veri üreteci sınıfı."""
    
    @staticmethod
    def generate_drones(num_drones: int, max_energy: float = 300.0, max_weight: float = 10.0) -> List[Drone]:
        """Belirtilen sayıda drone oluşturur. Tüm drone'lar merkezi noktada başlar."""
        center = (50.0, 50.0)
        return [
            Drone(
                id=i,
                max_energy=max_energy,
                current_energy=max_energy,
                max_weight=max_weight,
                position=center
            )
            for i in range(num_drones)
        ]

    @staticmethod
    def generate_delivery_points(num_points: int, area_size: float = 100.0) -> List[DeliveryPoint]:
        """Belirtilen sayıda teslimat noktası oluşturur."""
        return [
            DeliveryPoint(
                id=i,
                position=(
                    np.random.uniform(0, area_size),
                    np.random.uniform(0, area_size)
                ),
                weight=np.random.uniform(0.5, 5.0),
                priority=np.random.randint(1, 6)
            )
            for i in range(num_points)
        ]

    @staticmethod
    def generate_no_fly_zones(num_zones: int, area_size: float = 100.0) -> List[NoFlyZone]:
        """Belirtilen sayıda uçuşa yasak bölge oluşturur."""
        return [
            NoFlyZone(
                id=i,
                center=(
                    np.random.uniform(0, area_size),
                    np.random.uniform(0, area_size)
                ),
                radius=np.random.uniform(5.0, 15.0),
                start_time=np.random.uniform(0, 24),
                end_time=np.random.uniform(0, 24)
            )
            for i in range(num_zones)
        ] 