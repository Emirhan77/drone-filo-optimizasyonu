from dataclasses import dataclass
from typing import List, Tuple
import numpy as np

@dataclass
class Drone:
    """Drone sınıfı, drone'un özelliklerini ve durumunu temsil eder."""
    id: int
    max_weight: float  # Maksimum taşıyabileceği ağırlık
    battery: float  # Maksimum enerji kapasitesi (veri setindeki adlandırma)
    speed: float # Drone hızı
    start_pos: Tuple[float, float] # Başlangıç pozisyonu
    current_energy: float = 0.0 # Mevcut enerji, başlangıçta battery'ye eşitlenecek
    current_weight: float = 0.0  # Şu an taşıdığı ağırlık
    position: Tuple[float, float] = (0.0, 0.0)  # (x, y) koordinatları, başlangıçta start_pos'a eşitlenecek
    is_available: bool = True  # Drone'un müsait olup olmadığı

    def __post_init__(self):
        self.current_energy = self.battery
        self.position = self.start_pos

@dataclass
class DeliveryPoint:
    """Teslimat noktası sınıfı, teslimat noktasının özelliklerini temsil eder."""
    id: int
    pos: Tuple[float, float]  # (x, y) koordinatları (veri setindeki adlandırma)
    weight: float  # Paket ağırlığı
    priority: int  # Teslimat önceliği (1-5 arası)
    time_window: Tuple[float, float] # (başlangıç, bitiş) zaman aralığı
    is_delivered: bool = False  # Teslimatın tamamlanıp tamamlanmadığı
    safe_position: Tuple[float, float] = None  # No-fly zone dışındaki güvenli teslimat noktası

    @property
    def position(self) -> Tuple[float, float]: # algorithms.py uyumluluğu için
        return self.pos

@dataclass
class NoFlyZone:
    """Uçuşa yasak bölge sınıfı, yasaklı bölgenin özelliklerini temsil eder."""
    id: int
    coordinates: List[Tuple[float, float]]  # Çokgen köşe koordinatları
    active_time: Tuple[float, float]  # (başlangıç, bitiş) aktif zaman aralığı

    @property
    def start_time(self) -> float: # algorithms.py uyumluluğu için
        return self.active_time[0]

    @property
    def end_time(self) -> float: # algorithms.py uyumluluğu için
        return self.active_time[1]

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

import ast

def load_data_from_file(file_path: str) -> Tuple[List[Drone], List[DeliveryPoint], List[NoFlyZone]]:
    """
    Veri setini dosyadan yükler ve model nesnelerini oluşturur.
    Dosya formatı: Python listeleri ve sözlükleri içeren metin dosyası.
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # İçeriği güvenli bir şekilde Python nesnelerine dönüştür
    # Her bir listeyi ayrı ayrı bulup parse etmemiz gerekiyor
    # Basit bir varsayım: her liste "variable_name = [...]" formatında
    
    drones_str_start = content.find('drones = [')
    drones_str_end = content.find(']', drones_str_start) + 1
    drones_list_str = content[drones_str_start + len('drones = '):drones_str_end]
    
    deliveries_str_start = content.find('deliveries = [')
    deliveries_str_end = content.find(']', deliveries_str_start) + 1
    deliveries_list_str = content[deliveries_str_start + len('deliveries = '):deliveries_str_end]
    
    no_fly_zones_str_start = content.find('no_fly_zones = [')
    # no_fly_zones için kapanış parantezini daha dikkatli bulalım, iç içe yapılar olabilir
    # Bu basit parser için, son kapanış parantezini bulmak yeterli olacaktır.
    # Daha karmaşık dosyalar için daha sağlam bir parser gerekebilir.
    open_brackets = 0
    current_pos = no_fly_zones_str_start + content[no_fly_zones_str_start:].find('[')
    no_fly_zones_actual_start = current_pos
    
    for char_idx in range(current_pos, len(content)):
        if content[char_idx] == '[':
            open_brackets += 1
        elif content[char_idx] == ']':
            open_brackets -= 1
            if open_brackets == 0:
                no_fly_zones_str_end = char_idx + 1
                break
    else: # Döngü break ile bitmezse (hata durumu)
        raise ValueError("No-fly zones listesi düzgün kapatılmamış.")

    no_fly_zones_list_str = content[no_fly_zones_actual_start:no_fly_zones_str_end]

    try:
        drones_data = ast.literal_eval(drones_list_str)
        deliveries_data = ast.literal_eval(deliveries_list_str)
        no_fly_zones_data = ast.literal_eval(no_fly_zones_list_str)
    except Exception as e:
        raise ValueError(f"Veri dosyasını parse ederken hata: {e}\n"
                         f"Drones: {drones_list_str[:100]}...\n"
                         f"Deliveries: {deliveries_list_str[:100]}...\n"
                         f"NoFlyZones: {no_fly_zones_list_str[:100]}...")


    drones = [Drone(**d) for d in drones_data]
    delivery_points = [DeliveryPoint(**dp) for dp in deliveries_data]
    no_fly_zones = [NoFlyZone(**nfz) for nfz in no_fly_zones_data]

    return drones, delivery_points, no_fly_zones