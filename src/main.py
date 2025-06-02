from src.models import load_data_from_file # DataGenerator yerine load_data_from_file
from src.algorithms import DroneFleetOptimizer
from src.visualization import DroneFleetVisualizer
import matplotlib.pyplot as plt
import time

# run_scenario parametreleri dosya yolu alacak şekilde veya sabit dosya yolu kullanacak şekilde güncellenebilir.
# Şimdilik sabit dosya yolu kullanacağız ve num_drones, num_deliveries, num_zones parametrelerini kaldıracağız.
def run_scenario(data_file_path: str = "data/veri_seti.txt"):
    """Örnek senaryo çalıştırma fonksiyonu"""
    
    # Veri yükle
    start_time = time.time()
    try:
        drones, deliveries, zones = load_data_from_file(data_file_path)
    except Exception as e:
        print(f"Veri yüklenirken hata oluştu: {e}")
        return
    
    data_load_time = time.time() - start_time
    
    num_drones = len(drones)
    num_deliveries_initial = len(deliveries) # Başlangıçtaki teslimat sayısı
    num_zones = len(zones)

    print(f"\nSenaryo: {data_file_path} dosyasından yüklendi.")
    print(f"{num_drones} drone, {num_deliveries_initial} teslimat, {num_zones} uçuşa yasak bölge")
    print("-" * 50)
    print(f"Veri yükleme tamamlandı ({data_load_time:.2f} saniye)")
    
    print("No-fly zone'lar:")
    for z in zones:
        print(f"  ID: {z.id}, Koordinatlar: {z.coordinates}, Aktif Zaman: {z.active_time}")
    print("Teslimat noktaları:")
    for d in deliveries:
        print(f"  {d.id}: {d.position}")
    
    # Optimizasyon
    optimizer = DroneFleetOptimizer(drones, deliveries, zones)
    start_time = time.time()
    routes = optimizer.optimize()
    optimization_time = time.time() - start_time
    
    print(f"Optimizasyon tamamlandı ({optimization_time:.2f} saniye)")
    
    # Sonuçları analiz et
    total_deliveries = sum(1 for d in deliveries if d.is_delivered)
    completion_rate = (total_deliveries / num_deliveries_initial) * 100 if num_deliveries_initial > 0 else 0
    
    # Ortalama enerji tüketimi hesapla
    total_energy_used = 0
    used_drone_count = 0
    for drone in drones:
        if routes.get(drone.id):
            # Rota üzerindeki toplam mesafe (enerji)
            path = routes[drone.id]
            energy_used = sum(
                optimizer._euclidean_distance(path[i], path[i+1])
                for i in range(len(path)-1)
            )
            total_energy_used += energy_used
            used_drone_count += 1
    avg_energy = total_energy_used / used_drone_count if used_drone_count else 0
    print(f"Ortalama enerji tüketimi: {avg_energy:.2f}")
    
    print(f"\nSonuçlar:")
    print(f"Tamamlanan teslimat sayısı: {total_deliveries}/{num_deliveries_initial} ({completion_rate:.1f}%)")
    print(f"Kullanılan drone sayısı: {used_drone_count}/{num_drones}")
    
    # Görselleştirme
    visualizer = DroneFleetVisualizer(drones, deliveries, zones)
    
    # Rota görselleştirmesi
    visualizer.plot_environment(routes)
    plt.show()
    # Enerji tüketimi
    visualizer.plot_energy_consumption(routes)
    plt.show()
    # Teslimat istatistikleri
    visualizer.plot_delivery_statistics(routes)
    plt.show()

def main():
    """Ana fonksiyon"""
    print("Drone Filo Optimizasyonu - Test Senaryoları")
    print("=" * 50)
    
    # Senaryo: data/veri_seti.txt dosyasından yükle
    run_scenario() # Parametreler kaldırıldığı için argümansız çağrı
    
    # Eski senaryolar (artık doğrudan numara ile değil, farklı veri dosyaları ile çalıştırılabilir)
    # run_scenario("data/baska_bir_senaryo.txt")

if __name__ == "__main__":
    main() 