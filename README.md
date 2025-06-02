# Drone Filo Optimizasyonu

Bu proje, drone filosu ile teslimat rotalarının optimize edilmesi için geliştirilmiş bir sistemdir. Sistem, enerji kısıtları, ağırlık limitleri, teslimat öncelikleri ve uçuşa yasak bölgeler gibi dinamik kısıtları dikkate alarak optimal teslimat rotalarını belirler.

## Özellikler

- Drone, teslimat noktası ve uçuşa yasak bölge veri yapıları
- A* algoritması ile rota optimizasyonu
- Genetik algoritma ile çoklu drone optimizasyonu
- Dinamik kısıtlar için CSP (Constraint Satisfaction Problem) çözümü
- Performans analizi ve görselleştirme araçları

## Kurulum

1. Python 3.8 veya üstü sürümü yükleyin
2. Gerekli paketleri yükleyin:
   ```bash
   pip install -r requirements.txt
   ```

## Proje Yapısı

```
.
├── src/                    # Kaynak kodlar
│   ├── models.py          # Veri yapıları
│   ├── algorithms.py      # Optimizasyon algoritmaları
│   └── utils.py           # Yardımcı fonksiyonlar
├── tests/                 # Test dosyaları
├── data/                  # Test verileri
└── docs/                  # Dokümantasyon
```

## Kullanım

1. Test verilerini oluşturun:
   ```python
   from src.models import DataGenerator
   
   drones = DataGenerator.generate_drones(5)
   deliveries = DataGenerator.generate_delivery_points(20)
   zones = DataGenerator.generate_no_fly_zones(2)
   ```

2. Optimizasyon algoritmasını çalıştırın:
   ```python
   from src.algorithms import DroneFleetOptimizer
   
   optimizer = DroneFleetOptimizer(drones, deliveries, zones)
   routes = optimizer.optimize()
   ```

## Test

Testleri çalıştırmak için:
```bash
pytest tests/
```

## Lisans

Bu proje MIT lisansı altında lisanslanmıştır. 