# Drone Filosu Optimizasyon Projesi

Bu proje, drone filosu teslimat rotalarını optimize etmek için iki algoritmayı uygular ve karşılaştırır: Açgözlü (Greedy) Algoritma ve Genetik Algoritma. Simülasyon, drone enerji kısıtlamalarını, yük kapasitelerini, aktif zamanları olan Uçuşa Yasak Bölgeleri (NFZ'ler), bireysel drone hızlarını ve teslimat zaman pencerelerini dikkate alır.

## Proje Yapısı

```
.
├── 2425_yazLab_II_drone_filo.pdf  # Proje şartname dokümanı (varsayılan)
├── README.md                      # Bu dosya (Türkçe)
├── requirements.txt               # Python bağımlılıkları
├── data/
│   └── veri_seti.txt              # Drone'lar, teslimatlar ve NFZ'ler için veri seti
├── docs/                          # (Opsiyonel) Ek dokümantasyon
├── src/
│   ├── __init__.py                # src klasörünü bir paket yapar
│   ├── algorithms.py              # Optimizasyon algoritmalarını içerir (Açgözlü, GA, A*)
│   ├── main.py                    # Simülasyonları çalıştırmak için ana betik
│   ├── models.py                  # Drone, DeliveryPoint, NoFlyZone için veri sınıfları
│   └── visualization.py           # Görselleştirme mantığı (2D çizimler, 3D simülasyon)
└── tests/                         # (Opsiyonel) Birim testleri
    ├── __init__.py
    ├── test_algorithms.py
    ├── test_models.py
    └── test_visualization.py
```

## Özellikler

*   **Veri Odaklı Senaryolar**: Drone özelliklerini, teslimat noktası detaylarını (öncelik ve zaman pencereleri dahil) ve Uçuşa Yasak Bölge (aktif zamanları olan çokgensel) bilgilerini `data/veri_seti.txt` dosyasından yükler.
*   **Drone Modeli**:
    *   Bireysel ID, başlangıç pozisyonu, maksimum enerji (batarya), maksimum ağırlık kapasitesi ve hız.
    *   Mevcut enerji ve yük takibi.
*   **Teslimat Noktası Modeli**:
    *   ID, pozisyon, paket ağırlığı, öncelik seviyesi.
    *   Teslimat zaman pencerelerini (başlangıç ve bitiş zamanları) destekler.
*   **Uçuşa Yasak Bölge (NFZ) Modeli**:
    *   Koordinatlarla tanımlanan dikdörtgensel (çokgensel) NFZ'ler.
    *   Aktif periyotlarını tanımlayan `start_time` ve `end_time` ile dinamik NFZ'ler.
    *   Rota planlaması için NFZ'ler etrafında güvenlik tamponu.
*   **Rota Bulma**:
    *   **A\* Algoritması**: NFZ'leri ve aktif zamanlarını dikkate alarak noktalar arasında en uygun yolları bulmak için kullanılır. Sezgisel yöntemler ve güvenlik kontrolleri içerir.
    *   **Görünürlük Grafiği (Visibility Graph)**: Alternatif bir rota bulma stratejisi (bazı senaryolarda A\*'dan daha az hesaplama yoğun olabilir).
    *   **Doğrudan Yol & NFZ Kaçınma Sezgisel Yöntemi**: Açgözlü algoritmada A\* veya Görünürlük Grafiğine başvurmadan önce daha basit rota bulma denemeleri.
*   **Optimizasyon Algoritmaları**:
    1.  **Açgözlü (Greedy) Algoritma**:
        *   Bir maliyet fonksiyonuna göre mevcut en iyi drone'a teslimatları yinelemeli olarak atar.
        *   Maliyet Fonksiyonu (PDF'ten): `maliyet = mesafe * ağırlık + (öncelik * 100)`
        *   Drone enerjisini, ağırlık kapasitesini ve NFZ'lerden kaçınmayı dikkate alır.
        *   Şu anda teslimat zaman pencerelerini zorunlu *kılmaz*.
    2.  **Genetik Algoritma (GA)**:
        *   Potansiyel çözümlerden oluşan bir popülasyonu (drone-teslimat atamalarını temsil eden kromozomlar) evrimleştirir.
        *   **Kromozom Yapısı**: `drone_id`'yi bir `delivery_id` listesine eşleyen sözlük.
        *   **Uygunluk Fonksiyonu** (PDF'ten, zaman pencereleri için uyarlanmış): `Uygunluk = (Teslimat Sayısı * 50) - (Toplam Enerji * 0.1) - (İhlal Edilen Kısıt * 1000)`
            *   Kısıt ihlalleri şunları içerir: drone enerjisinin aşılması, ağırlık kapasitesinin aşılması, NFZ ihlalleri (evrim sırasında yaklaşık, son rota oluşturmada kesin) ve **teslimat zaman penceresi ihlalleri**.
        *   **Seçilim**: Turnuva seçilimi.
        *   **Çaprazlama (Crossover)**: Drone rotalarının tekdüze benzeri çaprazlaması, ardından teslimat benzersizliğini sağlamak için bir onarım mekanizması.
        *   **Mutasyon**: Bir rota içinde teslimatları değiştirme, atanmamış bir teslimatı ekleme veya bir teslimatı kaldırma gibi işlemleri içerir.
        *   **Elitizm**: En iyi bireyleri bir sonraki nesle taşır.
        *   **Hız İçin Yaklaşık Uygunluk**: Evrim sırasında enerji için Öklid mesafelerini ve hız için basitleştirilmiş NFZ kontrollerini kullanır. Son en iyi çözüm için tam A\* rota bulma kullanılır.
        *   **Dinamik Zaman Takibi**: Zaman bağımlı NFZ'leri ve teslimat zaman pencerelerini doğru bir şekilde değerlendirmek için her bir drone'nun rotası için tahmini mevcut zamanı izler.
        *   Zaman hesaplamaları için bireysel drone hızlarını kullanır.
*   **Görselleştirme (2D)**:
    *   **Matplotlib Kullanımı**: Proje, `DroneFleetVisualizer` sınıfı aracılığıyla Matplotlib kullanarak 2D görselleştirmeler sunar.
        *   Drone rotaları, başlangıç/bitiş noktaları, teslimat konumları (başarılı/başarısız durumlarına göre renk kodlu) ve Uçuşa Yasak Bölgeler (NFZ) harita üzerinde gösterilir.
        *   Drone başına enerji tüketimi ve yapılan teslimat sayısı gibi istatistikler için ayrı bar grafikler oluşturulur.
    *   **(Not: 3D Simülasyon Entegre Değil)**: `src/visualization.py` dosyası içerisinde `DroneSimulation3D` adında bir sınıf bulunmaktadır. Bu sınıf, Pygame ve OpenGL kullanarak temel bir 3D gösterim yapma potansiyeline sahip olsa da, bu yetenek mevcut `main.py` ana program akışına entegre edilmemiştir ve optimizasyon sonuçlarını doğrudan 3D olarak görselleştirmek için aktif olarak kullanılmamaktadır.

## Kurulum

1.  **Depoyu klonlayın (eğer uygulanabilirse).**
2.  **Bağımlılıkları yükleyin**:
    Python 3'ün kurulu olduğundan emin olun. Ardından, pip kullanarak gerekli paketleri yükleyin:
    ```bash
    pip install -r requirements.txt
    ```
    `requirements.txt` dosyası şunları içermelidir:
    ```
    matplotlib
    numpy
    networkx
    pygame
    PyOpenGL
    ```

## Kullanım

Simülasyonu çalıştırmak ve hem Açgözlü hem de Genetik algoritmaların sonuçlarını görmek için:

```bash
python -m src.main
```

Bu komut, `src` paketi içinde kullanılan göreli içe aktarmalar nedeniyle gerekli olan `main.py` betiğini bir modül olarak yürütür.

Betik şunları yapacaktır:
1.  `data/veri_seti.txt` dosyasından verileri yükleyecektir.
2.  Senaryo ayrıntılarını yazdıracaktır.
3.  Açgözlü algoritmayı çalıştıracak, sonuçlarını (teslimat sayısı, enerji vb.) yazdıracak ve rota grafiğini görüntüleyecektir.
4.  Genetik Algoritmayı çalıştıracak, sonuçlarını yazdıracak ve rota grafiğini, ardından GA için enerji ve teslimat istatistikleri grafiklerini görüntüleyecektir.

## Kod Genel Bakışı

### `src/models.py`
Temel veri yapılarını tanımlar:
*   `Drone`: Bir drone'un özelliklerini ve durumunu temsil eder.
*   `DeliveryPoint`: Bir teslimatın özelliklerini, zaman penceresi dahil olmak üzere temsil eder.
*   `NoFlyZone`: Koordinatları ve aktif zamanları olan bir uçuşa yasak bölgeyi temsil eder.
*   `DataGenerator`: (Rastgele veri üretmek için daha eski bir bileşen gibi görünmektedir, şu anda `veri_seti.txt` dosyasından yükleme yapan `main.py` tarafından kullanılmamaktadır).

### `src/algorithms.py`
Optimizasyon mantığını barındıran `DroneFleetOptimizer` sınıfını içerir:
*   **Rota Bulma**: `_euclidean_distance`, `_heuristic`, `_is_in_no_fly_zone`, `_segment_intersects_rectangle`, `_is_path_safe`, `find_path` (A\*), `build_visibility_graph`, `find_visibility_path`, `plan_path_around_no_fly_zone`.
*   **Açgözlü Algoritma**: `optimize()` metodu.
*   **Genetik Algoritma**:
    *   `optimize_with_ga()`: Ana GA döngüsü.
    *   `_initialize_population()`: İlk kromozom setini oluşturur.
    *   `_evaluate_chromosome_details()`: Bir kromozom için teslimatları, enerjiyi ve ihlalleri (zaman penceresi kontrolleri dahil) hesaplar.
    *   `_calculate_fitness()`: Uygunluk skorunu hesaplar.
    *   `_selection()`: Üreme için ebeveynleri seçer.
    *   `_crossover()`: Ebeveyn kromozomlarını birleştirir.
    *   `_repair_chromosome_for_unique_deliveries()`: Teslimat benzersizliğini sağlar.
    *   `_mutation()`: Kromozomlara varyasyonlar ekler.

### `src/visualization.py`
*   `DroneFleetVisualizer`: Matplotlib kullanarak 2D çizimler oluşturur (rotalar, enerji tüketimi, teslimat istatistikleri). Bu, `main.py` tarafından kullanılan birincil görselleştirme yöntemidir.
*   `DroneSimulation3D`: Pygame ve OpenGL ile temel bir 3D görselleştirme sınıfı içerir. Bu sınıf `main.py` tarafından aktif olarak çağrılmaz ve mevcut durumda optimizasyon sonuçlarını göstermek için kullanılmaz.

### `src/main.py`
*   `load_data_from_file()`: `data/veri_seti.txt` dosyasını ayrıştırır.
*   `reset_scenario_state()`: Algoritmaların taze veri kopyaları üzerinde çalışmasını sağlamak için yardımcı fonksiyon.
*   `analyze_and_print_results()`: Algoritma performansını görüntülemek için yardımcı fonksiyon.
*   `run_scenario()`: Veri yüklemeyi, hem Açgözlü hem de Genetik algoritmaları çalıştırmayı ve ilgili sonuçlarını ve görselleştirmelerini görüntülemeyi düzenler.
*   `if __name__ == "__main__":` bloğu, `run_scenario()` fonksiyonunu çağıran `main()` fonksiyonunu çağırır.

## Temel Parametreler & Yapılandırma

*   **Veri Seti**: `data/veri_seti.txt`, senaryo verilerinin birincil kaynağıdır.
*   **GA Parametreleri (`src/main.py` -> `optimizer.optimize_with_ga(...)` içinde)**:
    *   `population_size`: Her nesildeki birey sayısı.
    *   `generations`: GA'nın çalışacağı nesil sayısı.
    *   `crossover_rate`: Çaprazlama olasılığı.
    *   `mutation_rate`: Mutasyon olasılığı.
    *   `elitism_count`: Sonraki nesle taşınan en iyi birey sayısı.
*   **Güvenlik Tamponu (`src/algorithms.py` -> `DroneFleetOptimizer.__init__` içinde)**: `self.safety_buffer`, NFZ'ler etrafındaki açıklık mesafesini tanımlar.

## Potansiyel Gelecekteki Geliştirmeler / Hususlar

*   **Açgözlü Algoritma Zaman Pencereleri**: Açgözlü algoritma şu anda teslimat zaman pencerelerini zorunlu kılmamaktadır. GA ile daha doğrudan bir karşılaştırma için bu eklenebilir.
*   **GA Uygunluk Fonksiyonu - Öncelik**: Yüksek değerli hedefleri optimize etmek isteniyorsa, GA uygunluk fonksiyonu teslimat `priority`'sini açıkça içerecek şekilde değiştirilebilir.
*   **3D Görselleştirme Entegrasyonu**: `src/visualization.py` dosyasındaki `DroneSimulation3D` sınıfı, optimizasyon sonuçlarını 3D olarak görselleştirmek amacıyla `main.py` programına entegre edilebilir.
*   **Enerji Modeli İyileştirmeleri**: Gerekirse, yük, hız değişimleri vb. dikkate alan daha ayrıntılı bir enerji modeli uygulanabilir.
*   **Çoklu Tur**: Problem kapsamı, drone'ların üsse dönüp birden fazla tur yapmasını içeriyorsa, bu önemli bir özellik eklemesi olacaktır (mevcut durumda şarj özelliği bulunmamaktadır).
*   **Yapılandırma Dosyası**: GA parametrelerinin, dosya yollarının vb. daha kolay yönetimi için özel bir yapılandırma dosyası kullanılabilir.
*   **Birim Testleri**: `tests/` dizinini tüm modüller için kapsamlı birim testleriyle genişletmek sağlamlığı artıracaktır.