from typing import List, Tuple, Dict, Set, Optional
import heapq
import math
from dataclasses import dataclass
from src.models import Drone, DeliveryPoint, NoFlyZone
import networkx as nx

@dataclass
class Node:
    """A* algoritması için düğüm sınıfı"""
    position: Tuple[float, float]
    g_cost: float  # Başlangıç noktasından bu noktaya olan maliyet
    h_cost: float  # Bu noktadan hedef noktaya olan tahmini maliyet
    parent: Optional['Node'] = None
    
    @property
    def f_cost(self) -> float:
        """Toplam maliyet (g_cost + h_cost)"""
        return self.g_cost + self.h_cost
    
    def __lt__(self, other: 'Node') -> bool:
        """Heap için karşılaştırma operatörü"""
        return self.f_cost < other.f_cost

class DroneFleetOptimizer:
    """Drone filosu optimizasyonu için ana sınıf"""
    
    def __init__(self, drones: List[Drone], delivery_points: List[DeliveryPoint], no_fly_zones: List[NoFlyZone]):
        self.drones = drones
        self.delivery_points = delivery_points
        self.no_fly_zones = no_fly_zones
        self.grid_size = 100.0
        self.grid_resolution = 2.0  # Daha da kaba grid (Hızlandırma için)
        self.safety_buffer = 3.0  # Güvenlik mesafesi azaltıldı (9.0'dan 3.0'a test için)
        
    def _euclidean_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """İki nokta arasındaki Öklid mesafesini hesaplar"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def _heuristic(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """Hedef noktaya olan tahmini mesafeyi hesaplar"""
        # Öklid mesafesi kullanarak daha doğal rotalar elde et
        return self._euclidean_distance(pos1, pos2)

    # --- Polygon Helper Functions ---
    def _get_polygon_bounding_box(self, polygon_vertices: List[Tuple[float, float]]) -> Tuple[float, float, float, float]:
        """Bir poligonun sınırlayıcı kutusunu (min_x, min_y, max_x, max_y) döndürür."""
        if not polygon_vertices:
            return (0, 0, 0, 0)
        min_x = min(v[0] for v in polygon_vertices)
        min_y = min(v[1] for v in polygon_vertices)
        max_x = max(v[0] for v in polygon_vertices)
        max_y = max(v[1] for v in polygon_vertices)
        return min_x, min_y, max_x, max_y

    def _point_in_polygon(self, point: Tuple[float, float], polygon_vertices: List[Tuple[float, float]]) -> bool:
        """Bir noktanın çokgen içinde olup olmadığını kontrol eder (Ray Casting Algoritması)."""
        x, y = point
        n = len(polygon_vertices)
        inside = False
        p1x, p1y = polygon_vertices[0]
        for i in range(n + 1):
            p2x, p2y = polygon_vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def _distance_point_to_segment(self, p: Tuple[float, float], a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Bir p noktasının a-b segmentine olan minimum mesafesini hesaplar."""
        px, py = p
        ax, ay = a
        bx, by = b

        seg_len_sq = (bx - ax)**2 + (by - ay)**2
        if seg_len_sq == 0: # a ve b aynı nokta
            return self._euclidean_distance(p, a)

        # Segment üzerindeki p'ye en yakın noktanın projeksiyonu
        # t = [(p-a) . (b-a)] / |b-a|^2
        t = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / seg_len_sq
        t = max(0, min(1, t)) # t'yi [0,1] aralığına kısıtla

        closest_x = ax + t * (bx - ax)
        closest_y = ay + t * (by - ay)
        
        return self._euclidean_distance(p, (closest_x, closest_y))

    def _is_in_no_fly_zone(self, position: Tuple[float, float], current_time: float, with_buffer: bool = True) -> bool:
        """
        Verilen pozisyonun herhangi bir uçuşa yasak bölgede (poligonal) olup olmadığını kontrol eder.
        with_buffer=True ise güvenlik mesafesini de hesaba katar.
        """
        for zone in self.no_fly_zones:
            is_zone_currently_relevant = (current_time == 0.0) or \
                                         (zone.start_time <= current_time <= zone.end_time)
            if is_zone_currently_relevant:
                # 1. Nokta poligonun içinde mi?
                if self._point_in_polygon(position, zone.coordinates):
                    return True
                
                # 2. Buffer kontrolü (eğer with_buffer True ise)
                if with_buffer:
                    # Bounding box ön kontrolü
                    bb_min_x, bb_min_y, bb_max_x, bb_max_y = self._get_polygon_bounding_box(zone.coordinates)
                    if (position[0] < bb_min_x - self.safety_buffer or
                        position[0] > bb_max_x + self.safety_buffer or
                        position[1] < bb_min_y - self.safety_buffer or
                        position[1] > bb_max_y + self.safety_buffer):
                        # Nokta, buffer'lı sınırlayıcı kutunun kesinlikle dışındaysa, detaylı kontrole gerek yok
                        pass # continue to next zone if this was the only check
                    else:
                        # Nokta buffer'lı BB içinde olabilir, detaylı kenar mesafesi kontrolü yap
                        min_dist_to_edge = float('inf')
                        num_vertices = len(zone.coordinates)
                        for i in range(num_vertices):
                            p1_coord = zone.coordinates[i]
                            p2_coord = zone.coordinates[(i + 1) % num_vertices]
                            dist = self._distance_point_to_segment(position, p1_coord, p2_coord)
                            min_dist_to_edge = min(min_dist_to_edge, dist)
                        
                        if min_dist_to_edge <= self.safety_buffer:
                            return True
        return False
    
    def _is_point_strictly_in_no_fly_zone(self, position: Tuple[float, float], current_time: float) -> bool:
        """Bir noktanın tam olarak no-fly zone içinde olup olmadığını kontrol eder (buffer olmadan)"""
        return self._is_in_no_fly_zone(position, current_time, with_buffer=False)
    
    def _line_intersects_circle(self, p1, p2, center, radius) -> bool: # Bu metod artık NFZ için kullanılmayacak ama başka yerlerde gerekebilir.
        """İki nokta arasındaki çizgi, bir çemberi kesiyor mu? - Geliştirilmiş versiyon"""
        (x1, y1), (x2, y2) = p1, p2
        (cx, cy) = center
        
        # Vektörleştirme
        dx = x2 - x1
        dy = y2 - y1
        
        # Çizginin uzunluğu
        line_length = math.sqrt(dx*dx + dy*dy)
        
        # Çizgi çok kısaysa doğrudan nokta kontrolü yap
        if line_length < 1e-6: # Eğer p1 ve p2 çok yakınsa, p1'in çember içinde olup olmadığına bak
            return self._euclidean_distance(p1, center) <= radius
        
        # Normalize vektör
        # dx /= line_length # Bu satırlar aşağıdaki t hesaplaması için gerekli değil, kaldırılabilir
        # dy /= line_length
        
        # Merkez noktasından çizgiye olan en kısa mesafe
        # t = [(C-P1) . (P2-P1)] / |P2-P1|^2
        # P1 = (x1,y1), P2 = (x2,y2), C = (cx,cy)
        # V_P1P2 = (x2-x1, y2-y1)
        # V_P1C = (cx-x1, cy-y1)
        # t = (V_P1C . V_P1P2) / |V_P1P2|^2
        
        dot_product = (cx - x1) * (x2 - x1) + (cy - y1) * (y2 - y1)
        t = dot_product / line_length**2 # line_length_sq kullanılmalıydı, düzeltildi.
                                         # line_length_sq = dx*dx + dy*dy (yukarıda zaten hesaplanmıştı)
                                         # t = dot_product / seg_len_sq (seg_len_sq = line_length**2)

        # En yakın nokta çizgi segmenti dışındaysa, uç noktalara olan mesafeyi kontrol et
        if t < 0:
            # closest_point = p1 # En yakın nokta p1
            return self._euclidean_distance(p1, center) <= radius
        elif t > 1: # t > line_length değil, t > 1 olmalı (normalize edilmiş t için)
                    # Eğer normalize edilmemiş t kullanılıyorsa (yukarıdaki gibi), t > line_length doğru olurdu.
                    # Ancak burada t, [0,1] aralığında olmalı segment üzerindeki projeksiyon için.
            # closest_point = p2 # En yakın nokta p2
            return self._euclidean_distance(p2, center) <= radius
        else:
            # Çizgi üzerindeki en yakın nokta
            closest_x = x1 + t * (x2 - x1)
            closest_y = y1 + t * (y2 - y1)
            closest_point = (closest_x, closest_y)
            # En yakın noktanın merkeze olan mesafesi
            distance_to_center = self._euclidean_distance(closest_point, center)
            return distance_to_center <= radius

    # --- Polygon Segment Intersection Helper Functions ---
    def _on_segment(self, p: Tuple[float, float], q: Tuple[float, float], r: Tuple[float, float]) -> bool:
        """p, q, r noktalarının doğrusal olduğu durumda q'nun pr segmenti üzerinde olup olmadığını kontrol eder."""
        return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

    def _orientation(self, p: Tuple[float, float], q: Tuple[float, float], r: Tuple[float, float]) -> int:
        """(p, q, r) sıralı üçlüsünün yönünü bulur.
        0 -> p, q, r doğrusal
        1 -> Saat yönü
        2 -> Saat yönünün tersi
        """
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0: return 0  # Doğrusal
        return 1 if val > 0 else 2  # Saat yönü veya tersi

    def _segments_intersect(self, p1: Tuple[float, float], q1: Tuple[float, float],
                            p2: Tuple[float, float], q2: Tuple[float, float]) -> bool:
        """p1q1 ve p2q2 segmentlerinin kesişip kesişmediğini kontrol eder."""
        o1 = self._orientation(p1, q1, p2)
        o2 = self._orientation(p1, q1, q2)
        o3 = self._orientation(p2, q2, p1)
        o4 = self._orientation(p2, q2, q1)

        # Genel durum
        if o1 != o2 and o3 != o4:
            return True

        # Özel durumlar (doğrusal)
        if o1 == 0 and self._on_segment(p1, p2, q1): return True
        if o2 == 0 and self._on_segment(p1, q2, q1): return True
        if o3 == 0 and self._on_segment(p2, p1, q2): return True
        if o4 == 0 and self._on_segment(p2, q1, q2): return True

        return False

    def _segment_intersects_polygon(self, p_start: Tuple[float, float], p_end: Tuple[float, float],
                                   polygon_vertices: List[Tuple[float, float]],
                                   with_buffer: bool = True) -> bool:
        """Bir p_start-p_end segmentinin bir poligonla kesişip kesişmediğini kontrol eder."""
        num_vertices = len(polygon_vertices)
        for i in range(num_vertices):
            poly_p1 = polygon_vertices[i]
            poly_p2 = polygon_vertices[(i + 1) % num_vertices]
            if self._segments_intersect(p_start, p_end, poly_p1, poly_p2):
                return True
        
        # Buffer ile kesişim (segmentin herhangi bir noktasının buffer içinde olması)
        # Bu kısım daha karmaşık ve maliyetli olabilir. Şimdilik sadece poligonun kendisiyle kesişime bakıyoruz.
        # Daha sonra eklenebilir: segment üzerindeki noktaların buffer'a olan mesafesini kontrol et.
        # Veya poligonu buffer kadar genişletip (Minkowski sum) kesişim kontrolü yap.
        # Basit bir yaklaşım: segmentin orta noktasının buffer içinde olup olmadığını kontrol et.
        if with_buffer:
            mid_point = ((p_start[0] + p_end[0]) / 2, (p_start[1] + p_end[1]) / 2)
            # _is_in_no_fly_zone zaten buffer'ı ele alıyor, current_time=0.0 (genel kontrol)
            if self._is_in_no_fly_zone(mid_point, 0.0, with_buffer=True): # current_time geçici
                 # Eğer orta nokta buffer içindeyse, daha detaylı kontrol gerekebilir.
                 # Şimdilik, eğer orta nokta buffer içindeyse kesişim var sayalım.
                 # Bu, bazı güvenli yolları eleyebilir ama daha güvenli tarafta kalır.
                 # Daha hassas kontrol için segmentin tamamının buffer dışında kaldığından emin olunmalı.
                 pass # Bu kısmı daha sonra iyileştirebiliriz.

        return False


    def _is_path_safe(self, path: List[Tuple[float, float]], current_time: float) -> bool:
        """Bir yolun tamamen güvenli olup olmadığını kontrol eder"""
        if not path or len(path) < 2:
            return True
            
        # Her nokta güvenli mi?
        for point in path:
            if self._is_in_no_fly_zone(point, current_time, with_buffer=True): # Buffer ile kontrol
                return False
                
        # Her segment güvenli mi?
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i+1]
            
            for zone in self.no_fly_zones:
                is_zone_currently_relevant = (current_time == 0.0) or \
                                             (zone.start_time <= current_time <= zone.end_time)
                if is_zone_currently_relevant:
                    # 1. Segment poligonla kesişiyor mu (buffer'sız)?
                    if self._segment_intersects_polygon(p1, p2, zone.coordinates, with_buffer=False):
                        return False # Direkt kesişim varsa güvensiz.
                    
                    # 2. Segmentin tamamı buffer dışında mı?
                    #    Bunu kontrol etmek için segment üzerindeki örnek noktaların
                    #    veya segmentin kendisine en yakın mesafenin buffer dışında olduğunu doğrulamak gerekir.
                    #    Şimdilik, _is_in_no_fly_zone uç noktaları kontrol etti.
                    #    Segmentin kendisi için daha hassas buffer kontrolü eklenebilir.
                    #    Örneğin, segmentin orta noktasının buffer içinde olup olmadığına bakılabilir
                    #    veya segmentin kenarlara olan mesafesi kontrol edilebilir.
                    #    Basitleştirilmiş yaklaşım: Eğer segment poligonla kesişmiyorsa
                    #    ve uç noktalar buffer dışında ise, şimdilik güvenli kabul edelim.
                    #    Daha sağlam bir çözüm, segmentin buffer'lı poligona olan mesafesini kontrol etmektir.
                    #    Veya segment üzerindeki çok sayıda noktanın buffer dışında olduğunu kontrol etmek.
                    
                    # Hassas örnekleme ile buffer kontrolü (eski _is_path_safe'den uyarlanmış)
                    dist = self._euclidean_distance(p1, p2)
                    if dist > 1e-6: # Sadece anlamlı segmentler için
                        # Örnekleme adım sayısını minimuma indir (Hızlandırma için)
                        steps = max(1, int(dist / (self.grid_resolution * 10))) # Çok daha az adım
                        if steps > 3: # Çok uzun segmentler için üst sınırı daha da düşür
                            steps = 3
                        for step_idx in range(1, steps): # Uç noktalar zaten kontrol edildi
                            t = step_idx / steps
                            x = p1[0] + t * (p2[0] - p1[0])
                            y = p1[1] + t * (p2[1] - p1[1])
                            intermediate_point = (x, y)
                            if self._is_in_no_fly_zone(intermediate_point, current_time, with_buffer=True):
                                return False
        
        return True
    
    def _get_neighbors(self, node: Node, goal: Tuple[float, float], current_time: float) -> List[Node]:
        """Verilen düğümün komşularını döndürür"""
        neighbors = []
        x, y = node.position
        
        # 16 yönlü hareket (hızlandırma için yön sayısını azalt)
        angles = [i * (360/16) for i in range(16)]
        
        for angle in angles:
            rad = math.radians(angle)
            dx = math.cos(rad) * self.grid_resolution
            dy = math.sin(rad) * self.grid_resolution
            
            new_x = x + dx
            new_y = y + dy
            new_pos = (new_x, new_y)
            
            # Grid sınırları ve güvenlik kontrolü
            if (0 <= new_x <= self.grid_size and 0 <= new_y <= self.grid_size and 
                not self._is_in_no_fly_zone(new_pos, current_time, with_buffer=True)):
                
                # Poligonal NFZ'ler için uzaklık ve ceza mantığı güncellenmeli.
                # Şimdilik, _is_in_no_fly_zone zaten buffer'ı kontrol ediyor.
                # Ceza için, noktanın en yakın NFZ poligonuna olan mesafesi kullanılabilir.
                
                # Basitleştirilmiş ceza: Eğer nokta buffer'a çok yakınsa (örneğin buffer'ın yarısı içinde)
                # Bu kısım daha sonra detaylandırılabilir. Şimdilik ceza uygulamayalım veya basit tutalım.
                
                move_cost = self._euclidean_distance(node.position, new_pos)
                
                # Yakınlık cezası için (opsiyonel, daha sonra eklenebilir):
                # min_dist_to_any_nfz_edge = float('inf')
                # for zone_penalty in self.no_fly_zones:
                #     if zone_penalty.start_time <= current_time <= zone_penalty.end_time:
                #         num_vertices_penalty = len(zone_penalty.coordinates)
                #         for i_penalty in range(num_vertices_penalty):
                #             p1_penalty = zone_penalty.coordinates[i_penalty]
                #             p2_penalty = zone_penalty.coordinates[(i_penalty + 1) % num_vertices_penalty]
                #             dist_edge = self._distance_point_to_segment(new_pos, p1_penalty, p2_penalty)
                #             min_dist_to_any_nfz_edge = min(min_dist_to_any_nfz_edge, dist_edge)
                #
                # if min_dist_to_any_nfz_edge < self.safety_buffer * 0.5: # Buffer'ın yarısından daha yakınsa
                #     move_cost *= 3.0 # Ceza uygula
                # elif min_dist_to_any_nfz_edge < self.safety_buffer: # Buffer içindeyse ama çok yakın değilse
                #     move_cost *= 1.5

                new_g_cost = node.g_cost + move_cost
                new_h_cost = self._heuristic(new_pos, goal)
                
                neighbors.append(Node(
                    position=new_pos,
                    g_cost=new_g_cost,
                    h_cost=new_h_cost,
                    parent=node
                ))
        
        return neighbors
    
    def find_path(self, start: Tuple[float, float], goal: Tuple[float, float],
                 current_time: float, max_iterations: int = 2000) -> Optional[List[Tuple[float, float]]]: # max_iterations düşürüldü
        """A* algoritması ile başlangıç ve hedef noktalar arasında yol bulur"""
        # Öncelikle başlangıç ve hedef noktaların güvenli olup olmadığını kontrol et
        if self._is_in_no_fly_zone(start, current_time, with_buffer=True) or self._is_in_no_fly_zone(goal, current_time, with_buffer=True):
            print(f"[A*] Başlangıç veya hedef nokta no-fly zone içinde veya çok yakın!")
            return None

        # Hedef nokta no-fly zone içinde mi?
        if self._is_point_strictly_in_no_fly_zone(goal, current_time):
            print(f"[A*] Hedef nokta D{goal} no-fly zone içinde, rota planlanamıyor!")
            return None
        
        # İlk önce direkt yolu dene
        direct_path = [start, goal]
        if self._is_path_safe(direct_path, current_time):
            return direct_path
        
        print(f"[A*] Yol aranıyor: {start} -> {goal}")
        
        # A* algoritması
        start_node = Node(
            position=start,
            g_cost=0,
            h_cost=self._heuristic(start, goal)
        )
        
        open_set = []
        heapq.heappush(open_set, start_node)
        closed_set = set()
        cost_so_far = {start: 0}
        
        iteration = 0
        while open_set and iteration < max_iterations:
            current = heapq.heappop(open_set)
            
            # Hedefe ulaştık mı?
            if self._euclidean_distance(current.position, goal) < self.grid_resolution:
                path = []
                while current:
                    path.append(current.position)
                    current = current.parent
                path = path[::-1]
                
                # Son bir güvenlik kontrolü
                if self._is_path_safe(path, current_time):
                    print(f"[A*] Güvenli yol bulundu, adım sayısı: {len(path)}")
                    return path
                else:
                    print(f"[A*] Bulunan yol güvenli değil, aramaya devam ediliyor...")
            
            # Düğümü ziyaret edildi olarak işaretle
            closed_set.add(current.position)
            
            # Komşuları kontrol et
            for neighbor in self._get_neighbors(current, goal, current_time):
                if neighbor.position in closed_set:
                    continue
                
                new_cost = current.g_cost + self._euclidean_distance(current.position, neighbor.position)
                
                if neighbor.position not in cost_so_far or new_cost < cost_so_far[neighbor.position]:
                    cost_so_far[neighbor.position] = new_cost
                    heapq.heappush(open_set, neighbor)
            
            iteration += 1
            if iteration % 500 == 0:
                print(f"[A*] Iterasyon: {iteration}, open_set: {len(open_set)}")
        
        print(f"[A*] Yol bulunamadı veya iterasyon limiti aşıldı.")
        return None
    
    def _sample_no_fly_zone_points(self, zone: NoFlyZone, num_points: int = 128) -> List[Tuple[float, float]]:
        """No-fly zone (poligonal) etrafında daha fazla örnek nokta oluşturur.
           Bu fonksiyonun poligonal NFZ'ler için yeniden tasarlanması gerekir.
           Mevcut hali dairesel NFZ'lere göre çalışır.
           Şimdilik poligonun köşe noktalarını ve kenarlarının orta noktalarını döndürebiliriz.
        """
        # TODO: Poligonal NFZ'ler için bu fonksiyonu yeniden tasarla.
        # Geçici olarak köşe noktalarını ve buffer kadar dışındaki noktaları döndür.
        sampled_points = []
        # Köşe noktalarını ekle (buffer dışında olmaları için kontrol edilebilir)
        # for vertex in zone.coordinates:
        #     # Basit bir dışa öteleme (normal vektör yönünde)
        #     # Bu kısım karmaşık, şimdilik sadece köşeleri kullanalım veya daha basit bir strateji
        #     pass # Bu fonksiyonun kullanımı gözden geçirilmeli

        # print(f"[Warning] _sample_no_fly_zone_points poligonal NFZ'ler için tam olarak implemente edilmedi.")
        # Basitçe poligonun köşe noktalarını döndür
        # Veya poligonun dışına buffer kadar ötelenmiş köşeler. Bu da karmaşık.
        # Şimdilik, visibility graph bu fonksiyonu kullanıyorsa, bu graph'ın doğruluğu etkilenebilir.
        # En basit haliyle, poligonun köşe noktalarını döndürelim.
        # Ancak bu noktalar buffer içinde olabilir.
        # Güvenli olması için, buffer kadar dışarıda olmaları gerekir.
        # Bu fonksiyonun `find_visibility_path` içindeki kullanımı gözden geçirilmeli.
        # Şimdilik boş liste döndürelim veya bu fonksiyonun çağrıldığı yerleri güncelleyelim.
        return [] # Geçici olarak boş döndürülüyor.

    def build_visibility_graph(self, points: List[Tuple[float, float]], current_time: float) -> nx.Graph:
        """Visibility graph oluşturur"""
        G = nx.Graph()
        
        # Tüm noktaları grafa ekle
        for point in points:
            G.add_node(point)
        
        # Güvenli kenarları ekle
        for i, p1 in enumerate(points):
            for j, p2 in enumerate(points[i+1:], i+1):
                # Direkt yol güvenli mi?
                if p1 != p2 and not self._is_in_no_fly_zone(p1, current_time) and not self._is_in_no_fly_zone(p2, current_time):
                    # P1 ve P2 arasındaki çizgi hiçbir no-fly zone ile kesişiyor mu?
                    path_safe = True
                    for zone in self.no_fly_zones:
                        if zone.start_time <= current_time <= zone.end_time:
                            # Poligonal kesişim kontrolü
                            if self._segment_intersects_polygon(p1, p2, zone.coordinates, with_buffer=True):
                                # with_buffer=True burada segmentin buffer'lı poligona girip girmediğini kontrol etmeli.
                                # _segment_intersects_polygon'un buffer mantığı henüz tam değil.
                                # Şimdilik, _is_path_safe daha detaylı kontrol yapıyor.
                                # Burada sadece ham poligonla kesişime bakabiliriz,
                                # ve _is_path_safe'in daha sonra detaylı buffer kontrolü yapmasına güvenebiliriz.
                                # Veya _segment_intersects_polygon'a daha iyi buffer mantığı eklemeliyiz.
                                # Şimdilik, _is_path_safe'e benzer bir mantık kullanalım:
                                temp_path_for_check = [p1, p2]
                                if not self._is_path_safe(temp_path_for_check, current_time):
                                    path_safe = False
                                    break
                    
                    if path_safe:
                        distance = self._euclidean_distance(p1, p2)
                        G.add_edge(p1, p2, weight=distance)
        
        return G
    
    def find_visibility_path(self, start: Tuple[float, float], goal: Tuple[float, float],
                           current_time: float) -> Optional[List[Tuple[float, float]]]:
        """Visibility graph kullanarak iki nokta arasında yol bulur"""
        # Başlangıç ve hedef noktaların güvenli (buffer dahil) olup olmadığını kontrol et
        if self._is_in_no_fly_zone(start, current_time, with_buffer=True) or self._is_in_no_fly_zone(goal, current_time, with_buffer=True):
            print(f"[Visibility] Başlangıç veya hedef nokta D{goal} no-fly zone buffer'ı içinde veya çok yakın!")
            return None
        
        # Hedef nokta no-fly zone içinde mi (buffer olmadan)?
        if self._is_point_strictly_in_no_fly_zone(goal, current_time):
            print(f"[Visibility] Hedef nokta D{goal} no-fly zone içinde, rota planlanamıyor!")
            return None
        
        # Direkt yolu dene
        direct_path = [start, goal]
        if self._is_path_safe(direct_path, current_time):
            return direct_path

        # Visibility graph için noktalar
        graph_points = [start, goal]
        
        # No-fly zone'ları kontrol et ve gerekli noktaları ekle
        for zone in self.no_fly_zones:
            if zone.start_time <= current_time <= zone.end_time:
                # Bu no-fly zone yol üzerinde mi?
                # Poligonal NFZ için bu yol üzerinde mi kontrolü güncellenmeli.
                # _segment_intersects_polygon kullanılabilir.
                # _sample_no_fly_zone_points da poligonal için güncellenmeli.
                # Bu kısım şimdilik devre dışı bırakılabilir veya basitleştirilebilir.
                # Visibility graph'ın doğruluğu için bu adımlar önemli.
                # Şimdilik, _sample_no_fly_zone_points boş döndüğü için bu blok etkisiz olacak.
                # TODO: Bu kısmı poligonal NFZ'ler için yeniden ele al.
                # if self._segment_intersects_polygon(start, goal, zone.coordinates, with_buffer=True): # Buffer'lı kontrol
                #     zone_points = self._sample_no_fly_zone_points(zone, num_points=32) # Daha az nokta
                #     for point in zone_points:
                #         if not self._is_in_no_fly_zone(point, current_time, with_buffer=True):
                #             # Diğer NFZ'lerle çakışma kontrolü (poligonal)
                #             is_point_safe_from_other_zones = True
                #             for other_zone in self.no_fly_zones:
                #                 if other_zone.id != zone.id and \
                #                    other_zone.start_time <= current_time <= other_zone.end_time:
                #                    if self._point_in_polygon(point, other_zone.coordinates) or \
                #                       self._distance_point_to_segment_for_all_edges(point, other_zone.coordinates) <= self.safety_buffer:
                #                         is_point_safe_from_other_zones = False
                #                         break
                #             if is_point_safe_from_other_zones:
                #                 graph_points.append(point)
                pass # Geçici olarak bu bloğu atla

        # Visibility graph oluştur
        G = nx.Graph()
        
        # Noktaları grafa ekle
        for point in graph_points:
            G.add_node(point)
        
        # Güvenli kenarları ekle
        for i, p1 in enumerate(graph_points):
            for j, p2 in enumerate(graph_points[i+1:], i+1):
                # Aynı nokta değilse ve her iki nokta da güvenliyse
                if p1 != p2 and not self._is_in_no_fly_zone(p1, current_time, with_buffer=True) and not self._is_in_no_fly_zone(p2, current_time, with_buffer=True):
                    # İki nokta arasındaki yol güvenli mi?
                    path_safe = True
                    
                    # Her no-fly zone için kontrol
                    for zone_check in self.no_fly_zones: # Renamed 'zone' to 'zone_check' to avoid conflict if outer loop uses 'zone'
                        if zone_check.start_time <= current_time <= zone_check.end_time:
                            # Use the more robust _is_path_safe for polygonal NFZ check
                            temp_path_for_check = [p1, p2]
                            if not self._is_path_safe(temp_path_for_check, current_time):
                                path_safe = False
                                break
                    
                    if path_safe:
                        # Yol güvenli, kenarı ekle
                        distance = self._euclidean_distance(p1, p2)
                        # No-fly zone'lara yakınlık cezası
                        min_distance_to_nfz = float('inf')
                        for zone in self.no_fly_zones:
                            if zone.start_time <= current_time <= zone.end_time:
                                mid_point = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)
                                dist = self._euclidean_distance(mid_point, zone.center) - zone.radius
                                min_distance_to_nfz = min(min_distance_to_nfz, dist)
                        
                        if min_distance_to_nfz < self.safety_buffer * 2.0: # Önceki etkili menzil
                            distance *= 2.0  # Ceza miktarı
                        
                        G.add_edge(p1, p2, weight=distance)
        
        try:
            # Dijkstra algoritması ile en kısa yolu bul
            path = nx.shortest_path(G, source=start, target=goal, weight='weight')
            
            # Son bir güvenlik kontrolü
            if self._is_path_safe(path, current_time):
                return path
            else:
                print(f"[Visibility] Bulunan yol güvenli değil!")
                return None
                
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            print(f"[Visibility] Yol bulunamadı!")
            return None
    
    def filter_delivery_points(self, current_time: float) -> List[DeliveryPoint]:
        """No-fly zone içindeki teslimat noktalarını filtreler"""
        safe_delivery_points = []
        skipped_points = []
        
        for point in self.delivery_points:
            if not point.is_delivered:
                # Teslimat noktası kesin olarak bir no-fly zone içindeyse atla (buffer olmadan)
                if self._is_point_strictly_in_no_fly_zone(point.position, current_time):
                    skipped_points.append(point.id)
                else:
                    safe_delivery_points.append(point)
        
        if skipped_points:
            print(f"[Filtre] No-fly zone içindeki teslimat noktaları atlandı: {skipped_points}")
        
        return safe_delivery_points
    
    def optimize(self) -> Dict[int, List[Tuple[float, float]]]:
        """Drone filosu için optimum rotalar bulur - Geliştirilmiş versiyon"""
        print("[OPTIMIZE] Drone filosu optimizasyonu başlatıldı.")
        routes = {drone.id: [] for drone in self.drones}
        current_time = 0.0
        
        # Drone durumlarını takip et
        drone_states = {drone.id: {
            'position': drone.position,
            'energy': drone.current_energy,
            'weight': 0,  # Başlangıçta yük 0
            'route': []
        } for drone in self.drones}

        # Teslimat noktalarını filtrele (buffer dahil)
        deliveries_left = self.filter_delivery_points(current_time)
        
        print(f"[INFO] Toplam güvenli teslimat noktası sayısı: {len(deliveries_left)}")
        
        while deliveries_left:
            progress = False
            
            for drone in self.drones:
                state = drone_states[drone.id]
                
                if state['energy'] <= 0:
                    continue  # Enerjisi biten drone'u atla
                    
                # En yakın teslimat noktasını bul
                best_delivery = None
                best_path = None
                min_distance = float('inf')
                
                for delivery in deliveries_left:
                    # Ağırlık kontrolü
                    if state['weight'] + delivery.weight > drone.max_weight:
                        continue
                    
                    # Hedef noktanın güvenliğini tekrar kontrol et
                    if self._is_in_no_fly_zone(delivery.position, current_time, with_buffer=True):
                        continue
                    
                    # Yol bulma stratejileri
                    path = None
                    
                    # 1. Önce direkt yolu dene
                    direct_path = [state['position'], delivery.position]
                    if self._is_path_safe(direct_path, current_time):
                        path = direct_path
                    
                    # 2. Direkt yol güvenli değilse, no-fly zone'ları kontrol et
                    if not path:
                        # Poligonal NFZ'ler için _line_intersects_circle ve plan_path_around_no_fly_zone
                        # doğrudan uygulanamaz. Bu stratejinin yeniden düşünülmesi gerekir.
                        # Şimdilik, bu özel planlama adımını atlayıp doğrudan visibility graph veya A*'a geçebiliriz.
                        # TODO: Poligonal NFZ'ler için özel kaçınma stratejisi geliştir.
                        # for zone_opt in self.no_fly_zones:
                        #     if zone_opt.start_time <= current_time <= zone_opt.end_time:
                        #         # Eğer direkt yol bir NFZ poligonu ile kesişiyorsa (buffer'lı)
                        #         if self._segment_intersects_polygon(state['position'], delivery.position, zone_opt.coordinates, with_buffer=True):
                        #             # Poligonal NFZ etrafında yol planlama fonksiyonu çağrılmalı (henüz yok)
                        #             # path = self.plan_path_around_polygonal_nfz(...)
                        #             # if path: break
                        #             pass # Şimdilik bu özel planlama yok.
                        pass

                    # 3. No-fly zone etrafında planlama başarısız olduysa visibility graph dene
                    if not path:
                        path = self.find_visibility_path(state['position'], delivery.position, current_time)
                    
                    # 4. Son çare olarak A* dene
                    if not path:
                        path = self.find_path(state['position'], delivery.position, current_time)
                    
                    if path:
                        # Son bir kez yolun güvenli olduğunu kontrol et
                        if self._is_path_safe(path, current_time):
                            # Yol uzunluğunu hesapla
                            path_length = sum(self._euclidean_distance(path[i], path[i+1]) 
                                            for i in range(len(path)-1))
                            
                            # Enerji kontrolü
                            if path_length <= state['energy']:
                                if path_length < min_distance:
                                    min_distance = path_length
                                    best_delivery = delivery
                                    best_path = path
                
                # En iyi teslimat var mı?
                if best_delivery and best_path:
                    path_length = sum(self._euclidean_distance(best_path[i], best_path[i+1]) 
                                    for i in range(len(best_path)-1))
                    
                    print(f"[OPTIMIZE] Drone {drone.id} -> Teslimat D{best_delivery.id} " + 
                          f"(mesafe: {path_length:.2f}, kalan enerji: {state['energy']:.2f})")
                    
                    # Drone durumunu güncelle
                    state['route'].extend(best_path[1:])  # İlk noktayı atlayarak ekle
                    state['position'] = best_delivery.position
                    state['energy'] -= path_length
                    state['weight'] += best_delivery.weight
                    
                    # Teslimat tamamlandı
                    best_delivery.is_delivered = True
                    deliveries_left.remove(best_delivery)
                    
                    progress = True
            
            # Hiçbir drone ilerleme kaydedemediyse döngüden çık
            if not progress:
                print("[OPTIMIZE] Hiçbir drone daha fazla teslimat yapamıyor.")
                break
        
        # Nihai rotaları oluştur
        for drone in self.drones:
            state = drone_states[drone.id]
            if state['route']:
                routes[drone.id] = [drone.position] + state['route']  # Başlangıç noktasını ekle
        
        # Teslim edilemeyen noktaları raporla
        undelivered = [dp.id for dp in self.delivery_points if not dp.is_delivered]
        if undelivered:
            print(f"[OPTIMIZE] Teslim edilemeyen noktalar: {undelivered}")
        
        print("[OPTIMIZE] Optimizasyon tamamlandı.")
        return routes

    def plan_path_around_no_fly_zone(self, start: Tuple[float, float], goal: Tuple[float, float],
                                   zone: NoFlyZone, current_time: float) -> Optional[List[Tuple[float, float]]]:
        """No-fly zone (poligonal) etrafında özel olarak yol planlar.
           Bu fonksiyonun poligonal NFZ'ler için tamamen yeniden tasarlanması gerekir.
           Mevcut hali dairesel NFZ'lere göre çalışır ve artık geçerli değildir.
        """
        # print(f"[Warning] plan_path_around_no_fly_zone poligonal NFZ'ler için çağrıldı ama implementasyon dairesel.")
        # TODO: Poligonal NFZ'ler için bu fonksiyonu yeniden tasarla veya kaldır.
        #       Örneğin, poligonun teğet noktalarını veya visibility graph'ı kullanabilir.
        # Şimdilik None döndürerek bu stratejinin kullanılmamasını sağlıyoruz.
        return None