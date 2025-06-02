from typing import List, Tuple, Dict, Set, Optional
import heapq
import math
import random # Added import for random module
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
        self.grid_resolution = 0.25  # Daha hassas grid
        self.safety_buffer = 9.0  # Güvenlik mesafesi artırıldı (8.0'dan 9.0'a)
        # self.DEFAULT_ASSUMED_SPEED = 10.0  # Units of distance per unit of time - REMOVED
        
    def _euclidean_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """İki nokta arasındaki Öklid mesafesini hesaplar"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def _heuristic(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """Hedef noktaya olan tahmini mesafeyi hesaplar"""
        # Öklid mesafesi kullanarak daha doğal rotalar elde et
        return self._euclidean_distance(pos1, pos2)
    
    def _get_rectangle_bounds(self, coordinates: List[Tuple[float, float]]) -> Tuple[float, float, float, float]:
        """Verilen koordinatlardan (muhtemelen bir dikdörtgen) min/max x ve y değerlerini bulur."""
        if not coordinates:
            return 0, 0, 0, 0
        x_coords = [c[0] for c in coordinates]
        y_coords = [c[1] for c in coordinates]
        return min(x_coords), max(x_coords), min(y_coords), max(y_coords)

    def _is_in_no_fly_zone(self, position: Tuple[float, float], current_time: float, with_buffer: bool = True) -> bool:
        """
        Verilen pozisyonun herhangi bir uçuşa yasak bölgede (dikdörtgen varsayılır) olup olmadığını kontrol eder.
        with_buffer=True ise güvenlik mesafesini de hesaba katar.
        """
        px, py = position
        for zone in self.no_fly_zones:
            is_zone_currently_relevant = (current_time == 0.0) or \
                                         (zone.start_time <= current_time <= zone.end_time)
            if is_zone_currently_relevant:
                x_min, x_max, y_min, y_max = self._get_rectangle_bounds(zone.coordinates)
                
                buffer = self.safety_buffer if with_buffer else 0
                
                if (x_min - buffer <= px <= x_max + buffer and
                    y_min - buffer <= py <= y_max + buffer):
                    # Daha hassas kontrol: Eğer buffer varsa ve nokta poligonun dışında ama buffer içindeyse,
                    # bu durum da "içinde" sayılır. Eğer buffer yoksa, tam içinde olmalı.
                    if not with_buffer: # Strict check without buffer
                        if not (x_min <= px <= x_max and y_min <= py <= y_max):
                            continue # Not strictly inside this zone

                    # Check if point is inside the original polygon (for non-rectangular general polygons, a point-in-polygon test would be needed here)
                    # For axis-aligned rectangles, the buffered check is sufficient if the point is within the buffered bounds.
                    # However, for a more general polygon solution, we'd need a proper point-in-polygon test first,
                    # and then a distance-to-polygon-boundary check for the buffer.
                    # For now, assuming axis-aligned rectangles from data_veri_seti.txt, this simplified check works.
                    return True
        return False

    def _is_point_strictly_in_no_fly_zone(self, position: Tuple[float, float], current_time: float) -> bool:
        """Bir noktanın tam olarak no-fly zone (dikdörtgen varsayılır) içinde olup olmadığını kontrol eder (buffer olmadan)"""
        return self._is_in_no_fly_zone(position, current_time, with_buffer=False)

    def _on_segment(self, p: Tuple[float, float], q: Tuple[float, float], r: Tuple[float, float]) -> bool:
        """Given three colinear points p, q, r, the function checks if point q lies on line segment 'pr'"""
        if (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])):
            return True
        return False

    def _orientation(self, p: Tuple[float, float], q: Tuple[float, float], r: Tuple[float, float]) -> int:
        """Find orientation of ordered triplet (p, q, r).
        Returns:
        0 -> p, q and r are colinear
        1 -> Clockwise
        2 -> Counterclockwise
        """
        val = (q[1] - p[1]) * (r[0] - q[0]) - \
              (q[0] - p[0]) * (r[1] - q[1])
        if val == 0: return 0  # Collinear
        return 1 if val > 0 else 2  # Clockwise or Counterclockwise

    def _segments_intersect(self, p1: Tuple[float, float], q1: Tuple[float, float],
                            p2: Tuple[float, float], q2: Tuple[float, float]) -> bool:
        """Return true if line segment 'p1q1' and 'p2q2' intersect."""
        o1 = self._orientation(p1, q1, p2)
        o2 = self._orientation(p1, q1, q2)
        o3 = self._orientation(p2, q2, p1)
        o4 = self._orientation(p2, q2, q1)

        # General case
        if o1 != o2 and o3 != o4:
            return True

        # Special Cases for colinearity
        if o1 == 0 and self._on_segment(p1, p2, q1): return True
        if o2 == 0 and self._on_segment(p1, q2, q1): return True
        if o3 == 0 and self._on_segment(p2, p1, q2): return True
        if o4 == 0 and self._on_segment(p2, q1, q2): return True

        return False

    def _segment_intersects_rectangle(self, p1: Tuple[float, float], p2: Tuple[float, float],
                                      rect_coords: List[Tuple[float, float]], buffer: float) -> bool:
        """Checks if line segment p1p2 intersects with a rectangle defined by rect_coords, including a buffer."""
        if not rect_coords or len(rect_coords) < 4: # Need at least 4 points for a rectangle
             return False # Or handle error appropriately

        # Assuming rect_coords are ordered (e.g., bottom-left, bottom-right, top-right, top-left)
        # For axis-aligned rectangles from _get_rectangle_bounds:
        x_min, x_max, y_min, y_max = self._get_rectangle_bounds(rect_coords)
        
        # Apply buffer to rectangle bounds
        rx_min, rx_max, ry_min, ry_max = x_min - buffer, x_max + buffer, y_min - buffer, y_max + buffer

        # Check if segment is trivially outside the buffered bounding box of the rectangle
        if max(p1[0], p2[0]) < rx_min or min(p1[0], p2[0]) > rx_max or \
           max(p1[1], p2[1]) < ry_min or min(p1[1], p2[1]) > ry_max:
            return False

        # Check intersection with each of the four buffered rectangle edges
        # Define the 4 vertices of the buffered rectangle
        # For a general polygon, we would iterate through its edges.
        # For an axis-aligned rectangle:
        rect_edges = [
            ((rx_min, ry_min), (rx_max, ry_min)),  # Bottom
            ((rx_max, ry_min), (rx_max, ry_max)),  # Right
            ((rx_max, ry_max), (rx_min, ry_max)),  # Top
            ((rx_min, ry_max), (rx_min, ry_min))   # Left
        ]

        for r_p1, r_p2 in rect_edges:
            if self._segments_intersect(p1, p2, r_p1, r_p2):
                return True
        
        # Also check if either endpoint of the segment is inside the buffered rectangle
        # This handles cases where the segment is entirely within the rectangle
        # (The _is_in_no_fly_zone check for points in _is_path_safe already covers endpoints,
        # but this can be an additional check here if needed for this specific function's contract)
        # For now, relying on _is_path_safe to check points.
        # If point p1 or p2 is inside the buffered rectangle, it's an intersection.
        # This is implicitly covered if _is_path_safe checks points first.
        # However, if the segment is *entirely* within the (buffered) NFZ,
        # and _is_path_safe checks points first, it would have already returned False.
        # If we want this function to be standalone for segment intersection:
        if self._is_in_no_fly_zone(p1, 0.0, with_buffer=True) and self._is_in_no_fly_zone(p2, 0.0, with_buffer=True):
             # Check if points are inside the *specific* zone being tested, not just *any* NFZ
            current_zone_rect_min_x, current_zone_rect_max_x, current_zone_rect_min_y, current_zone_rect_max_y = self._get_rectangle_bounds(rect_coords)
            if (current_zone_rect_min_x - buffer <= p1[0] <= current_zone_rect_max_x + buffer and
                current_zone_rect_min_y - buffer <= p1[1] <= current_zone_rect_max_y + buffer):
                return True # p1 is inside this specific buffered zone

        return False

    def _is_path_safe(self, path: List[Tuple[float, float]], current_time: float) -> bool:
        """Bir yolun tamamen güvenli olup olmadığını kontrol eder (dikdörtgen NFZ'ler varsayılır)"""
        if not path or len(path) < 2:
            return True
            
        # Her nokta güvenli mi?
        for point in path:
            if self._is_in_no_fly_zone(point, current_time, with_buffer=True): # Check with buffer for points
                return False
                
        # Her segment güvenli mi?
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i+1]
            
            for zone in self.no_fly_zones:
                if zone.start_time <= current_time <= zone.end_time or current_time == 0.0:
                    # Segment-rectangle intersection check (with safety buffer)
                    # The safety_buffer is now applied inside _segment_intersects_rectangle
                    if self._segment_intersects_rectangle(p1, p2, zone.coordinates, self.safety_buffer):
                        return False
                    
                    # Hassas örnekleme kontrolü - DAHA DA YOĞUN ÖRNEKLEME (Test amaçlı)
                    # This might be redundant if _segment_intersects_rectangle is robust
                    # For now, keeping it to be cautious, but it adds computational cost.
                    dist = self._euclidean_distance(p1, p2)
                    # steps = max(10, int(dist / 0.2))  # Eski: Her 0.2 birimde bir örnek
                    steps = max(20, int(dist / 0.1))  # Yeni: Her 0.1 birimde bir örnek, min 20 adım
                    
                    if dist > 0: # Avoid division by zero for coincident points
                        for step_idx in range(1, steps): # Check intermediate points, not endpoints (already checked)
                            t = step_idx / steps
                            x_sample = p1[0] + t * (p2[0] - p1[0])
                            y_sample = p1[1] + t * (p2[1] - p1[1])
                            sample_point = (x_sample, y_sample)
                            
                            # Check if sample point is strictly inside the *specific* current zone (without buffer)
                            # This is to catch segments that pass through the zone without intersecting edges
                            # if the _segment_intersects_rectangle only checks edge intersections.
                            # However, a robust segment-polygon intersection should handle this.
                            # For now, this adds an extra layer of safety for the sampling part.
                            if self._is_in_no_fly_zone(sample_point, current_time, with_buffer=False): # Strict check for sample points
                                 # We need to ensure this check is against the *current* zone, not any NFZ.
                                 # This requires passing the zone to _is_in_no_fly_zone or a specialized check.
                                 # For now, if it's in *any* NFZ strictly, it's unsafe.
                                 return False
        
        return True
    
    def _get_min_dist_to_nfz_boundary(self, position: Tuple[float, float], zone: NoFlyZone) -> float:
        """Calculates the minimum distance from a point to the boundary of a rectangular NFZ."""
        px, py = position
        x_min, x_max, y_min, y_max = self._get_rectangle_bounds(zone.coordinates)

        dx = max(x_min - px, 0, px - x_max)
        dy = max(y_min - py, 0, py - y_max)
        
        if dx == 0 and dy == 0: # Point is inside or on the boundary
            # Calculate distance to closest edge if inside
            dist_to_left = px - x_min
            dist_to_right = x_max - px
            dist_to_bottom = py - y_min
            dist_to_top = y_max - py
            return -min(dist_to_left, dist_to_right, dist_to_bottom, dist_to_top) # Negative if inside

        return math.sqrt(dx*dx + dy*dy)


    def _get_neighbors(self, node: Node, goal: Tuple[float, float], current_time: float) -> List[Node]:
        """Verilen düğümün komşularını döndürür (dikdörtgen NFZ'ler varsayılır)"""
        neighbors = []
        x, y = node.position
        
        angles = [i * (360/64) for i in range(64)]
        
        for angle in angles:
            rad = math.radians(angle)
            dx = math.cos(rad) * self.grid_resolution
            dy = math.sin(rad) * self.grid_resolution
            new_pos = (x + dx, y + dy)
            
            if not (0 <= new_pos[0] <= self.grid_size and 0 <= new_pos[1] <= self.grid_size):
                continue

            # Check if the new_pos itself is inside a buffered NFZ
            if self._is_in_no_fly_zone(new_pos, current_time, with_buffer=True):
                continue

            # Check if the path segment from node.position to new_pos intersects any NFZ
            segment_safe = True
            for zone_to_check in self.no_fly_zones:
                if zone_to_check.start_time <= current_time <= zone_to_check.end_time or current_time == 0.0:
                    if self._segment_intersects_rectangle(node.position, new_pos, zone_to_check.coordinates, self.safety_buffer):
                        segment_safe = False
                        break
            if not segment_safe:
                continue
            
            min_dist_overall = float('inf')
            for zone in self.no_fly_zones:
                 if zone.start_time <= current_time <= zone.end_time or current_time == 0.0:
                    dist_to_boundary = self._get_min_dist_to_nfz_boundary(new_pos, zone)
                    # If point is inside (dist_to_boundary < 0), this check should have been caught by _is_in_no_fly_zone
                    # We are interested in positive distances to apply penalty
                    if dist_to_boundary >= 0:
                         min_dist_overall = min(min_dist_overall, dist_to_boundary)
            
            move_cost = self._euclidean_distance(node.position, new_pos)
            if min_dist_overall < self.safety_buffer * 2.0 and min_dist_overall >= self.safety_buffer : # Apply penalty if close but outside buffer
                move_cost *= 3.0
            elif min_dist_overall < self.safety_buffer:
                # This case should ideally not be reached if _is_in_no_fly_zone(with_buffer=True) works correctly
                # Or if segment check catches it. Adding high penalty as a fallback.
                move_cost *= 10.0 # Higher penalty if very close or somehow inside buffer

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
                 current_time: float, max_iterations: int = 10000) -> Optional[List[Tuple[float, float]]]:
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
            # if iteration % 500 == 0: # Temporarily disable verbose A* iteration logging
                # print(f"[A*] Iterasyon: {iteration}, open_set: {len(open_set)}")
        
        print(f"[A*] Yol bulunamadı veya iterasyon limiti aşıldı.")
        return None
    
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
                # Check points with buffer
                if p1 != p2 and not self._is_in_no_fly_zone(p1, current_time, with_buffer=True) and \
                   not self._is_in_no_fly_zone(p2, current_time, with_buffer=True):
                    path_segment_safe = True
                    for zone in self.no_fly_zones:
                        if zone.start_time <= current_time <= zone.end_time or current_time == 0.0:
                            if self._segment_intersects_rectangle(p1, p2, zone.coordinates, self.safety_buffer):
                                path_segment_safe = False
                                break
                    
                    if path_segment_safe:
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
                # Check if the direct line from start to goal intersects this zone
                if self._segment_intersects_rectangle(start, goal, zone.coordinates, self.safety_buffer * 1.5): # Increased buffer for sampling trigger
                    # No-fly zone etrafında daha fazla nokta örnekle
                    # _sample_no_fly_zone_points was based on circles, needs rework for polygons
                    # For now, this part might be less effective or needs to be adapted to polygon vertices + offset points
                    # Temporary: use polygon vertices as sample points if _sample_no_fly_zone_points is not adapted
                    # zone_points = self._sample_no_fly_zone_points(zone, num_points=128) # This needs rework
                    
                    # Alternative: Use vertices of the NFZ + some offset points
                    temp_sample_points = []
                    for coord_idx in range(len(zone.coordinates)):
                        v1 = zone.coordinates[coord_idx]
                        v2 = zone.coordinates[(coord_idx + 1) % len(zone.coordinates)] # Next vertex
                        
                        # Midpoint of edge
                        mid_edge = ((v1[0] + v2[0]) / 2, (v1[1] + v2[1]) / 2)
                        
                        # Normal vector (approx for rectangle)
                        edge_vec = (v2[0] - v1[0], v2[1] - v1[1])
                        # A simple outward normal (not normalized, direction depends on winding order)
                        # Assuming clockwise for typical screen coordinates: (dy, -dx) is outward-ish
                        # For CCW: (-dy, dx)
                        # Let's assume CCW for polygon coordinates generally.
                        norm_vec = (-(v2[1] - v1[1]), v2[0] - v1[0])
                        norm_len = math.sqrt(norm_vec[0]**2 + norm_vec[1]**2)
                        if norm_len > 0:
                            norm_vec = (norm_vec[0]/norm_len, norm_vec[1]/norm_len)
                        
                        offset_dist = self.safety_buffer * 1.5 # Sample points outside buffer
                        
                        # Point offset from midpoint of edge
                        offset_point_mid = (mid_edge[0] + norm_vec[0] * offset_dist,
                                            mid_edge[1] + norm_vec[1] * offset_dist)
                        # Points offset from vertices
                        offset_point_v1 = (v1[0] + norm_vec[0] * offset_dist,
                                           v1[1] + norm_vec[1] * offset_dist)

                        if 0 <= offset_point_mid[0] <= self.grid_size and 0 <= offset_point_mid[1] <= self.grid_size:
                             temp_sample_points.append(offset_point_mid)
                        if 0 <= offset_point_v1[0] <= self.grid_size and 0 <= offset_point_v1[1] <= self.grid_size:
                             temp_sample_points.append(offset_point_v1)

                    for point in temp_sample_points: # zone.coordinates: # Using vertices as simple sample points
                        if not self._is_in_no_fly_zone(point, current_time, with_buffer=True):
                            # Diğer no-fly zone'larla çakışma kontrolü
                            is_point_globally_safe = True
                            for other_zone in self.no_fly_zones:
                                # if other_zone != zone and (other_zone.start_time <= current_time <= other_zone.end_time or current_time == 0.0):
                                # The _is_in_no_fly_zone check above already handles this for the point itself.
                                # The critical part is ensuring the path *to* this sample point is safe.
                                # This is handled when building graph edges.
                                pass # The check _is_in_no_fly_zone(point, ...) covers this.
                            
                            # Check if path from start/goal to this sample point is safe
                            # This is complex here, better handled by graph edge creation.
                            # For now, just add the point if it's not in any NFZ.
                            graph_points.append(point)
        
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
                    for zone_check in self.no_fly_zones:
                        if zone_check.start_time <= current_time <= zone_check.end_time or current_time == 0.0:
                            if self._segment_intersects_rectangle(p1, p2, zone_check.coordinates, self.safety_buffer):
                                path_segment_safe = False
                                break
                    
                    if path_segment_safe:
                        distance = self._euclidean_distance(p1, p2)
                        min_dist_overall_segment = float('inf')
                        # Penalty for proximity of the segment (e.g., midpoint) to NFZs
                        mid_point_segment = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)
                        for zone_penalty in self.no_fly_zones:
                            if zone_penalty.start_time <= current_time <= zone_penalty.end_time or current_time == 0.0:
                                dist_to_boundary = self._get_min_dist_to_nfz_boundary(mid_point_segment, zone_penalty)
                                if dist_to_boundary >= 0: # Only consider if outside or on boundary
                                    min_dist_overall_segment = min(min_dist_overall_segment, dist_to_boundary)
                        
                        if min_dist_overall_segment < self.safety_buffer * 2.0 and min_dist_overall_segment >= self.safety_buffer:
                            distance *= 2.0
                        elif min_dist_overall_segment < self.safety_buffer:
                             distance *= 5.0 # Higher penalty if segment midpoint is very close

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
                best_candidate_delivery = None
                best_candidate_path = None
                best_candidate_path_length = float('inf') # To store the length of the best path
                min_overall_cost = float('inf') # This will be the decision driver
                
                for delivery in deliveries_left:
                    # DEBUG log from line 614 removed
                    # Ağırlık kontrolü
                    if state['weight'] + delivery.weight > drone.max_weight:
                        # DEBUG log from line 617 removed
                        continue
                    
                    # Hedef noktanın güvenliğini tekrar kontrol et
                    if self._is_in_no_fly_zone(delivery.position, current_time, with_buffer=True):
                        # DEBUG log from line 622 removed
                        continue
                    
                    # Yol bulma stratejileri
                    path = None
                    path_found_by = "None"
                    
                    # 1. Önce direkt yolu dene
                    # DEBUG log from line 630 removed
                    direct_path = [state['position'], delivery.position]
                    if self._is_path_safe(direct_path, current_time):
                        path = direct_path
                        path_found_by = "Direct"
                        # DEBUG log from line 635 removed
                    else:
                        pass
                    
                    # 2. Direkt yol güvenli değilse, no-fly zone'ları kontrol et
                    if not path:
                        # DEBUG log from line 641 removed
                        for zone_idx, zone in enumerate(self.no_fly_zones):
                            # Check if zone is active or if it's initial planning (current_time == 0.0)
                            if zone.start_time <= current_time <= zone.end_time or current_time == 0.0:
                                # DEBUG log from line 645 removed
                                if self._segment_intersects_rectangle(state['position'], delivery.position,
                                                             zone.coordinates, self.safety_buffer):
                                    # DEBUG log from line 648 removed
                                    # No-fly zone etrafında yol planla
                                    path = self.plan_path_around_no_fly_zone(
                                        state['position'], delivery.position, zone, current_time)
                                    if path:
                                        path_found_by = f"Around_NFZ_{zone.id}"
                                        # DEBUG log from line 654 removed
                                        break # Found a path around this zone, stop checking other zones for this strategy
                                    else:
                                        pass
                        if not path:
                             pass
                    
                    # 3. No-fly zone etrafında planlama başarısız olduysa visibility graph dene
                    if not path:
                        # DEBUG log from line 663 removed
                        path = self.find_visibility_path(state['position'], delivery.position, current_time)
                        if path:
                            path_found_by = "VisibilityGraph"
                            # DEBUG log from line 667 removed
                        else:
                            pass
                    
                    # 4. Son çare olarak A* dene
                    if not path:
                        # DEBUG log from line 673 removed
                        path = self.find_path(state['position'], delivery.position, current_time)
                        if path:
                            path_found_by = "A*"
                            # DEBUG log from line 677 removed
                        else:
                            pass
                    
                    if path:
                        # DEBUG log from line 682 removed
                        # Son bir kez yolun güvenli olduğunu kontrol et
                        if self._is_path_safe(path, current_time):
                            # DEBUG log from line 685 removed
                            # Yol uzunluğunu hesapla
                            path_length = sum(self._euclidean_distance(path[i], path[i+1])
                                            for i in range(len(path)-1))
                            # DEBUG log from line 689 removed
                            
                            current_delivery_cost = path_length * delivery.weight + (delivery.priority * 100)

                            # Enerji kontrolü
                            if path_length <= state['energy']:
                                # DEBUG log from line 693 removed
                                if current_delivery_cost < min_overall_cost:
                                    print(f"[OPTIMIZE] Drone {drone.id}: D{delivery.id} new candidate (cost {current_delivery_cost:.2f}, path {path_length:.2f}m).")
                                    min_overall_cost = current_delivery_cost
                                    best_candidate_delivery = delivery
                                    best_candidate_path = path
                                    best_candidate_path_length = path_length
                                # else: # Original DEBUG log for 'not better' removed
                                    # Optional: print(f"[OPTIMIZE] Drone {drone.id}: D{delivery.id} not better (cost {current_delivery_cost:.2f} >= {min_overall_cost:.2f}).")
                            # else: # Original DEBUG log for 'insufficient energy' removed
                                # Optional: print(f"[OPTIMIZE] Drone {drone.id}: Skipped D{delivery.id} - Insufficient energy (Path: {path_length:.2f}, Energy: {state['energy']:.2f}).")
                        else:
                            pass
                    else:
                        pass
                
                # En iyi teslimat var mı?
                if best_candidate_delivery and best_candidate_path: # Changed variable names
                    # path_length is now best_candidate_path_length, no need to recalculate here
                    
                    # Calculate energy left for logging *before* state update
                    energy_after_delivery = state['energy'] - best_candidate_path_length
                    print(f"[OPTIMIZE] Drone {drone.id} -> Assigning D{best_candidate_delivery.id} " +
                          f"(cost: {min_overall_cost:.2f}, path: {best_candidate_path_length:.2f}m, energy left: {energy_after_delivery:.2f})")
                    
                    # Drone durumunu güncelle
                    state['route'].extend(best_candidate_path[1:])  # İlk noktayı atlayarak ekle
                    state['position'] = best_candidate_delivery.position
                    state['energy'] -= best_candidate_path_length # Use stored length
                    # state['weight'] += best_delivery.weight # Original line - accumulates weight
                    state['weight'] = 0 # Reset weight after successful delivery (assuming one package at a time)
                    # DEBUG log from line 722 removed
                    
                    # Teslimat tamamlandı
                    best_candidate_delivery.is_delivered = True # Changed variable name
                    deliveries_left.remove(best_candidate_delivery) # Changed variable name
                    
                    progress = True
            
            # Hiçbir drone ilerleme kaydedemediyse döngüden çık
            if not progress:
                # DEBUG log from line 732 removed
                print("[OPTIMIZE] Hiçbir drone daha fazla teslimat yapamıyor.")
                break
            else:
                pass
        
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

    # --- GENETIC ALGORITHM IMPLEMENTATION STARTS HERE ---

    def _get_drone_by_id(self, drone_id: int) -> Optional[Drone]:
        for drone in self.drones:
            if drone.id == drone_id:
                return drone
        return None

    def _get_delivery_by_id(self, delivery_id: int) -> Optional[DeliveryPoint]:
        for delivery_point in self.delivery_points:
            if delivery_point.id == delivery_id:
                return delivery_point
        return None

    def _evaluate_chromosome_details(self, chromosome: Dict[int, List[int]]) -> Tuple[int, float, int]:
        """
        Evaluates a chromosome to get number of deliveries, total energy, and violations.
        Chromosome: {drone_id: [delivery_id_1, delivery_id_2, ...]}
        """
        num_actual_deliveries = 0
        total_energy_consumed = 0.0
        constraint_violations_score = 0 # Penalty score

        # Keep track of deliveries made by this chromosome to count unique ones
        uniquely_delivered_ids_in_chromosome = set()

        # Create temporary drone states for this evaluation
        temp_drone_states = {
            drone.id: {
                'position': drone.position, # Initial position
                'energy': drone.max_energy, # Start with full energy for evaluation
                'current_weight': 0.0 # Starts empty for its sequence
            } for drone in self.drones
        }
        
        # Reset is_delivered status for all delivery points for this evaluation
        # This is important because GA evaluates many chromosomes independently
        original_delivery_status = {dp.id: dp.is_delivered for dp in self.delivery_points}
        for dp in self.delivery_points:
            dp.is_delivered = False

        for drone_id, delivery_id_route in chromosome.items():
            drone = self._get_drone_by_id(drone_id)
            if not drone:
                constraint_violations_score += 1000 # Invalid drone ID in chromosome
                continue

            current_drone_pos = temp_drone_states[drone_id]['position']
            current_drone_energy = temp_drone_states[drone_id]['energy']
            drone_specific_current_time = 0.0 # Initialize time for this drone's route
            # current_drone_weight is implicitly 0 before picking up each package in the sequence

            for delivery_id in delivery_id_route:
                delivery = self._get_delivery_by_id(delivery_id)
                if not delivery:
                    constraint_violations_score += 1000 # Invalid delivery ID
                    continue # Skip to next delivery in route

                if delivery.id in uniquely_delivered_ids_in_chromosome: # Already delivered by another drone in this chromosome
                    constraint_violations_score += 500 # Penalty for duplicate delivery assignment
                    continue

                # 1. Weight Constraint Check (for the specific package)
                if delivery.weight > drone.max_weight:
                    constraint_violations_score += 1000
                    # print(f"[GA_EVAL] Drone {drone_id} cannot carry D{delivery_id} (weight {delivery.weight} > max {drone.max_weight})")
                    break # Stop this drone's route if it can't carry a package

                # 2. Rota Bulma ve Güvenlik Kontrolü (GA Hızı İçin Yaklaşım)
                # print(f"[GA_EVAL_APPROX] Drone {drone_id} path: {current_drone_pos} -> D{delivery_id} ({delivery.position})")
                
                # GENETİK ALGORİTMA HIZLANDIRMASI:
                # GA'nın evrimsel süreci boyunca (çok sayıda jenerasyon ve birey değerlendirilirken)
                # her bir rota segmenti için tam A* algoritması ile yol bulmak ve enerji hesaplamak
                # aşırı derecede yavaş olacaktır.
                # Bu nedenle, GA çalışırken fitness hesaplamasını hızlandırmak için bir YAKLAŞIM kullanılır:
                #   - Enerji Maliyeti: İki nokta arasındaki direkt Öklid mesafesi kullanılır.
                #   - NFZ İhlali: Mevcut nokta ile teslimat noktası arasındaki direkt doğru parçasının
                #                  aktif bir NFZ ile (daha küçük bir tampon bölge ile) kesişip kesişmediği kontrol edilir.
                # Bu yaklaşım, GA'nın çok daha hızlı çalışmasını sağlar.
                # En iyi çözüm bulunduktan sonra, bu çözüm için GERÇEK rotalar ve maliyetler
                # optimize_with_ga metodunun sonunda tam A* algoritması kullanılarak hesaplanır.

                path_distance = self._euclidean_distance(current_drone_pos, delivery.position)
                path_energy = path_distance # Assuming energy is proportional to distance for approximation
                
                time_for_segment = 0.0
                # drone object is already fetched and checked at the start of the loop for drone_id
                if drone and drone.speed > 0:
                    time_for_segment = path_distance / drone.speed
                elif drone and drone.speed == 0: # Handle case where speed is explicitly 0
                    time_for_segment = float('inf') # Or some very large number, effectively making it non-viable
                    constraint_violations_score += 200 # Penalize if drone has 0 speed but is assigned tasks
                # If drone is None, it's already penalized earlier.

                is_segment_safe_approx = True
                for nfz in self.no_fly_zones:
                    segment_start_time_eval = drone_specific_current_time
                    segment_end_time_eval = drone_specific_current_time + time_for_segment
                    
                    nfz_is_active_during_segment = (nfz.start_time <= segment_end_time_eval and \
                                                    nfz.end_time >= segment_start_time_eval)

                    if nfz_is_active_during_segment:
                        if self._segment_intersects_rectangle(current_drone_pos, delivery.position, nfz.coordinates, self.safety_buffer):
                            is_segment_safe_approx = False
                            break
                
                if not is_segment_safe_approx:
                    constraint_violations_score += 1500 # Higher penalty for direct NFZ violation in approximation
                    # print(f"[GA_EVAL_APPROX] Drone {drone_id} direct segment to D{delivery_id} unsafe (NFZ).")
                    break # Stop this drone's route for this chromosome

                # 3. Energy Constraint Check (based on Euclidean distance)
                if path_energy > current_drone_energy:
                    constraint_violations_score += 1000
                    # print(f"[GA_EVAL] Drone {drone_id} no energy for D{delivery_id} (path {path_energy} > remaining {current_drone_energy})")
                    break # Stop this drone's route

                # If all checks pass for this segment:
                total_energy_consumed += path_energy
                current_drone_energy -= path_energy
                current_drone_pos = delivery.position # Drone moves to delivery location
                
                # Accumulate travel time
                arrival_time_at_delivery = drone_specific_current_time + time_for_segment
                
                # 4. Time Window Constraint Check
                if arrival_time_at_delivery < delivery.time_window_start:
                    # Drone arrived early, waits until window opens
                    drone_specific_current_time = delivery.time_window_start
                elif arrival_time_at_delivery > delivery.time_window_end:
                    constraint_violations_score += 750 # Penalty for late delivery
                    # print(f"[GA_EVAL] Drone {drone_id} late for D{delivery_id} (arrival {arrival_time_at_delivery:.2f} > end {delivery.time_window_end:.2f})")
                    break # Stop this drone's route, delivery failed due to time window
                else:
                    # Arrived within the time window
                    drone_specific_current_time = arrival_time_at_delivery

                # Weight is implicitly handled as one package at a time for this model
                
                uniquely_delivered_ids_in_chromosome.add(delivery.id)
                # Mark as delivered for this chromosome's evaluation context
                # This is tricky if DeliveryPoint objects are shared.
                # For fitness, we just need the count.
                # The actual DeliveryPoint.is_delivered should only be set by the final chosen solution.
                # So, we use `uniquely_delivered_ids_in_chromosome` for counting.

        num_actual_deliveries = len(uniquely_delivered_ids_in_chromosome)
        
        # Restore original delivery statuses
        for dp_id, status in original_delivery_status.items():
            dp = self._get_delivery_by_id(dp_id)
            if dp:
                dp.is_delivered = status
                
        return num_actual_deliveries, total_energy_consumed, constraint_violations_score

    def _calculate_fitness(self, chromosome: Dict[int, List[int]]) -> float:
        """
        Calculates fitness based on PDF:
        Fitness = (teslimat sayısı * 50) - (toplam enerji * 0.1) - (ihlal edilen kısıt * 1000)
        """
        num_deliveries, total_energy, violations = self._evaluate_chromosome_details(chromosome) # Removed current_time=0.0
        
        fitness = (num_deliveries * 50.0) - (total_energy * 0.1) - (violations * 1000.0)
        # print(f"[GA_FITNESS] Chromosome: {chromosome}, Deliveries: {num_deliveries}, Energy: {total_energy:.2f}, Violations: {violations}, Fitness: {fitness:.2f}")
        return fitness

    def _initialize_population(self, population_size: int) -> List[Dict[int, List[int]]]:
        population = []
        # Get all delivery points not strictly in an NFZ (initial filter)
        available_globally = [dp for dp in self.delivery_points if not self._is_point_strictly_in_no_fly_zone(dp.position, 0.0)]
        
        for i in range(population_size):
            chromosome: Dict[int, List[int]] = {drone.id: [] for drone in self.drones}
            
            # Create a fresh, shuffled list of available delivery IDs for this chromosome
            # This ensures each chromosome generation attempt has all potential deliveries available
            current_available_delivery_ids = [dp.id for dp in available_globally]
            random.shuffle(current_available_delivery_ids)
            
            assigned_deliveries_in_chromosome = set()

            for drone_id in chromosome.keys(): # Iterate through drones (can be shuffled too for more randomness)
                drone = self._get_drone_by_id(drone_id)
                if not drone: continue

                # Decide how many deliveries this drone will attempt in this chromosome (e.g., 0 to 3)
                num_deliveries_for_drone = random.randint(0, min(3, len(current_available_delivery_ids)))
                
                temp_drone_current_weight = 0.0 # Track weight for this drone's sequence in this chromosome

                for _ in range(num_deliveries_for_drone):
                    if not current_available_delivery_ids: break # No more deliveries to assign

                    # Try to pick a delivery that hasn't been assigned in this chromosome yet
                    picked_delivery_id = -1
                    for potential_dp_id in current_available_delivery_ids:
                        if potential_dp_id not in assigned_deliveries_in_chromosome:
                            picked_delivery_id = potential_dp_id
                            break
                    
                    if picked_delivery_id != -1:
                        delivery = self._get_delivery_by_id(picked_delivery_id)
                        if delivery:
                            # Basic check: can drone carry this specific package (ignoring accumulated load for init simplicity)
                            if delivery.weight <= drone.max_weight:
                                chromosome[drone_id].append(delivery.id)
                                assigned_deliveries_in_chromosome.add(delivery.id)
                                current_available_delivery_ids.remove(delivery.id) # Remove from list for this chromosome
                            # else:
                                # print(f"[GA_INIT_DEBUG] Drone {drone_id} cannot carry D{delivery.id} (weight {delivery.weight} > max {drone.max_weight}) for initial assignment")
                        # else:
                            # print(f"[GA_INIT_DEBUG] Delivery ID {picked_delivery_id} not found during init.")
                    else:
                        # No unassigned deliveries left that this drone could pick
                        break
            
            # print(f"[GA_INIT_DEBUG] Chromosome {i}: {chromosome}")
            population.append(chromosome)
            
        print(f"[GA_INIT] Initialized population of size {len(population)} with potentially non-empty routes.")
        return population

    def _selection(self, population_with_fitness: List[Tuple[Dict[int, List[int]], float]], tournament_size: int = 3) -> Dict[int, List[int]]:
        """Selects one parent using tournament selection."""
        if not population_with_fitness:
            # Fallback, should not happen with a populated list
            # Return an empty chromosome structure if population is empty
            return {drone.id: [] for drone in self.drones}
        
        # Ensure tournament_size is not larger than population size
        actual_tournament_size = min(tournament_size, len(population_with_fitness))
        if actual_tournament_size == 0:
             # This case implies population_with_fitness was empty, handled above, but as a safeguard:
             return {drone.id: [] for drone in self.drones}

        tournament_contenders = random.sample(population_with_fitness, actual_tournament_size)
        
        # Sort contenders by fitness (second element of tuple), descending
        tournament_contenders.sort(key=lambda x: x[1], reverse=True)
        
        return tournament_contenders[0][0] # Return the chromosome of the best contender

    def _order_crossover_single_route(self, route1: List[int], route2: List[int]) -> Tuple[List[int], List[int]]:
        """Performs Order Crossover (OX1) on two individual drone routes (lists of delivery_ids)."""
        size = len(route1) # Assumes route1 and route2 are of the same conceptual space, even if lengths differ
        
        # Create empty offspring routes of the same conceptual size (max possible deliveries)
        # This is tricky if routes can be of variable length.
        # For OX1, it's usually applied to permutations of the same set of items.
        # Here, routes are subsets. Let's adapt.
        
        # If either route is empty, or too short for meaningful crossover, handle simply.
        if size == 0 or len(route2) == 0:
            return list(route1), list(route2) # Return copies
        if size == 1 and len(route2) == 1:
            return list(route1), list(route2)
        if size == 1: # route1 has 1, route2 has more or 1
            # Basic: one gets route1's item, other gets route2's first if different, else copy
            o1 = list(route1)
            o2 = list(route2)
            if route1[0] in route2: # if the single item is in route2, just return copies to avoid issues
                 return o1, o2
            # else, try to make them different if possible (very basic)
            # This simple OX1 adaptation for variable length, non-permutation routes is non-trivial.
            # For now, let's do a simpler per-drone route swap or a more direct combination.
            # The PDF is general: "İki rotadan yeni rota üret."
            # The current _crossover already does this at a higher level.
            # Let's refine the existing _crossover to be a bit more structured.
            # For now, the existing _crossover will be kept, and we focus on repair.
            # A true per-route OX1 for subsets is more involved.
            pass # Placeholder for more advanced single-route crossover if needed later

        # Fallback to simple copy if OX1 is too complex for current variable-length routes
        return list(route1), list(route2)


    def _crossover(self, parent1: Dict[int, List[int]], parent2: Dict[int, List[int]]) -> Tuple[Dict[int, List[int]], Dict[int, List[int]]]:
        offspring1: Dict[int, List[int]] = {drone.id: [] for drone in self.drones}
        offspring2: Dict[int, List[int]] = {drone.id: [] for drone in self.drones}

        # Strategy: For each drone, decide if its route comes from parent1 or parent2 for each offspring.
        # This is a form of uniform crossover at the drone-route assignment level.
        for drone_id_key in parent1.keys(): # Iterate through all possible drone IDs
            p1_route = parent1.get(drone_id_key, [])
            p2_route = parent2.get(drone_id_key, [])

            if random.random() < 0.5: # Offspring1 gets from Parent1, Offspring2 from Parent2 for this drone
                offspring1[drone_id_key] = list(p1_route)
                offspring2[drone_id_key] = list(p2_route)
            else: # Offspring1 gets from Parent2, Offspring2 from Parent1 for this drone
                offspring1[drone_id_key] = list(p2_route)
                offspring2[drone_id_key] = list(p1_route)
        
        # Repair step: Ensure each delivery is assigned to at most one drone in each offspring
        # This is critical. If D1 is in DroneA's route in Offspring1 and also DroneB's route in Offspring1,
        # one must be removed.
        offspring1 = self._repair_chromosome_for_unique_deliveries(offspring1)
        offspring2 = self._repair_chromosome_for_unique_deliveries(offspring2)
        
        return offspring1, offspring2

    def _repair_chromosome_for_unique_deliveries(self, chromosome: Dict[int, List[int]]) -> Dict[int, List[int]]:
        """Ensures each delivery ID appears at most once in the entire chromosome."""
        assigned_deliveries_globally = set()
        repaired_chromosome: Dict[int, List[int]] = {drone_id: [] for drone_id in chromosome.keys()}

        # Iterate through drones, perhaps in a fixed or random order
        # For simplicity, using the order of drone IDs
        for drone_id in sorted(chromosome.keys()):
            for delivery_id in chromosome[drone_id]:
                if delivery_id not in assigned_deliveries_globally:
                    repaired_chromosome[drone_id].append(delivery_id)
                    assigned_deliveries_globally.add(delivery_id)
                # else:
                    # print(f"[GA_REPAIR] Delivery {delivery_id} already assigned, removing from drone {drone_id} in this chromosome.")
        return repaired_chromosome

    def _mutation(self, chromosome: Dict[int, List[int]], mutation_rate: float) -> Dict[int, List[int]]:
        if random.random() >= mutation_rate:
            return chromosome # No mutation

        mutated_chromosome = {drone_id: list(route) for drone_id, route in chromosome.items()} # Deep copy

        mutation_type = random.choice(["swap", "insert_unassigned", "remove_delivery"]) # Add more types later if needed

        if mutation_type == "swap":
            # print("[GA_MUTATE_DEBUG] Attempting SWAP mutation")
            drone_ids_with_routes = [did for did, r in mutated_chromosome.items() if len(r) >= 2]
            if drone_ids_with_routes:
                drone_to_mutate = random.choice(drone_ids_with_routes)
                route = mutated_chromosome[drone_to_mutate]
                idx1, idx2 = random.sample(range(len(route)), 2)
                route[idx1], route[idx2] = route[idx2], route[idx1]
                # print(f"[GA_MUTATE_DEBUG] SWAP: Drone {drone_to_mutate} route now: {route}")
        
        elif mutation_type == "insert_unassigned":
            # print("[GA_MUTATE_DEBUG] Attempting INSERT_UNASSIGNED mutation")
            all_assigned_deliveries = set()
            for route in mutated_chromosome.values():
                for delivery_id in route:
                    all_assigned_deliveries.add(delivery_id)
            
            unassigned_deliveries = [
                dp.id for dp in self.delivery_points
                if dp.id not in all_assigned_deliveries and
                   not self._is_point_strictly_in_no_fly_zone(dp.position, 0.0) # Basic check
            ]
            
            if unassigned_deliveries:
                delivery_to_add = random.choice(unassigned_deliveries)
                # Pick any drone, even if it's currently idle
                # Ensure there are drones to pick from
                if list(mutated_chromosome.keys()):
                    drone_to_receive = random.choice(list(mutated_chromosome.keys()))
                    route = mutated_chromosome[drone_to_receive]
                    insert_pos = random.randint(0, len(route))
                    route.insert(insert_pos, delivery_to_add)
                    # print(f"[GA_MUTATE_DEBUG] INSERT_UNASSIGNED: Added D{delivery_to_add} to Drone {drone_to_receive} at pos {insert_pos}. Route: {route}")
                # Repair again to ensure uniqueness if this somehow creates a duplicate (shouldn't if logic is right)
                # However, the main repair is after crossover. This mutation itself should respect uniqueness.
                # The check for `dp.id not in all_assigned_deliveries` handles this.
        
        elif mutation_type == "remove_delivery":
            # print("[GA_MUTATE_DEBUG] Attempting REMOVE_DELIVERY mutation")
            drone_ids_with_routes = [did for did, r in mutated_chromosome.items() if r]
            if drone_ids_with_routes:
                drone_to_mutate = random.choice(drone_ids_with_routes)
                route = mutated_chromosome[drone_to_mutate]
                if route: # If the chosen drone actually has a route
                    remove_idx = random.randrange(len(route))
                    # removed_delivery = route.pop(remove_idx) # Keep for debug if needed
                    route.pop(remove_idx)
                    # print(f"[GA_MUTATE_DEBUG] REMOVE_DELIVERY: Removed D{removed_delivery} from Drone {drone_to_mutate}. Route: {route}")

        # After any mutation, it's good practice to ensure the chromosome is still valid
        # regarding unique delivery assignments, although our current repair step is after crossover.
        # For simplicity now, we rely on the fitness function to heavily penalize invalid states
        # that might arise from complex mutations if not handled carefully.
        # The `_repair_chromosome_for_unique_deliveries` is called after crossover, which is the main point of conflict.
        return mutated_chromosome

    # Note: The _mutation method was already a placeholder that did a swap.
    # The existing placeholder is actually a decent basic swap mutation.
    # I will ensure it's correctly placed and the random import is handled at the class/module level.
    # The previous `import random` inside _mutation was not ideal.
    # No changes needed to the _mutation logic itself if the existing placeholder is used,
    # just ensuring it's called correctly and `random` is available.
    # The `import random` is already at the top of the file.

    def optimize_with_ga(self, population_size: int = 50, generations: int = 100,
                         crossover_rate: float = 0.8, mutation_rate: float = 0.1,
                         elitism_count: int = 2) -> Dict[int, List[Tuple[float, float]]]:
        print("[OPTIMIZE_GA] Genetic Algorithm optimization started.")
        
        population = self._initialize_population(population_size)
        best_overall_fitness = -float('inf')
        best_overall_chromosome = None

        for gen in range(generations):
            fitness_scores = [self._calculate_fitness(ind) for ind in population]

            # Store population and scores together for sorting
            pop_with_fitness = list(zip(population, fitness_scores))
            pop_with_fitness.sort(key=lambda x: x[1], reverse=True) # Sort by fitness descending

            current_best_fitness = pop_with_fitness[0][1]
            if current_best_fitness > best_overall_fitness:
                best_overall_fitness = current_best_fitness
                best_overall_chromosome = pop_with_fitness[0][0]
            
            print(f"[GA_GEN {gen+1}/{generations}] Best Fitness: {current_best_fitness:.2f} (Overall Best: {best_overall_fitness:.2f})")

            new_population = []
            
            # Elitism: carry over the best individuals
            for i in range(elitism_count):
                new_population.append(pop_with_fitness[i][0])

            # Fill the rest of the population
            while len(new_population) < population_size:
                # Select parents using tournament selection
                parent1 = self._selection(pop_with_fitness, tournament_size=3)
                parent2 = self._selection(pop_with_fitness, tournament_size=3)
                
                # Ensure parent1 and parent2 are not None (which _selection might return on empty input, though unlikely here)
                # The fallback in _selection now returns an empty chromosome structure, so direct None check isn't needed,
                # but we might want to handle cases where selection consistently returns empty/poor parents.
                # For now, proceed assuming selection returns valid (though possibly empty-routed) chromosomes.

                if random.random() < crossover_rate:
                    # TODO: Implement proper crossover
                    offspring1, offspring2 = self._crossover(parent1, parent2)
                else:
                    offspring1 = {drone.id: list(parent1.get(drone.id, [])) for drone in self.drones}
                    offspring2 = {drone.id: list(parent2.get(drone.id, [])) for drone in self.drones}
                
                new_population.append(self._mutation(offspring1, mutation_rate))
                if len(new_population) < population_size:
                    new_population.append(self._mutation(offspring2, mutation_rate))
            
            population = new_population[:population_size]

        print(f"[OPTIMIZE_GA] Genetic Algorithm optimization finished after {generations} generations.")
        print(f"[OPTIMIZE_GA] Best overall fitness: {best_overall_fitness:.2f}")
        print(f"[OPTIMIZE_GA] Best chromosome: {best_overall_chromosome}")

        # Convert the best chromosome to the final route format
        # This involves generating the actual paths (list of coordinates) for the sequence of deliveries
        final_routes: Dict[int, List[Tuple[float, float]]] = {drone.id: [] for drone in self.drones}
        if best_overall_chromosome:
            # Reset delivery states before final path generation for the chosen solution
            for dp in self.delivery_points:
                dp.is_delivered = False

            temp_drone_states_final = {
                drone.id: {
                    'position': drone.position,
                    'energy': drone.max_energy,
                    'current_weight': 0.0
                } for drone in self.drones
            }

            for drone_id, delivery_id_route in best_overall_chromosome.items():
                drone = self._get_drone_by_id(drone_id)
                if not drone: continue

                current_pos = temp_drone_states_final[drone_id]['position']
                current_energy = temp_drone_states_final[drone_id]['energy']
                actual_path_for_drone = [current_pos] # Start with drone's initial position
                drone_specific_current_time_final = 0.0 # Initialize time for this drone's final route

                for delivery_id in delivery_id_route:
                    delivery = self._get_delivery_by_id(delivery_id)
                    if not delivery: continue
                    
                    # Check if already delivered by another drone in this solution (should be handled by GA logic ideally)
                    # For final output, we ensure it's marked delivered only once.
                    if delivery.is_delivered: continue

                    if delivery.weight > drone.max_weight: break # Should have been caught by fitness
                    
                    # Use drone_specific_current_time_final for pathfinding and safety checks
                    path_segments = self.find_path(current_pos, delivery.position, drone_specific_current_time_final)
                    if not path_segments or not self._is_path_safe(path_segments, drone_specific_current_time_final):
                        # print(f"[GA_FINAL_PATH] Path not found or unsafe for D{delivery_id} at time {drone_specific_current_time_final:.2f}")
                        break # Path issue, should have been penalized by fitness

                    path_distance = sum(self._euclidean_distance(path_segments[i], path_segments[i+1]) for i in range(len(path_segments)-1))
                    # Assuming path_energy is equivalent to path_distance for this calculation step
                    path_energy = path_distance

                    if path_energy > current_energy:
                        # print(f"[GA_FINAL_PATH] Insufficient energy for D{delivery_id} (Path: {path_energy:.2f}, Energy: {current_energy:.2f})")
                        break # Energy issue, should have been penalized
                    
                    time_for_segment_actual = 0.0
                    # drone object is available in this scope
                    if drone.speed > 0:
                        time_for_segment_actual = path_distance / drone.speed
                    elif drone.speed == 0: # Drone with 0 speed cannot move
                        time_for_segment_actual = float('inf') # Path would take infinite time
                        # This should ideally be caught by fitness or earlier checks,
                        # but as a safeguard in final pathing.
                        # print(f"[GA_FINAL_PATH] Drone {drone.id} has 0 speed, cannot complete path to D{delivery.id}")
                        break # Stop processing this drone's route

                    actual_path_for_drone.extend(path_segments[1:])
                    current_pos = delivery.position
                    current_energy -= path_energy
                    
                    # Accumulate travel time for this segment
                    arrival_time_at_delivery_final = drone_specific_current_time_final + time_for_segment_actual
                    
                    # Check time window for final path
                    if arrival_time_at_delivery_final < delivery.time_window_start:
                        # Drone arrived early, effectively waits. Update current time for next segment.
                        drone_specific_current_time_final = delivery.time_window_start
                        # print(f"[GA_FINAL_PATH] Drone {drone_id} arrived early for D{delivery_id} at {arrival_time_at_delivery_final:.2f}, waiting until {delivery.time_window_start:.2f}")
                    elif arrival_time_at_delivery_final > delivery.time_window_end:
                        # print(f"[GA_FINAL_PATH] Drone {drone_id} is LATE for D{delivery_id}. Arrival: {arrival_time_at_delivery_final:.2f}, Window End: {delivery.time_window_end:.2f}. Delivery NOT made.")
                        # Do not mark as delivered, and stop this drone's route here as it failed a constraint.
                        # The path segments up to the previous delivery (if any) are kept.
                        # We need to remove the last added segments for this failed delivery.
                        if len(path_segments) > 1 : # if path_segments had more than just the start point
                             actual_path_for_drone = actual_path_for_drone[:-len(path_segments[1:])] # Remove segments of failed delivery
                        current_pos = actual_path_for_drone[-1] if actual_path_for_drone else drone.position # Revert position
                        # Energy for this failed segment was not yet deducted from temp_drone_states_final, only from local current_energy
                        break # Stop processing this drone's route
                    else:
                        # Arrived within time window
                        drone_specific_current_time_final = arrival_time_at_delivery_final
                    
                    delivery.is_delivered = True # Mark as delivered for the final solution
                
                if len(actual_path_for_drone) > 1: # If any segments were added
                    final_routes[drone_id] = actual_path_for_drone
        
        return final_routes

    # --- END OF GENETIC ALGORITHM IMPLEMENTATION ---

    def plan_path_around_no_fly_zone(self, start: Tuple[float, float], goal: Tuple[float, float],
                                   zone: NoFlyZone, current_time: float) -> Optional[List[Tuple[float, float]]]:
        """
        Plans a simple path around a single rectangular NoFlyZone by trying to route through
        waypoints offset from the zone's corners and edge midpoints.
        This is a heuristic and checks safety against ALL no-fly zones.
        """
        # print(f"[PLAN_AROUND_NFZ] Attempting to plan path from {start} to {goal} around NFZ {zone.id}")
        
        x_min, x_max, y_min, y_max = self._get_rectangle_bounds(zone.coordinates)
        
        # Define waypoints around the NFZ, offset by safety_buffer + a small epsilon
        epsilon = 1.0  # Small distance to ensure waypoints are outside the buffer
        offset = self.safety_buffer + epsilon

        potential_waypoints = [
            (x_min - offset, y_min - offset), (x_max + offset, y_min - offset),
            (x_max + offset, y_max + offset), (x_min - offset, y_max + offset),
            (x_min - offset, (y_min + y_max) / 2), # Mid-left
            (x_max + offset, (y_min + y_max) / 2), # Mid-right
            ((x_min + x_max) / 2, y_min - offset), # Mid-bottom
            ((x_min + x_max) / 2, y_max + offset)  # Mid-top
        ]

        valid_paths = []

        for waypoint in potential_waypoints:
            # Check if waypoint is within grid boundaries
            if not (0 <= waypoint[0] <= self.grid_size and 0 <= waypoint[1] <= self.grid_size):
                # print(f"[PLAN_AROUND_NFZ] Waypoint {waypoint} out of bounds for NFZ {zone.id}")
                continue

            # Check if the waypoint itself is safe (not in *any* NFZ, including the one we are trying to avoid)
            # This uses the general _is_in_no_fly_zone which checks all NFZs.
            if self._is_in_no_fly_zone(waypoint, current_time, with_buffer=True):
                # print(f"[PLAN_AROUND_NFZ] Waypoint {waypoint} is inside an NFZ buffer (checked against all NFZs).")
                continue

            path_segment1 = [start, waypoint]
            path_segment2 = [waypoint, goal]

            # Check if path from start to waypoint is safe (against ALL NFZs)
            if not self._is_path_safe(path_segment1, current_time):
                # print(f"[PLAN_AROUND_NFZ] Path segment {start} -> {waypoint} is not safe (checked against all NFZs).")
                continue
            
            # Check if path from waypoint to goal is safe (against ALL NFZs)
            if not self._is_path_safe(path_segment2, current_time):
                # print(f"[PLAN_AROUND_NFZ] Path segment {waypoint} -> {goal} is not safe (checked against all NFZs).")
                continue

            # If all checks pass, this is a candidate path
            full_path = [start, waypoint, goal]
            path_length = self._euclidean_distance(start, waypoint) + self._euclidean_distance(waypoint, goal)
            valid_paths.append({'path': full_path, 'length': path_length})
            # print(f"[PLAN_AROUND_NFZ] Found candidate path via {waypoint} for NFZ {zone.id}, length {path_length:.2f}")

        if not valid_paths:
            # print(f"[PLAN_AROUND_NFZ] No safe path found around NFZ {zone.id} using waypoints.")
            return None

        # Sort valid paths by length and return the shortest one
        valid_paths.sort(key=lambda p: p['length'])
        shortest_path = valid_paths[0]['path']
        # print(f"[PLAN_AROUND_NFZ] Selected shortest path around NFZ {zone.id}: {shortest_path} with length {valid_paths[0]['length']:.2f}")
        return shortest_path
