from .models import Drone, DeliveryPoint, NoFlyZone # Changed to relative import
from .algorithms import DroneFleetOptimizer # Changed to relative import
from .visualization import DroneFleetVisualizer # Changed to relative import
import matplotlib.pyplot as plt
import time
import ast # For parsing the data file
import numpy as np # For no-fly zone calculations
import copy # For deep copying objects
from typing import List, Dict, Tuple # Added missing import for type hints

def load_data_from_file(filepath: str):
    """Veri setini dosyadan yükler."""
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Güvenli bir şekilde string'i Python objelerine çevir
    # Dosya içeriğini ayrı ayrı değerlendir
    data_dict = {}
    current_var_name = None
    buffer = ""
    for line in content.splitlines():
        stripped_line = line.strip()
        if not stripped_line or stripped_line.startswith("#"):
            continue
        if "=" in line and "[" in line: # Start of a new list assignment
            if current_var_name and buffer:
                try:
                    data_dict[current_var_name] = ast.literal_eval(buffer.strip().rstrip(','))
                except Exception as e:
                    print(f"Error parsing {current_var_name}: {e}")
                    print(f"Buffer content: {buffer}")
                    data_dict[current_var_name] = [] # Assign empty list on error
            current_var_name = stripped_line.split('=')[0].strip()
            buffer = stripped_line.split('=', 1)[1].strip()
        elif current_var_name:
            buffer += "\n" + line
    
    if current_var_name and buffer: # Process the last variable
         try:
            data_dict[current_var_name] = ast.literal_eval(buffer.strip().rstrip(','))
         except Exception as e:
            print(f"Error parsing {current_var_name}: {e}")
            print(f"Buffer content: {buffer}")
            data_dict[current_var_name] = []


    raw_drones = data_dict.get('drones', [])
    raw_deliveries = data_dict.get('deliveries', [])
    raw_zones = data_dict.get('no_fly_zones', [])

    drones = [
        Drone(
            id=d["id"],
            max_energy=float(d["battery"]), # Assuming battery is max_energy
            current_energy=float(d["battery"]), # Assuming battery is current_energy
            max_weight=float(d["max_weight"]),
            position=tuple(d["start_pos"]),
            speed=float(d["speed"])
        ) for d in raw_drones
    ]
    
    deliveries = [
        DeliveryPoint(
            id=dp["id"],
            position=tuple(dp["pos"]),
            weight=float(dp["weight"]),
            priority=int(dp["priority"]),
            time_window_start=float(dp["time_window"][0]) if "time_window" in dp and isinstance(dp["time_window"], list) and len(dp["time_window"]) == 2 else 0.0,
            time_window_end=float(dp["time_window"][1]) if "time_window" in dp and isinstance(dp["time_window"], list) and len(dp["time_window"]) == 2 else float('inf')
        ) for dp in raw_deliveries
    ]
    
    zones = []
    for z_data in raw_zones:
        # Directly use the coordinates from the file
        coordinates = [tuple(c) for c in z_data["coordinates"]]
        
        zones.append(NoFlyZone(
            id=z_data["id"],
            coordinates=coordinates, # Store coordinates directly
            start_time=float(z_data["active_time"][0]),
            end_time=float(z_data["active_time"][1])
        ))
        
    return drones, deliveries, zones

def reset_scenario_state(initial_drones: List[Drone], initial_deliveries: List[DeliveryPoint]):
    """Resets drone energy and delivery status for a new optimization run."""
    drones_copy = copy.deepcopy(initial_drones)
    deliveries_copy = copy.deepcopy(initial_deliveries)
    
    # Reset drone states
    for drone_obj in drones_copy: # Use a different variable name to avoid conflict
        drone_obj.current_energy = drone_obj.max_energy
        drone_obj.current_weight = 0.0
        # Ensure position is a tuple, might not be strictly necessary if always loaded correctly
        # but good for safety if Drone objects could be modified in unexpected ways.
        drone_obj.position = tuple(drone_obj.position)
        drone_obj.is_available = True

    # Reset delivery states
    for dp in deliveries_copy:
        dp.is_delivered = False
    return drones_copy, deliveries_copy

def analyze_and_print_results(routes: Dict[int, List[Tuple[float, float]]],
                              drones_in_run: List[Drone], # Drones used for this specific run
                              deliveries_in_run: List[DeliveryPoint], # Deliveries used for this specific run
                              num_total_deliveries_in_scenario: int,
                              algorithm_name: str,
                              optimizer_ref: DroneFleetOptimizer):
    """Analyzes and prints the results for a given set of routes."""
    print(f"\n--- {algorithm_name} Sonuçları ---")
    
    total_deliveries_made = sum(1 for d in deliveries_in_run if d.is_delivered)
    completion_rate = (total_deliveries_made / num_total_deliveries_in_scenario) * 100 if num_total_deliveries_in_scenario > 0 else 0.0
    
    total_energy_used = 0
    active_drones_count = 0 # Count of drones that actually have routes

    for drone_obj in drones_in_run:
        route_path = routes.get(drone_obj.id)
        if route_path and len(route_path) > 1: # Drone has a route with at least one segment
            active_drones_count += 1
            energy_used_for_drone = sum(
                optimizer_ref._euclidean_distance(route_path[i], route_path[i+1])
                for i in range(len(route_path)-1)
            )
            total_energy_used += energy_used_for_drone
            
    avg_energy_per_active_drone = total_energy_used / active_drones_count if active_drones_count else 0
    
    print(f"Tamamlanan teslimat sayısı: {total_deliveries_made}/{num_total_deliveries_in_scenario} ({completion_rate:.1f}%)")
    print(f"Aktif (rota atanmış) drone sayısı: {active_drones_count}/{len(drones_in_run)}")
    print(f"Ortalama enerji tüketimi (aktif drone'lar için): {avg_energy_per_active_drone:.2f}")
    undelivered_ids = sorted([dp.id for dp in deliveries_in_run if not dp.is_delivered])
    if undelivered_ids:
        print(f"Teslim edilemeyen noktalar (ID): {undelivered_ids}")


def run_scenario(): # Removed parameters
    """Örnek senaryo çalıştırma fonksiyonu"""
    
    # Veri yükle
    load_start_time = time.time()
    try:
        # Load initial state once
        initial_drones, initial_deliveries, initial_zones = load_data_from_file("data/veri_seti.txt")
    except FileNotFoundError:
        print("Hata: data/veri_seti.txt dosyası bulunamadı.")
        return
    except Exception as e:
        print(f"Veri yüklenirken hata oluştu: {e}")
        return
        
    data_load_time = time.time() - load_start_time
    
    if not initial_drones and not initial_deliveries:
        print("Yüklenecek drone veya teslimat verisi bulunamadı.")
        return

    num_drones_scenario = len(initial_drones)
    num_deliveries_scenario = len(initial_deliveries)
    num_zones_scenario = len(initial_zones)

    print(f"\nSenaryo: {num_drones_scenario} drone, {num_deliveries_scenario} teslimat, {num_zones_scenario} uçuşa yasak bölge (veri_seti.txt'den yüklendi)")
    print("-" * 50)
    print(f"Veri yükleme tamamlandı ({data_load_time:.2f} saniye)")
    
    # --- Greedy Algorithm Run ---
    print("\n\n==================================================")
    print("[MAIN] GREEDY ALGORİTMA İLE OPTİMİZASYON BAŞLATILIYOR...")
    print("==================================================")
    greedy_drones, greedy_deliveries = reset_scenario_state(initial_drones, initial_deliveries)
    optimizer_greedy = DroneFleetOptimizer(greedy_drones, greedy_deliveries, initial_zones) # Use fresh copies
    
    greedy_start_time = time.time()
    routes_greedy = optimizer_greedy.optimize()
    greedy_optimization_time = time.time() - greedy_start_time
    print(f"Greedy Algoritma Optimizasyonu tamamlandı ({greedy_optimization_time:.2f} saniye)")
    analyze_and_print_results(routes_greedy, greedy_drones, greedy_deliveries, num_deliveries_scenario, "Greedy Algorithm", optimizer_greedy)

    visualizer_greedy = DroneFleetVisualizer(greedy_drones, greedy_deliveries, initial_zones)
    visualizer_greedy.plot_environment(routes_greedy, title_suffix=" - Greedy Algorithm")
    plt.show(block=False)

    # --- Genetic Algorithm Run ---
    print("\n\n====================================================")
    print("[MAIN] GENETİK ALGORİTMA İLE OPTİMİZASYON BAŞLATILIYOR...")
    print("====================================================")
    ga_drones, ga_deliveries = reset_scenario_state(initial_drones, initial_deliveries)
    optimizer_ga = DroneFleetOptimizer(ga_drones, ga_deliveries, initial_zones) # Use fresh copies

    ga_start_time = time.time()
    # Ensure GA also uses its own fresh set of deliveries for is_delivered tracking
    # The optimizer_ga is initialized with ga_deliveries, which are deep copies.
    # The GA's internal logic (e.g. _evaluate_chromosome_details, optimize_with_ga final pathing)
    # should correctly use and manage the is_delivered status of these ga_deliveries.
    routes_ga = optimizer_ga.optimize_with_ga(population_size=200, generations=500)
    ga_optimization_time = time.time() - ga_start_time
    print(f"Genetik Algoritma Optimizasyonu tamamlandı ({ga_optimization_time:.2f} saniye)")
    analyze_and_print_results(routes_ga, ga_drones, ga_deliveries, num_deliveries_scenario, "Genetic Algorithm", optimizer_ga)
    
    visualizer_ga = DroneFleetVisualizer(ga_drones, ga_deliveries, initial_zones)
    visualizer_ga.plot_environment(routes_ga, title_suffix=" - Genetic Algorithm")
    
    # Plotting other statistics - decide if these should be for GA, Greedy, or both
    # For now, let's assume these are for the GA results as it's the more complex one.
    print("\n--- Genetic Algorithm - Ek İstatistikler ---")
    visualizer_ga.plot_energy_consumption(routes_ga)
    plt.show(block=False)
    visualizer_ga.plot_delivery_statistics(routes_ga)
    plt.show() # Last plot can be blocking to keep all windows open until closed by user

def main():
    """Ana fonksiyon"""
    print("Drone Filo Optimizasyonu - Test Senaryoları")
    print("=" * 50)
    
    # Senaryo veri_seti.txt'den yüklenecek
    run_scenario()
    
    # Eski senaryolar kaldırıldı çünkü artık dosyadan yükleniyor
    # run_scenario(3, 3, 1)
    # run_scenario(10, 50, 5)

if __name__ == "__main__":
    main()