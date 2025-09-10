"""
🚛 SIMPLIFIED VEHICLE ROUTE OPTIMIZATION SYSTEM
==============================================
Business Goal: Optimize fertilizer delivery from warehouse to KDs
- Minimize total delivery cost
- Use owned vehicles only (no rental complexity)
- Generate real-world driving routes
"""

# ===========================
# 1. LIBRARY IMPORTS & SETUP
# ===========================
import requests
import json
import numpy as np
from math import radians, sin, cos, sqrt, atan2
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import folium

print("✅ All libraries loaded successfully!")

# ===========================
# 2. DATA DEFINITION
# ===========================
"""
Define warehouse and KD locations with their demands
Warehouse MUST be first (index 0) - depot where all routes start/end
"""

# all_locations = [
#     ("Warehouse (Bhawan)", 31.522334, 72.576500),        # Depot - 0 demand
#     ("KD Ada Gagh Chowk", 31.570639, 72.889028),         # Customer 1
#     ("KD Ada Sheikhan", 31.591778, 72.836139),           # Customer 2
#     ("KD Barkhudar", 31.495, 72.85875),                  # Customer 3
#     ("KD Chak 137", 31.653, 72.865278),                  # Customer 4
#     ("KD Chak 144", 31.677139, 72.909444),               # Customer 5
# ]

# # Demands: warehouse = 0, KDs have positive demand
# all_demands = [0, 117, 908, 118, 150, 496]  # Total: 1,789 bags

print(f"📍 Loaded {len(all_locations)} locations")
print(f"📦 Total demand: {sum(all_demands)} bags")

# ===========================
# 3. SIMPLIFIED VEHICLE FLEET (OWNED ONLY)
# ===========================
"""
Simple fleet design - all owned vehicles with distance-based costs
No rental complexity, just capacity and cost per km
"""

vehicles = [
    # Large trucks - for high-volume deliveries (like the 908-bag customer)
    {"count": 4, "capacity": 1000, "cost_per_km": 100.0},
    
    # Medium trucks - balanced capacity and cost
    {"count": 6, "capacity": 500, "cost_per_km": 70.0},
    
    # Small vans - cost-efficient for small deliveries
    {"count": 3, "capacity": 200, "cost_per_km": 50.0},
]
# Calculate total fleet capacity
total_capacity = sum(v["count"] * v["capacity"] for v in vehicles)
print(f"🚛 Fleet capacity: {total_capacity} bags (demand: {sum(all_demands)} bags)")
print(f"📊 Capacity utilization: {sum(all_demands)/total_capacity*100:.1f}%")

# ===========================
# 4. DISTANCE CALCULATION
# ===========================

def get_osrm_matrix(locations):
    """
    Get real driving distances using OSRM routing engine
    Returns actual road distances for accurate optimization
    """
    print(f"🌐 Getting real driving distances for {len(locations)} locations...")

    OSRM_URL = "http://router.project-osrm.org"

    # Prepare coordinates (OSRM uses lon,lat format)
    coordinates = [[lon, lat] for name, lat, lon in locations]
    coord_string = ";".join([f"{lon},{lat}" for lon, lat in coordinates])

    url = f"{OSRM_URL}/table/v1/driving/{coord_string}"
    params = {
        'sources': ';'.join([str(i) for i in range(len(locations))]),
        'destinations': ';'.join([str(i) for i in range(len(locations))]),
        'annotations': 'distance,duration'
    }

    try:
        response = requests.get(url, params=params, timeout=30)
        response.raise_for_status()
        data = response.json()

        if data['code'] == 'Ok':
            distances = np.array(data['distances']) / 1000  # Convert to km
            print(f"✅ Real driving distances retrieved!")
            return distances
        else:
            print(f"❌ OSRM error: {data['message']}")
            return None

    except Exception as e:
        print(f"❌ OSRM failed: {e}")
        return None

def haversine_fallback(locations):
    """
    Fallback distance calculation if OSRM fails
    Uses straight-line distance with road factor
    """
    n = len(locations)
    distances = np.zeros((n, n))

    for i in range(n):
        for j in range(n):
            if i != j:
                lat1, lon1 = locations[i][1], locations[i][2]
                lat2, lon2 = locations[j][1], locations[j][2]

                # Haversine formula
                R = 6371.0  # Earth's radius in km
                dlat = radians(lat2 - lat1)
                dlon = radians(lon2 - lon1)
                a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
                c = 2 * atan2(sqrt(a), sqrt(1-a))

                distances[i][j] = R * c * 1.3  # 30% road factor

    return distances

# ===========================
# 5. SIMPLIFIED COST CALCULATION
# ===========================

def calculate_vehicle_cost(distance_km, cost_per_km):
    """
    SIMPLIFIED: Only owned vehicles
    Cost = distance × cost_per_km
    """
    return distance_km * cost_per_km

def make_cost_evaluator(cost_per_km, distance_matrix, manager):
    """
    Create cost evaluation function for OR-Tools
    Simple: distance × cost_per_km
    """
    def cost_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        distance_km = distance_matrix[from_node][to_node]

        cost = distance_km * cost_per_km
        return int(round(cost * 100))  # Scale for integer precision

    return cost_callback

# ===========================
# 6. MAIN OPTIMIZATION FUNCTION
# ===========================

def solve_vrp(locations, demands, vehicles):
    """
    Simplified VRP solver - owned vehicles only
    """

    # Step 1: Get distance matrix
    print("\n🔍 Step 1: Getting distance matrix...")
    distance_matrix = get_osrm_matrix(locations)

    if distance_matrix is None:
        print("⚠️ Using fallback Haversine distances...")
        distance_matrix = haversine_fallback(locations)

    # Step 2: Create vehicle specifications
    print("\n🚛 Step 2: Setting up vehicle fleet...")
    vehicle_specs = []
    for vehicle_type in vehicles:
        for _ in range(vehicle_type["count"]):
            vehicle_specs.append({
                'capacity': vehicle_type["capacity"],
                'cost_per_km': vehicle_type["cost_per_km"]
            })

    print(f"📊 Total vehicles available: {len(vehicle_specs)}")

    # Step 3: Set up OR-Tools model
    print("\n⚙️ Step 3: Setting up optimization model...")
    manager = pywrapcp.RoutingIndexManager(len(locations), len(vehicle_specs), 0)
    routing = pywrapcp.RoutingModel(manager)

    # Step 4: Add cost evaluators for each vehicle
    for vehicle_id, spec in enumerate(vehicle_specs):
        cost_callback = make_cost_evaluator(
            spec['cost_per_km'], distance_matrix, manager
        )
        cost_evaluator_index = routing.RegisterTransitCallback(cost_callback)
        routing.SetArcCostEvaluatorOfVehicle(cost_evaluator_index, vehicle_id)

    # Step 5: Add capacity constraints
    def demand_callback(from_index):
        return demands[manager.IndexToNode(from_index)]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # No slack
        [spec['capacity'] for spec in vehicle_specs],  # Vehicle capacities
        True,  # Start cumul to zero
        'Capacity'
    )

    # Step 6: Solve the problem
    print("\n🔍 Step 4: Solving optimization...")
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 60

    solution = routing.SolveWithParameters(search_parameters)

    if not solution:
        print("❌ No solution found!")
        return None, None

    # Step 7: Extract solution
    print("\n✅ Solution found! Extracting routes...")
    routes = []

    for vehicle_id in range(len(vehicle_specs)):
        index = routing.Start(vehicle_id)
        route_nodes = []
        route_load = 0
        route_distance = 0

        # Follow the route
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route_nodes.append(node)
            route_load += demands[node]

            next_index = solution.Value(routing.NextVar(index))
            if not routing.IsEnd(next_index):
                next_node = manager.IndexToNode(next_index)
                route_distance += distance_matrix[node][next_node]
            index = next_index

        # Only include routes that visit customers
        if len(route_nodes) > 1:
            spec = vehicle_specs[vehicle_id]
            
            # 🔧 CRITICAL FIX: Add return trip to warehouse
            last_customer = route_nodes[-1]
            return_distance = distance_matrix[last_customer][0]  # Distance back to warehouse
            route_distance += return_distance
            
            # Calculate corrected cost with complete route
            route_cost = calculate_vehicle_cost(route_distance, spec['cost_per_km'])
            
            routes.append({
                'vehicle_id': vehicle_id + 1,
                'route_nodes': route_nodes,
                'complete_route': route_nodes + [0],  # Show complete route including return
                'load': route_load,
                'capacity': spec['capacity'],
                'distance': route_distance,  # Now includes return trip
                'return_distance': return_distance,  # Track return leg separately
                'cost': route_cost,
                'cost_per_km': spec['cost_per_km'],
                'utilization': (route_load / spec['capacity']) * 100
            })


    return routes, distance_matrix

# ===========================
# 7. RESULTS DISPLAY
# ===========================

def display_results(routes, locations, demands):
    """
    Display optimization results clearly
    """
    if not routes:
        print("❌ No routes to display!")
        return

    print("\n" + "="*60)
    print("🚛 OPTIMIZED DELIVERY ROUTES")
    print("="*60)

    total_cost = 0
    total_distance = 0
    total_load = 0

    for route in routes:
        # Route details
        route_names = [locations[node][0] for node in route['route_nodes']]
        print(f"\n🚚 Vehicle {route['vehicle_id']} ({route['capacity']} bags capacity):")
        print(f"   📍 Route: {' → '.join(route_names)} → Warehouse")
        print(f"   📦 Load: {route['load']}/{route['capacity']} bags ({route['load']/route['capacity']*100:.1f}% full)")
        print(f"   🛣️  Distance: {route['distance']:.1f} km")
        print(f"   💰 Cost: {route['distance']:.1f}km × ₨{route['cost_per_km']}/km = ₨{route['cost']:.2f}")
        print(f"   📊 Cost per bag: ₨{route['cost']/route['load']:.1f}")

        total_cost += route['cost']
        total_distance += route['distance']
        total_load += route['load']

    # Summary
    print("\n" + "="*60)
    print("📊 OPTIMIZATION SUMMARY")
    print("="*60)
    print(f"🚛 Vehicles used: {len(routes)}")
    print(f"🛣️  Total distance: {total_distance:.1f} km")
    print(f"💰 Total cost: ₨{total_cost:.2f}")
    print(f"📦 Total bags delivered: {total_load}")
    print(f"📊 Average cost per bag: ₨{total_cost/total_load:.2f}")
    print(f"📊 Average cost per km: ₨{total_cost/total_distance:.2f}")

# ===========================
# 8. RUN THE OPTIMIZATION
# ===========================

if __name__ == "__main__":
    print("🚀 Starting Simplified Vehicle Route Optimization...")

    # Filter locations (keep warehouse + locations with demand)
    locations = []
    demands = []

    for i, (location, demand) in enumerate(zip(all_locations, all_demands)):
        if i == 0 or demand > 0:  # Keep warehouse and customers with demand
            locations.append(location)
            demands.append(demand)

    print(f"📍 Active locations: {len(locations)}")
    print(f"📦 Total demand: {sum(demands)} bags")

    # Solve the VRP
    routes, distance_matrix = solve_vrp(locations, demands, vehicles)

    # Display results
    display_results(routes, locations, demands)

    print("\n✅ Optimization complete!")