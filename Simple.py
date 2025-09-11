"""
🚛 ENHANCED VEHICLE ROUTE OPTIMIZATION SYSTEM
==============================================
Business Goal: Optimize fertilizer delivery from warehouse to KDs
- Minimize total delivery cost (fixed + variable)
- Use vehicle profiles for road restrictions
- Weight-based optimization (kgs)
- Interactive map visualization
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
from folium import plugins
import webbrowser
import os
from datetime import datetime

print("✅ All libraries loaded successfully!")

# ===========================
# 2. DATA DEFINITION
# ===========================
"""
Define warehouse and KD locations with their demands in kgs
Warehouse MUST be first (index 0) - depot where all routes start/end
"""

all_locations = [
    ("Warehouse (Bhawan)", 31.522334, 72.576500),        # Depot - 0 demand
    ("KD Ada Gagh Chowk", 31.570639, 72.889028),         # Customer 1
    ("KD Ada Sheikhan", 31.591778, 72.836139),           # Customer 2
    ("KD Barkhudar", 31.495, 72.85875),                  # Customer 3
    ("KD Chak 137", 31.653, 72.865278),                  # Customer 4
    ("KD Chak 144", 31.677139, 72.909444),               # Customer 5
]

# Demands: warehouse = 0, KDs have positive demand in kgs
all_demands = [0, 5850, 45400, 5900, 7500, 24800]  # Total: 89,450 kgs (converted from bags)

print(f"📍 Loaded {len(all_locations)} locations")
print(f"📦 Total demand: {sum(all_demands):,} kgs")

# ===========================
# 3. ENHANCED VEHICLE FLEET WITH PROFILES
# ===========================
"""
Enhanced fleet with vehicle profiles for road restrictions
Includes fixed costs and per-km costs
"""

vehicles = [
    # Heavy trucks - restricted roads, high capacity
    {
        "name": "Heavy Truck",
        "count": 2, 
        "capacity": 50000,  # 50 tons
        "fixed_cost": 5000.0,  # Daily fixed cost
        "cost_per_km": 120.0,
        "profile": "truck",  # Truck routing profile
        "max_weight": 50000,
        "restrictions": {
            "avoid_residential": True,
            "avoid_service": True,
            "avoid_track": True,
            "avoid_ferries": True,
            "avoid_uturns": True,
            "min_road_width": 4.0  # meters
        }
    },
    
    # Medium trucks - balanced capacity and access
    {
        "name": "Medium Truck", 
        "count": 4, 
        "capacity": 25000,  # 25 tons
        "fixed_cost": 3000.0,
        "cost_per_km": 85.0,
        "profile": "truck",
        "max_weight": 25000,
        "restrictions": {
            "avoid_residential": False,  # Can access some residential
            "avoid_service": True,
            "avoid_track": True,
            "avoid_ferries": True,
            "avoid_uturns": True,
            "min_road_width": 3.5
        }
    },
    
    # Light trucks - flexible access, lower capacity
    {
        "name": "Light Truck",
        "count": 6, 
        "capacity": 10000,  # 10 tons
        "fixed_cost": 2000.0,
        "cost_per_km": 60.0,
        "profile": "driving",  # Car routing for flexibility
        "max_weight": 10000,
        "restrictions": {
            "avoid_residential": False,
            "avoid_service": False,  # Can access service roads
            "avoid_track": True,
            "avoid_ferries": True,
            "avoid_uturns": False,  # More maneuverable
            "min_road_width": 2.5
        }
    },
    
    # Small vans - maximum flexibility
    {
        "name": "Small Van",
        "count": 4, 
        "capacity": 3000,  # 3 tons
        "fixed_cost": 1200.0,
        "cost_per_km": 45.0,
        "profile": "driving",
        "max_weight": 3000,
        "restrictions": {
            "avoid_residential": False,
            "avoid_service": False,
            "avoid_track": False,  # Can access farm paths if needed
            "avoid_ferries": True,
            "avoid_uturns": False,
            "min_road_width": 2.0
        }
    }
]

# Calculate total fleet capacity
total_capacity = sum(v["count"] * v["capacity"] for v in vehicles)
print(f"🚛 Fleet capacity: {total_capacity:,} kgs (demand: {sum(all_demands):,} kgs)")
print(f"📊 Capacity utilization: {sum(all_demands)/total_capacity*100:.1f}%")

# ===========================
# 4. ENHANCED DISTANCE CALCULATION WITH PROFILES
# ===========================

def get_osrm_matrix_with_profile(locations, profile="driving", restrictions=None):
    """
    Get real driving distances using OSRM with vehicle profiles
    Supports different routing profiles for different vehicle types
    """
    print(f"🌐 Getting distances for profile: {profile}")

    OSRM_URL = "http://router.project-osrm.org"
    
    # Prepare coordinates (OSRM uses lon,lat format)
    coordinates = [[lon, lat] for name, lat, lon in locations]
    coord_string = ";".join([f"{lon},{lat}" for lon, lat in coordinates])

    url = f"{OSRM_URL}/table/v1/{profile}/{coord_string}"
    params = {
        'sources': ';'.join([str(i) for i in range(len(locations))]),
        'destinations': ';'.join([str(i) for i in range(len(locations))]),
        'annotations': 'distance,duration'
    }
    
    # Add restrictions for truck profiles
    if profile == "truck" and restrictions:
        exclude_options = []
        if restrictions.get("avoid_residential"):
            exclude_options.append("residential")
        if restrictions.get("avoid_service"):
            exclude_options.append("service")
        if restrictions.get("avoid_track"):
            exclude_options.append("track")
        if restrictions.get("avoid_ferries"):
            exclude_options.append("ferry")
            
        if exclude_options:
            params['exclude'] = ",".join(exclude_options)

    try:
        response = requests.get(url, params=params, timeout=30)
        response.raise_for_status()
        data = response.json()

        if data['code'] == 'Ok':
            distances = np.array(data['distances']) / 1000  # Convert to km
            durations = np.array(data['durations']) / 60    # Convert to minutes
            print(f"✅ Real driving distances retrieved for {profile}!")
            return distances, durations
        else:
            print(f"❌ OSRM error: {data['message']}")
            return None, None

    except Exception as e:
        print(f"❌ OSRM failed: {e}")
        return None, None

def haversine_fallback_enhanced(locations, road_factor=1.4):
    """
    Enhanced fallback with better road factor estimation
    """
    n = len(locations)
    distances = np.zeros((n, n))
    durations = np.zeros((n, n))

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

                straight_distance = R * c
                distances[i][j] = straight_distance * road_factor
                durations[i][j] = distances[i][j] / 50 * 60  # Assume 50 km/h avg speed

    return distances, durations

# ===========================
# 5. ENHANCED COST CALCULATION
# ===========================

def calculate_vehicle_cost(distance_km, fixed_cost, cost_per_km, load_kgs=None, capacity_kgs=None):
    """
    Enhanced cost calculation with fixed and variable components
    """
    variable_cost = distance_km * cost_per_km
    total_cost = fixed_cost + variable_cost
    
    # Add efficiency bonus for high utilization
    if load_kgs and capacity_kgs:
        utilization = load_kgs / capacity_kgs
        if utilization > 0.8:  # Bonus for >80% utilization
            total_cost *= 0.95  # 5% discount
    
    return total_cost

def make_cost_evaluator_enhanced(vehicle_spec, distance_matrix, manager):
    """
    Enhanced cost evaluation with fixed and variable costs
    """
    def cost_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        
        if from_node == to_node:
            return 0
            
        distance_km = distance_matrix[from_node][to_node]
        
        # Only variable cost for individual segments
        # Fixed cost will be added once per route
        cost = distance_km * vehicle_spec['cost_per_km']
        return int(round(cost * 100))  # Scale for integer precision

    return cost_callback

# ===========================
# 6. MAIN OPTIMIZATION FUNCTION
# ===========================

def solve_vrp_enhanced(locations, demands, vehicles):
    """
    Enhanced VRP solver with vehicle profiles and fixed costs
    """
    print("\n🔍 Step 1: Getting distance matrices for different vehicle profiles...")
    
    # Get distance matrices for different profiles
    profile_matrices = {}
    
    # Get truck profile for heavy/medium trucks
    truck_distances, truck_durations = get_osrm_matrix_with_profile(
        locations, "truck", vehicles[0]["restrictions"]
    )
    
    # Get driving profile for light vehicles
    driving_distances, driving_durations = get_osrm_matrix_with_profile(
        locations, "driving"
    )
    
    # Use fallback if needed
    if truck_distances is None:
        print("⚠️ Using fallback distances for trucks...")
        truck_distances, truck_durations = haversine_fallback_enhanced(locations, 1.6)
    
    if driving_distances is None:
        print("⚠️ Using fallback distances for light vehicles...")
        driving_distances, driving_durations = haversine_fallback_enhanced(locations, 1.3)
    
    profile_matrices["truck"] = truck_distances
    profile_matrices["driving"] = driving_distances

    print("\n🚛 Step 2: Setting up enhanced vehicle fleet...")
    vehicle_specs = []
    for vehicle_type in vehicles:
        for i in range(vehicle_type["count"]):
            spec = vehicle_type.copy()
            spec['vehicle_index'] = len(vehicle_specs)
            spec['type_instance'] = i + 1
            vehicle_specs.append(spec)

    print(f"📊 Total vehicles available: {len(vehicle_specs)}")

    print("\n⚙️ Step 3: Setting up optimization model...")
    manager = pywrapcp.RoutingIndexManager(len(locations), len(vehicle_specs), 0)
    routing = pywrapcp.RoutingModel(manager)

    # Add cost evaluators for each vehicle
    for vehicle_id, spec in enumerate(vehicle_specs):
        distance_matrix = profile_matrices[spec['profile']]
        cost_callback = make_cost_evaluator_enhanced(spec, distance_matrix, manager)
        cost_evaluator_index = routing.RegisterTransitCallback(cost_callback)
        routing.SetArcCostEvaluatorOfVehicle(cost_evaluator_index, vehicle_id)

    # Add capacity constraints
    def demand_callback(from_index):
        return demands[manager.IndexToNode(from_index)]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # No slack
        [spec['capacity'] for spec in vehicle_specs],
        True,  # Start cumul to zero
        'Capacity'
    )

    print("\n🔍 Step 4: Solving optimization...")
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 120

    solution = routing.SolveWithParameters(search_parameters)

    if not solution:
        print("❌ No solution found!")
        return None, None, None

    print("\n✅ Solution found! Extracting routes...")
    routes = []

    for vehicle_id in range(len(vehicle_specs)):
        index = routing.Start(vehicle_id)
        route_nodes = []
        route_load = 0
        route_distance = 0

        spec = vehicle_specs[vehicle_id]
        distance_matrix = profile_matrices[spec['profile']]

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
            # Add return trip to warehouse
            last_customer = route_nodes[-1]
            return_distance = distance_matrix[last_customer][0]
            route_distance += return_distance
            
            # Calculate total cost with fixed and variable components
            total_cost = calculate_vehicle_cost(
                route_distance, 
                spec['fixed_cost'], 
                spec['cost_per_km'],
                route_load,
                spec['capacity']
            )
            
            routes.append({
                'vehicle_id': vehicle_id + 1,
                'vehicle_name': f"{spec['name']} #{spec['type_instance']}",
                'vehicle_type': spec['name'],
                'profile': spec['profile'],
                'route_nodes': route_nodes,
                'complete_route': route_nodes + [0],
                'load': route_load,
                'capacity': spec['capacity'],
                'distance': route_distance,
                'return_distance': return_distance,
                'fixed_cost': spec['fixed_cost'],
                'variable_cost': route_distance * spec['cost_per_km'],
                'total_cost': total_cost,
                'cost_per_km': spec['cost_per_km'],
                'utilization': (route_load / spec['capacity']) * 100,
                'restrictions': spec['restrictions']
            })

    return routes, profile_matrices, vehicle_specs

# ===========================
# 7. ENHANCED VISUALIZATION
# ===========================

def create_route_map(routes, locations, demands, save_path="route_map.html"):
    """
    Create interactive map with route visualization
    """
    if not routes:
        print("❌ No routes to visualize!")
        return None
    
    print("\n🗺️ Creating interactive route map...")
    
    # Calculate map center
    lats = [loc[1] for loc in locations]
    lons = [loc[2] for loc in locations]
    center_lat = sum(lats) / len(lats)
    center_lon = sum(lons) / len(lons)
    
    # Create base map
    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=11,
        tiles='OpenStreetMap'
    )
    
    # Color scheme for different vehicle types
    colors = ['red', 'blue', 'green', 'purple', 'orange', 'darkred', 
              'lightred', 'beige', 'darkblue', 'darkgreen', 'cadetblue', 
              'darkpurple', 'white', 'pink', 'lightblue', 'lightgreen']
    
    # Add warehouse marker
    warehouse = locations[0]
    folium.Marker(
        location=[warehouse[1], warehouse[2]],
        popup=folium.Popup(f"🏭 {warehouse[0]}<br>Depot", max_width=200),
        tooltip="Warehouse/Depot",
        icon=folium.Icon(color='black', icon='home', prefix='fa')
    ).add_to(m)
    
    # Add customer markers and routes
    for i, route in enumerate(routes):
        color = colors[i % len(colors)]
        
        # Add route polyline
        route_coords = []
        for node in route['complete_route']:
            loc = locations[node]
            route_coords.append([loc[1], loc[2]])
        
        folium.PolyLine(
            locations=route_coords,
            weight=4,
            color=color,
            opacity=0.8,
            popup=folium.Popup(
                f"🚛 {route['vehicle_name']}<br>"
                f"📦 Load: {route['load']:,} kgs<br>"
                f"🛣️ Distance: {route['distance']:.1f} km<br>"
                f"💰 Cost: ₨{route['total_cost']:,.0f}",
                max_width=250
            )
        ).add_to(m)
        
        # Add customer markers for this route
        for j, node in enumerate(route['route_nodes'][1:], 1):  # Skip warehouse
            loc = locations[node]
            demand = demands[node]
            
            folium.Marker(
                location=[loc[1], loc[2]],
                popup=folium.Popup(
                    f"📍 {loc[0]}<br>"
                    f"📦 Demand: {demand:,} kgs<br>"
                    f"🚛 Vehicle: {route['vehicle_name']}<br>"
                    f"📊 Stop #{j}",
                    max_width=200
                ),
                tooltip=f"{loc[0]} ({demand:,} kgs)",
                icon=folium.Icon(color=color, icon='shopping-cart', prefix='fa')
            ).add_to(m)
    
    # Add route summary table
    summary_html = """
    <div style='position: fixed; 
                top: 10px; right: 10px; width: 300px; height: auto;
                background-color: white; border:2px solid grey; z-index:9999; 
                font-size:12px; padding: 10px; border-radius: 5px;
                box-shadow: 0 0 15px rgba(0,0,0,0.2);'>
    <h4>📊 Route Summary</h4>
    <table style='width:100%; font-size:10px;'>
    <tr><th>Vehicle</th><th>Load (kg)</th><th>Distance</th><th>Cost</th></tr>
    """
    
    total_cost = 0
    total_distance = 0
    for route in routes:
        summary_html += f"""
        <tr>
            <td>{route['vehicle_name'][:15]}</td>
            <td>{route['load']:,}</td>
            <td>{route['distance']:.0f}km</td>
            <td>₨{route['total_cost']:,.0f}</td>
        </tr>
        """
        total_cost += route['total_cost']
        total_distance += route['distance']
    
    summary_html += f"""
    <tr style='font-weight:bold; border-top: 1px solid #ccc;'>
        <td>TOTAL</td>
        <td>{sum(r['load'] for r in routes):,}</td>
        <td>{total_distance:.0f}km</td>
        <td>₨{total_cost:,.0f}</td>
    </tr>
    </table>
    </div>
    """
    
    m.get_root().html.add_child(folium.Element(summary_html))
    
    # Add legend
    legend_html = """
    <div style='position: fixed; 
                bottom: 50px; left: 50px; width: 200px; height: 120px; 
                background-color: white; border:2px solid grey; z-index:9999; 
                font-size:12px; padding: 10px; border-radius: 5px;'>
    <h4>🗺️ Map Legend</h4>
    <p><i class="fa fa-home" style="color:black"></i> Warehouse/Depot</p>
    <p><i class="fa fa-shopping-cart" style="color:red"></i> Customer Locations</p>
    <p><span style="color:blue">─────</span> Delivery Routes</p>
    <p><b>Click markers and lines for details</b></p>
    </div>
    """
    
    m.get_root().html.add_child(folium.Element(legend_html))
    
    # Save map
    m.save(save_path)
    print(f"✅ Interactive map saved as: {save_path}")
    
    return m

# ===========================
# 8. ENHANCED RESULTS DISPLAY
# ===========================

def display_enhanced_results(routes, locations, demands):
    """
    Display enhanced optimization results with fixed costs
    """
    if not routes:
        print("❌ No routes to display!")
        return

    print("\n" + "="*80)
    print("🚛 ENHANCED DELIVERY ROUTES WITH VEHICLE PROFILES")
    print("="*80)

    total_cost = 0
    total_distance = 0
    total_load = 0
    total_fixed_cost = 0
    total_variable_cost = 0

    for route in routes:
        route_names = [locations[node][0] for node in route['route_nodes']]
        
        print(f"\n🚚 {route['vehicle_name']} ({route['capacity']:,} kgs capacity)")
        print(f"   📍 Route: {' → '.join(route_names)} → Warehouse")
        print(f"   📦 Load: {route['load']:,}/{route['capacity']:,} kgs ({route['utilization']:.1f}% full)")
        print(f"   🛣️ Distance: {route['distance']:.1f} km (profile: {route['profile']})")
        print(f"   💰 Fixed Cost: ₨{route['fixed_cost']:,.0f}")
        print(f"   💰 Variable Cost: ₨{route['variable_cost']:,.0f} ({route['distance']:.1f}km × ₨{route['cost_per_km']}/km)")
        print(f"   💰 Total Cost: ₨{route['total_cost']:,.0f}")
        print(f"   📊 Cost per kg: ₨{route['total_cost']/route['load']:.2f}")
        
        # Show restrictions
        restrictions = route['restrictions']
        restriction_list = []
        if restrictions['avoid_residential']: restriction_list.append("No residential roads")
        if restrictions['avoid_service']: restriction_list.append("No service roads")
        if restrictions['avoid_track']: restriction_list.append("No tracks")
        if restrictions['avoid_uturns']: restriction_list.append("No U-turns")
        if restrictions['avoid_ferries']: restriction_list.append("No ferries")
        
        if restriction_list:
            print(f"   🚫 Restrictions: {', '.join(restriction_list)}")

        total_cost += route['total_cost']
        total_distance += route['distance']
        total_load += route['load']
        total_fixed_cost += route['fixed_cost']
        total_variable_cost += route['variable_cost']

    # Enhanced summary
    print("\n" + "="*80)
    print("📊 ENHANCED OPTIMIZATION SUMMARY")
    print("="*80)
    print(f"🚛 Vehicles used: {len(routes)}")
    print(f"🛣️ Total distance: {total_distance:.1f} km")
    print(f"💰 Total fixed costs: ₨{total_fixed_cost:,.0f}")
    print(f"� Total variable costs: ₨{total_variable_cost:,.0f}")
    print(f"💰 TOTAL COST: ₨{total_cost:,.0f}")
    print(f"📦 Total weight delivered: {total_load:,} kgs")
    print(f"📊 Average cost per kg: ₨{total_cost/total_load:.2f}")
    print(f"📊 Average cost per km: ₨{total_cost/total_distance:.2f}")
    print(f"📊 Fixed vs Variable ratio: {total_fixed_cost/total_cost*100:.1f}% : {total_variable_cost/total_cost*100:.1f}%")

# ===========================
# 9. RUN THE ENHANCED OPTIMIZATION
# ===========================

if __name__ == "__main__":
    print("🚀 Starting Enhanced Vehicle Route Optimization...")

    # Filter locations (keep warehouse + locations with demand)
    locations = []
    demands = []

    for i, (location, demand) in enumerate(zip(all_locations, all_demands)):
        if i == 0 or demand > 0:  # Keep warehouse and customers with demand
            locations.append(location)
            demands.append(demand)

    print(f"📍 Active locations: {len(locations)}")
    print(f"📦 Total demand: {sum(demands):,} kgs")

    # Solve the enhanced VRP
    routes, distance_matrices, vehicle_specs = solve_vrp_enhanced(locations, demands, vehicles)

    if routes:
        # Display results
        display_enhanced_results(routes, locations, demands)
        
        # Create interactive map
        map_file = f"route_optimization_{datetime.now().strftime('%Y%m%d_%H%M%S')}.html"
        route_map = create_route_map(routes, locations, demands, map_file)
        
        # Open map in browser
        try:
            full_path = os.path.abspath(map_file)
            webbrowser.open(f'file://{full_path}')
            print(f"🌐 Opening map in browser: {map_file}")
        except:
            print(f"💡 Please open {map_file} manually in your browser")
        
        print("\n✅ Enhanced optimization complete!")
        print(f"📊 Key Improvements:")
        print(f"   • Vehicle profiles for road restrictions")
        print(f"   • Fixed + variable cost optimization")
        print(f"   • Weight-based metrics (kgs)")
        print(f"   • Interactive route visualization")
        print(f"   • Enhanced vehicle utilization tracking")
    else:
        print("❌ No solution found!")