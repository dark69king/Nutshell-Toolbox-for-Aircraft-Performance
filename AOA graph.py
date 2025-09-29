import math
import sys
import matplotlib.pyplot as plt

ENGINES = {
    "CFM56-7B": {"thrust_kn": 121, "tsfc": 0.000036},
    "GE90-115B": {"thrust_kn": 514, "tsfc": 0.000030},
    "RR Trent 900": {"thrust_kn": 374, "tsfc": 0.000032},
    "PW4000": {"thrust_kn": 276, "tsfc": 0.000035},
    "GEnx": {"thrust_kn": 330, "tsfc": 0.000029},
    "CF34-8C": {"thrust_kn": 61.3, "tsfc": 0.000041},
    "Lycoming IO-360": {"thrust_kn": 0.8, "tsfc": 0.000070}
}

AIRCRAFTS = {
    # Shape factor < 1.0 -> Fuller curve (traditional airfoils)
    # Shape factor > 1.0 -> Sharper curve (supercritical airfoils)
    "Cessna 172": {"wingspan": 11.0, "ar": 7.32, "cl_max_clean": 1.6, "stall_aoa_clean": 17.0, "shape_factor": 0.80},
    "Boeing 747-8": {"wingspan": 68.5, "ar": 8.5, "cl_max_clean": 1.45, "stall_aoa_clean": 16.0, "shape_factor": 1.10},
    "Airbus A380-800": {"wingspan": 79.75, "ar": 7.5, "cl_max_clean": 1.5, "stall_aoa_clean": 16.5, "shape_factor": 1.05},
    "Boeing 787-9": {"wingspan": 60.1, "ar": 11.0, "cl_max_clean": 1.5, "stall_aoa_clean": 15.5, "shape_factor": 1.20},
    "Boeing 737-800": {"wingspan": 35.8, "ar": 9.45, "cl_max_clean": 1.4, "stall_aoa_clean": 15.0, "shape_factor": 1.15},
    "Airbus A320neo": {"wingspan": 35.8, "ar": 9.5, "cl_max_clean": 1.4, "stall_aoa_clean": 15.0, "shape_factor": 1.15},
    "Embraer E175": {"wingspan": 26.0, "ar": 8.2, "cl_max_clean": 1.45, "stall_aoa_clean": 15.0, "shape_factor": 0.95},
}

G = 9.80665
RHO = 1.225
C_D0 = 0.020
OSWALD_EFFICIENCY = 0.80
PERFORMANCE_C_L_MAX = 1.8
POST_STALL_DROP_FACTOR = 0.45
PLOT_MAX_AOA = 25.0

class AircraftPerformance:
    def __init__(self, aircraft_data, engine_data, num_engines, takeoff_weight_kg):
        self.aircraft_name = aircraft_data['name']
        self.engine_name = engine_data['name']
        self.wingspan = aircraft_data['wingspan']
        self.aspect_ratio = aircraft_data['ar']
        self.cl_max_plot = aircraft_data['cl_max_clean']
        self.stall_angle_plot = aircraft_data['stall_aoa_clean']
        self.shape_factor = aircraft_data['shape_factor']
        self.wing_area = self.wingspan**2 / self.aspect_ratio
        self.total_thrust_N = engine_data['thrust_kn'] * 1000 * num_engines
        self.tsfc = engine_data['tsfc']
        self.takeoff_weight_N = takeoff_weight_kg * G
        
        print("\n⚙️  Analyzing Configuration...")
        print(f"   - Aircraft: {self.aircraft_name} (Wing Area: {self.wing_area:.2f} m², Aspect Ratio: {self.aspect_ratio})")
        print(f"   - Engines: {num_engines} x {self.engine_name} (Total Thrust: {self.total_thrust_N/1000:.2f} kN)")
        print(f"   - Weight: {takeoff_weight_kg} kg")

    def calculate_stall_speed(self):
        v_stall = math.sqrt(2 * self.takeoff_weight_N / (RHO * self.wing_area * PERFORMANCE_C_L_MAX))
        return v_stall

    def calculate_performance_at_speed(self, velocity_ms):
        if velocity_ms <= 0: return 0, 0, float('inf'), 0
        c_l = (2 * self.takeoff_weight_N) / (RHO * self.wing_area * velocity_ms**2)
        c_di = c_l**2 / (math.pi * self.aspect_ratio * OSWALD_EFFICIENCY)
        c_d = C_D0 + c_di
        drag_N = 0.5 * RHO * velocity_ms**2 * self.wing_area * c_d
        alpha_rad = c_l / (2 * math.pi)
        alpha_deg = math.degrees(alpha_rad)
        return c_l, c_d, drag_N, alpha_deg

    def find_max_speed(self, v_stall):
        for v in range(int(v_stall), 500):
            _, _, drag_N, _ = self.calculate_performance_at_speed(v)
            if drag_N > self.total_thrust_N: return v - 1
        return 499

    def generate_lift_curve_data(self):
        aoa_degrees, cl_values = [], []
        
        cl_max = self.cl_max_plot
        stall_angle_deg = self.stall_angle_plot

        post_stall_end_angle = stall_angle_deg + 7.0
        cl_post_stall = cl_max * (1 - POST_STALL_DROP_FACTOR)

        for i in range(101):
            aoa = (i / 100) * PLOT_MAX_AOA
            aoa_degrees.append(aoa)
            
            if aoa <= stall_angle_deg:
                # A single, smooth sine function tuned by the shape_factor
                normalized_aoa = aoa / stall_angle_deg
                cl = cl_max * math.sin((math.pi / 2) * (normalized_aoa ** self.shape_factor))
            elif stall_angle_deg < aoa <= post_stall_end_angle:
                # Smoothly decay the lift from CL_max to the post-stall value
                progress = (aoa - stall_angle_deg) / (post_stall_end_angle - stall_angle_deg)
                cl = cl_post_stall + (cl_max - cl_post_stall) * (math.cos(math.pi * progress) + 1) / 2
            else:
                cl = cl_post_stall
            cl_values.append(cl)
            
        return aoa_degrees, cl_values

    def run_analysis(self):
        results = {}
        v_min_drag = math.sqrt((2 * self.takeoff_weight_N) / (RHO * self.wing_area)) * \
                     (1 / (math.pi * self.aspect_ratio * OSWALD_EFFICIENCY * C_D0))**0.25
        _, _, min_drag, _ = self.calculate_performance_at_speed(v_min_drag)

        if min_drag > self.total_thrust_N:
            results['feasible'] = False
            results['reason'] = (f"Not feasible. Min thrust needed is {min_drag/1000:.2f} kN, "
                                 f"but only {self.total_thrust_N/1000:.2f} kN is available.")
            return results
        
        results['feasible'] = True
        v_stall_ms = self.calculate_stall_speed()
        results['stall_speed_ms'] = v_stall_ms
        results['stall_speed_kmh'] = v_stall_ms * 3.6
        v_max_ms = self.find_max_speed(v_stall_ms)
        results['max_speed_ms'] = v_max_ms
        results['max_speed_kmh'] = v_max_ms * 3.6
        v_cruise_ms = v_max_ms * 0.85
        cl_cruise, _, drag_cruise, aoa_cruise = self.calculate_performance_at_speed(v_cruise_ms)
        thrust_cruise = drag_cruise
        fuel_flow_kg_hr = (self.tsfc * thrust_cruise) * 3600
        
        results['cruise_analysis'] = {
            "speed_kmh": v_cruise_ms * 3.6, "thrust_required_kN": thrust_cruise / 1000,
            "aoa_deg": aoa_cruise, "lift_coefficient": cl_cruise, "fuel_flow_kg_hr": fuel_flow_kg_hr
        }
        return results

def plot_lift_curve(aoa_data, cl_data, aircraft_name):
    plt.plot(aoa_data, cl_data, label='Lift Curve')
    stall_cl = max(cl_data)
    stall_idx = cl_data.index(stall_cl)
    stall_aoa = aoa_data[stall_idx]
    
    plt.scatter([stall_aoa], [stall_cl], color='red', zorder=5, 
                label=f'Stall Point ($C_{{L_{{max}}}}$={stall_cl:.2f} at {stall_aoa:.1f}°)')
    
    plt.axvline(x=stall_aoa, color='r', linestyle='--', alpha=0.7)
    plt.axhline(y=stall_cl, color='r', linestyle='--', alpha=0.7)
    
    plt.title(f'Lift Curve for {aircraft_name}')
    plt.xlabel('Angle of Attack (Degrees)')
    plt.ylabel('Coefficient of Lift ($C_L$)')
    plt.legend()
    plt.grid(True)
    
    filename = f"lift_curve_{aircraft_name.replace(' ', '_')}.png"
    plt.savefig(filename)
    return filename

def get_user_choice(options, title):
    print(f"\n--- Select {title} ---")
    option_list = list(options.keys())
    for i, item in enumerate(option_list): print(f"  [{i+1}] {item}")
    while True:
        try:
            choice = int(input(f"Enter your choice (1-{len(option_list)}): "))
            if 1 <= choice <= len(option_list):
                key = option_list[choice - 1]
                data = options[key]; data['name'] = key
                return data
            else: print("Invalid choice.")
        except ValueError: print("Invalid input.")

def main():
    print("✈️  Aircraft Performance Analysis Tool ✈️")
    print("="*40)
    aircraft_choice = get_user_choice(AIRCRAFTS, "Aircraft")
    engine_choice = get_user_choice(ENGINES, "Engine")
    
    while True:
        try:
            num_engines = int(input("\nEnter the number of engines: "))
            if num_engines > 0: break
            else: print("Must be positive.")
        except ValueError: print("Invalid input.")
            
    while True:
        try:
            weight_kg = float(input("Enter total takeoff weight in kg (e.g., 75000): "))
            if weight_kg > 0: break
            else: print("Must be positive.")
        except ValueError: print("Invalid input.")

    analyzer = AircraftPerformance(aircraft_choice, engine_choice, num_engines, weight_kg)
    performance_data = analyzer.run_analysis()
    
    print("\n--- PERFORMANCE ANALYSIS RESULTS ---")
    if not performance_data['feasible']:
        print(f"❌ STATUS: NOT FEASIBLE\n   Reason: {performance_data['reason']}")
        sys.exit()
        
    print("✅ STATUS: FEASIBLE")
    print(f"\n[+] Flight Envelope:\n  - Stall Speed: {performance_data['stall_speed_kmh']:.2f} km/h"
          f"\n  - Max Speed (Sea Level): {performance_data['max_speed_kmh']:.2f} km/h")
    
    cruise = performance_data['cruise_analysis']
    print("\n[+] Estimated Cruise Performance (at 85% Max Speed):")
    print(f"  - Cruise Speed: {cruise['speed_kmh']:.2f} km/h"
          f"\n  - Fuel Consumption: {cruise['fuel_flow_kg_hr']:.2f} kg/hour")
    
    print("\n📈 Generating dynamic lift curve plot...")
    aoa_values, cl_values = analyzer.generate_lift_curve_data()
    plot_filename = plot_lift_curve(aoa_values, cl_values, analyzer.aircraft_name)
    print(f"   - Plot saved as '{plot_filename}'")
    print("="*40)

if __name__ == "__main__":
    main()