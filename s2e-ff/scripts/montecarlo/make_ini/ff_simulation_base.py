import pathlib
import numpy as np

def echo():
    with open(pathlib.Path(__file__).parent.parent.parent.parent/"data/initialize_files/ff_simulation_base.ini", 'w') as f:
        print(
"""[TIME]
// Simulation start time [UTC]
simulation_start_time_utc = 2024/01/01 12:00:00.0

// Simulation duration [sec]
simulation_duration_s = 100

// Simulation step time [sec]
// Minimum time step for the entire simulation
simulation_step_s = 0.1

// Attitude Update Period [sec]
// Attitude is updated at the period specified here 
attitude_update_period_s = 0.1 // should be larger than 'simulation_step_s'

// Attitide Δt for Runge-Kutt method [sec]
// This must be smaller than 'attitude_update_period_s' 
attitude_integral_step_s = 0.001

// Orbit Update Period [sec]
// Orbit is updated at the period specified here
orbit_update_period_s = 0.1 // should be larger than 'simulation_step_s'

// Orbit Δt for Runge-Kutta method [sec]
// This must be smaller than 'orbit_ppdate_period_s'
orbit_integral_step_s = 0.1 

// Thermal Update Period [sec]
// Thermal is updated at the period specified here
thermal_update_period_s = 0.1 // should be larger than 'simulation_step_s'

// Thermal Δt for Runge-Kutta method [sec]
// This must be smaller than 'thermal_update_period_s'
thermal_integral_step_s = 0.1

// Component Update Period [sec]
component_update_period_s = 0.1 // should be larger than 'simulation_step_s'

// Log Output Period [sec]
log_output_period_s = 0.1   // should be larger than 'simulation_step_s'

// Simulation speed
// 0: as fast as possible, 1: real-time, >1: faster than real-time, <1: slower than real-time
simulation_speed_setting = 0


[CELESTIAL_INFORMATION]
// Whether global celestial information is logged or not
logging = ENABLE

// Definition of Inertial frame
inertial_frame = J2000
// The center object is also used to define the gravity constant of the center body
center_object = EARTH
aberration_correction = NONE

// Earth Rotation model
// Idle:no motion, Simple:rotation only, Full:full-dynamics
rotation_mode = Simple

// Definition of calculation celestial bodies
number_of_selected_body = 3
selected_body_name(0) = EARTH
selected_body_name(1) = SUN
selected_body_name(2) = MOON
selected_body_name(3) = MARS


[CSPICE_KERNELS]
// CSPICE Kernel files definition
tls  = ../../../ExtLibraries/cspice/generic_kernels/lsk/naif0010.tls
tpc1 = ../../../ExtLibraries/cspice/generic_kernels/pck/de-403-masses.tpc
tpc2 = ../../../ExtLibraries/cspice/generic_kernels/pck/gm_de431.tpc
tpc3 = ../../../ExtLibraries/cspice/generic_kernels/pck/pck00010.tpc
bsp  = ../../../ExtLibraries/cspice/generic_kernels/spk/planets/de430.bsp


[HIPPARCOS_CATALOGUE]
catalogue_file_path = ../../../ExtLibraries/HipparcosCatalogue/hip_main.csv
max_magnitude = 3.0	// Max magnitude to read from Hip catalog
calculation = DISABLE
logging = DISABLE


[RANDOMIZE]
// Seed of randam. When this value is 0, the seed will be varied by time.
rand_seed = {0}

[SIMULATION_SETTINGS]
// Whether the ini files are saved or not
save_initialize_files = ENABLE

// Initialize files
// File name must not over 1024 characters (defined in initialize_file_access.hpp as kMaxCharLength)
// If you want to add a spacecraft, create the corresponding spacecraft.ini, and specify it as spacecraft_file(1), spacecraft_file(2), ect.
// If you want to add a ground station, create the corresponding ground_station.ini, and specify it as ground_station_file(1), ground_station_file(2), ect.
number_of_simulated_spacecraft = 2
spacecraft_file(0)   = ../../data/initialize_files/ff_satellite.ini
spacecraft_file(1)   = ../../data/initialize_files/ff_satellite_2.ini
log_file_save_directory = ../../data/logs/
"""
        .format(
            np.random.randint(0, 63355, dtype=np.uint32)
        ),
        file=f)