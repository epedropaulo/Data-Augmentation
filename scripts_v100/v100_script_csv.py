from yade import polyhedra_utils, export, qt
import time
import csv  # Import the CSV module

# Number of simulations to run
number_of_simulations = 3  

# Set up the material and geometry
gravel = PolyhedraMat()
gravel.density = 2600  # kg/m^3
gravel.young = 1E7  # Pa
gravel.poisson = 0.3  # 20000 / 1E7
gravel.frictionAngle = pi / 4  # rad

# Open a file to save the simulation times
with open("simulation_times.txt", "w") as file:
    file.write("Simulation Times Report\n")
    file.write("=======================\n")

# Open a CSV file to save the simulation data
with open("simulation_data.csv", "w", newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    # Write the header for the CSV
    csvwriter.writerow(["Simulation Number", "Box Length", "Simulation Time (seconds)"])

# Variable to keep track of the current simulation number
current_simulation_number = 0
half_lenght_box = 0

# Function to stop the simulation after a specific number of iterations and record the VTK file
def Stop():
    if O.iter >= 700:
        # Export the polyhedral data using VTKExporter with a specified folder and unique file name
        vtkExporter = export.VTKExporter(f'simulation_{current_simulation_number}_{half_lenght_box}_')  # Set the path here
        vtkExporter.exportPolyhedra()
        print(f"Simulation {current_simulation_number} completed and saved.")
        
        O.pause()

# Function to run a single simulation
def run_simulation(simulation_number):
    global current_simulation_number
    current_simulation_number = simulation_number

    global half_lenght_box
    half_lenght_box = 0.04 * (current_simulation_number + 1)
    height = half_lenght_box * 3  # ideal = * 5

    box = geom.facetParallelepiped(
        center=(
            half_lenght_box, 
            half_lenght_box,
            height
        ),
        extents=(
            half_lenght_box,
            half_lenght_box,
            height
        ),
        height=height,
        wallMask=31,
        material=gravel
    )

    O.bodies.append(box)

    polyhedra_utils.fillBox(
        material=gravel,
        seed=1,
        mincoord=(0, 0, 0),
        maxcoord=(
            2*half_lenght_box,
            2*half_lenght_box,
            2*height
        ),
        sizemin=[   # lengths of the bounding box
            0.025,
            0.025,
            0.025
        ],
        sizemax=[
            0.05,
            0.05,
            0.05
        ]
    )

    O.engines = [
        ForceResetter(),
        # approximate collision detection, create interactions | functions of boundary
        InsertionSortCollider([Bo1_Polyhedra_Aabb(), Bo1_Wall_Aabb(), Bo1_Facet_Aabb()]),

        InteractionLoop(
            [  # collisions handler
                Ig2_Wall_Polyhedra_PolyhedraGeom(),
                Ig2_Polyhedra_Polyhedra_PolyhedraGeom(),
                Ig2_Facet_Polyhedra_PolyhedraGeom()
            ],
            [  # collision "physics"
                Ip2_PolyhedraMat_PolyhedraMat_PolyhedraPhys()
            ],
            [  # contact law -- apply forces
                Law2_PolyhedraGeom_PolyhedraPhys_Volumetric()
            ]
        ),
        # GravityEngine(gravity=(0,0,-9.81)),
        NewtonIntegrator(damping=0.3, gravity=(0, 0, -9.81)),

        # Will run the Stop function every 5 real seconds
        PyRunner(command='Stop()', realPeriod=5, label='checker')
    ]

    # Set the timestep
    O.dt = 0.0015

    # Run the simulation and wait for it to finish
    O.run(wait=True)

# Main loop to run multiple simulations
total_start_time = time.time()  # Record the start time of all simulations

for i in range(number_of_simulations):
    print(f"Starting simulation {i}...")
    simulation_start_time = time.time()  # Record the start time of the current simulation
    
    run_simulation(i)
    
    simulation_end_time = time.time()  # Record the end time of the current simulation
    simulation_time = simulation_end_time - simulation_start_time  # Calculate the duration
    print(f"Simulation {i} took {simulation_time:.2f} seconds.")
    
    # Save the simulation time to the text file
    with open("simulation_times.txt", "a") as file:
        file.write(f"Simulation {i} with length {half_lenght_box} took {simulation_time:.2f} seconds.\n")
    
    # Save the simulation time and box length to the CSV file
    with open("simulation_data.csv", "a", newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow([i, half_lenght_box, simulation_time])
    
    O.reset()  # Reset the simulation environment for the next run

# Register and calculate the total duration of the simulations
total_end_time = time.time()
total_time = total_end_time - total_start_time
print(f"All simulations took {total_time:.2f} seconds in total.")

# Save the total time to the text file
with open("simulation_times.txt", "a") as file:
    file.write(f"\nAll simulations took {total_time:.2f} seconds in total.\n")
