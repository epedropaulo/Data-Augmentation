from yade import polyhedra_utils, export, qt

def Stop():
    if O.iter >= 2500:
        # Export the polyhedral data using VTKExporter with a specified folder and unique file name
        vtkExporter = export.VTKExporter(f'simulation_output')  # Set the path here
        vtkExporter.exportPolyhedra()
        print(f"Simulation completed and saved.")
        
        O.pause()

# Set up the material and geometry
gravel = PolyhedraMat()
gravel.density = 2600  # kg/m^3
gravel.young = 1E7  # Pa
gravel.poisson = 0.3  # 20000 / 1E7
gravel.frictionAngle = pi / 4  # rad

half_lenght_box = 0.1
height = half_lenght_box * 5

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
qt.View()
