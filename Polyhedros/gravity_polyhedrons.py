from yade import polyhedra_utils, qt, export

# Define a function to stop the simulation after 400 iterations and record the VTK file
def Stop():
        if O.iter >= 350:
                O.pause()
                
                # Export the polyhedral data using VTKExporter
                vtkExporter = export.VTKExporter('polyhedra_output_')
                vtkExporter.exportPolyhedra()

gravel = PolyhedraMat()
gravel.density = 2600           # kg/m^ 3
gravel.young = 1E7              # Pa
gravel.poisson = 0.3            # 20000 / 1E7
gravel.frictionAngle = pi/4     # rad

"""box = geom.facetBox(
        center=(0.15, 0.15, 0.15),
        extents=(0.15, 0.15, 0.15),
        wallMask=31,
        material=gravel
        )
"""
height=0.4
box = geom.facetParallelepiped(
        center=(0.15, 0.15, 0.15),
        extents=(0.15, 0.15, height),
        height=height,
        wallMask=31, 
        material=gravel
        )

O.bodies.append(box)

polyhedra_utils.fillBox(
        mincoord=(0, 0, -0.2),
        maxcoord=(0.3, 0.3, 0.55),
        material=gravel,
        seed=4,
        sizemin=[0.025, 0.025, 0.025],
        sizemax=[0.05, 0.05, 0.05]
)

O.engines = [
        ForceResetter(),
        # approximate collision detection, create interactions | functions of boundary
        InsertionSortCollider([Bo1_Polyhedra_Aabb(), Bo1_Wall_Aabb(), Bo1_Facet_Aabb()]),

        InteractionLoop(
                [# collisions handler
                        Ig2_Wall_Polyhedra_PolyhedraGeom(),
                        Ig2_Polyhedra_Polyhedra_PolyhedraGeom(),
                        Ig2_Facet_Polyhedra_PolyhedraGeom()
                ],
                [# collision "physics"
                        Ip2_PolyhedraMat_PolyhedraMat_PolyhedraPhys()
                ],
                [# contact law -- apply forces
                        Law2_PolyhedraGeom_PolyhedraPhys_Volumetric()
                ]
        ),
        # GravityEngine(gravity=(0,0,-9.81)),
        NewtonIntegrator(damping=0.3, gravity=(0, 0, -9.81)),

        # save data for Paraview
        #VTKRecorder(fileName='3d-vtk-', recorders=['all'], firstIterRun=300, iterLast=400, ascii=False),

        # Will run the checkUnbalanced every 5 real seconds
        PyRunner(command='Stop()', realPeriod=5, label='checker')
]

# Set the timestep
O.dt=0.0025

# Open the 3D view
view = qt.View()
