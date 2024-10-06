from yade import polyhedra_utils


polyhedra_utils.fillBox(
    (0, 0, 0), (0.3, 0.3, 0.3), defaultMaterial(), 
    sizemin=(0.02, 0.02, 0.02), sizemax=(0.05, 0.05, 0.05), seed=4
)

box = geom.facetBox((0.15, 0.15, 0.15), (0.15, 0.15, 0.15), wallMask=31)

O.bodies.append(box)

O.engines=[
    ForceResetter(),
    InsertionSortCollider(
        [Bo1_Polyhedra_Aabb(), Bo1_Wall_Aabb(), Bo1_Facet_Aabb()]
    ),
    InteractionLoop(
        # handle Poly+Poly and facet+Poly collisions
        [Ig2_Wall_Polyhedra_PolyhedraGeom(), Ig2_Polyhedra_Polyhedra_PolyhedraGeom(), Ig2_Facet_Polyhedra_PolyhedraGeom()],
        [Ip2_PolyhedraMat_PolyhedraMat_PolyhedraPhys()], # collision "physics"
        [Law2_PolyhedraGeom_PolyhedraPhys_Volumetric()] # contact law -- apply forces
    ),
    NewtonIntegrator(damping=0.5, gravity=(0,0,-9.81)),

    # call the checkUnbalanced function (defined below) every 3 seconds
    PyRunner(command='checkUnbalanced()', realPeriod=3, label='checker')
]

def checkUnbalanced():
	if unbalancedForce() < .05:
		O.pause()

O.dt = 0.0005
O.run()
