from yade import polyhedra_utils

gravel = PolyhedraMat()
gravel.density = 2600  #kg/m^3
gravel.young = 1E7  #Pa
gravel.poisson = 0.3#20000 / 1E7
gravel.frictionAngle = pi/4  #rad

box = geom.facetBox((0.15, 0.15, 0.15), (0.15, 0.15, 0.15), wallMask=31, material=gravel)
O.bodies.append(box)

polyhedra_utils.fillBox(
        (0, 0, 0), (0.3, 0.3, 0.3), gravel, seed=4,
        sizemin=[0.025, 0.025, 0.025], sizemax=[0.05, 0.05, 0.05]
)

def checkUnbalanced():
	if unbalancedForce() < .05:
		O.pause()

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
                        Ip2_PolyhedraMat_PolyhedraMat_PolyhedraPhys(),
                        Ip2_FrictMat_PolyhedraMat_FrictPhys(),
                        Ip2_FrictMat_FrictMat_FrictPhys()
                ],
                [# contact law -- apply forces
                        Law2_PolyhedraGeom_PolyhedraPhys_Volumetric(),
                        Law2_ScGeom_FrictPhys_CundallStrack()
                ]
        ),
        # GravityEngine(gravity=(0,0,-9.81)),
        NewtonIntegrator(damping=0.3, gravity=(0, 0, -9.81)),

        # Will run the checkUnbalanced every 5 real seconds
        PyRunner(command='checkUnbalanced()', realPeriod=5, label='checker')
]

O.dt=0.0025
