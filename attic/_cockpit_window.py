"""Scratch: render the cockpit view and save it, for a human to look at."""
import os
import sys

import pygame
from OpenGL.GL import GL_RGB, GL_UNSIGNED_BYTE, glFinish, glReadPixels

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import main
from simulation import ChaseCamera, SIM_TIME_STEP_S, Simulation

HERE = os.path.dirname(os.path.abspath(__file__))


def shot(name, sim, camera, view):
    main.draw_scene(sim, camera, view)
    glFinish()
    data = glReadPixels(0, 0, main.WIDTH, main.HEIGHT, GL_RGB, GL_UNSIGNED_BYTE)
    image = pygame.image.frombuffer(data, (main.WIDTH, main.HEIGHT), "RGB")
    path = os.path.join(HERE, name)
    pygame.image.save(pygame.transform.flip(image, False, True), path)
    print("wrote %s" % (path,))


pygame.init()
pygame.display.set_mode((main.WIDTH, main.HEIGHT),
                        main.DOUBLEBUF | main.OPENGL | main.HIDDEN)
main.resize(main.WIDTH, main.HEIGHT)
main.init_gl()
main.load_scenery(main.DEFAULT_SCENERY_FILE)

sim = Simulation(airframe=main.airframe_preset(main.AIRFRAME_PRESET))
sim.on_the_pad()
camera = ChaseCamera()
main.fly_frame(sim, camera, SIM_TIME_STEP_S, {})
shot("_cockpit_pad.png", sim, camera, main.VIEW_COCKPIT)

# The same frame with the rotors left out, to show they are what is up there.
rotors = (main.draw_main_rotor, main.draw_tail_rotor)
main.draw_main_rotor = lambda azimuth_deg: None
main.draw_tail_rotor = lambda azimuth_deg: None
shot("_cockpit_norotor.png", sim, camera, main.VIEW_COCKPIT)
main.draw_main_rotor, main.draw_tail_rotor = rotors

trim_fraction = sim.trim_controls.collective_fraction
for _ in range(int(12.0 / SIM_TIME_STEP_S)):
    keys = ({pygame.KSCAN_W: True}
            if sim.pilot.collective_axis < trim_fraction else {})
    main.fly_frame(sim, camera, SIM_TIME_STEP_S, keys)
print("hovering: %s" % (sim.telemetry(),))
shot("_cockpit_air.png", sim, camera, main.VIEW_COCKPIT)
shot("_cockpit_chase.png", sim, camera, main.VIEW_CHASE)

pygame.quit()
