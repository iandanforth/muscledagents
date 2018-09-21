import sys
import random
import numpy as np
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)
import pymunk
import pymunk.pygame_util

def add_ball(space):
    mass = 1
    radius = 14
    moment = pymunk.moment_for_circle(mass, 0, radius)
    body = pymunk.Body(mass, moment)
    x = random.randint(120, 380)
    body.position = x, 550
    shape = pymunk.Circle(body, radius)
    shape.friction = 1.0
    space.add(body, shape)
    return shape


def add_arm(space, config={}):

    default_config = {
        "arm_center": (300, 300),
        "lower_arm_length": 200,
        "lower_arm_starting_angle": 15,
        "lower_arm_mass": 10,
        "brach_rest_length": 5,
        "brach_stiffness": 450,
        "brach_damping": 200,
        "tricep_rest_length": 30,
        "tricep_stiffness": 50,
        "tricep_damping": 400
    }

    # Like dict.update() if it had an overwrite=False option
    config.update({k: v for k, v in default_config.items() if k not in list(config.keys())})

    # Upper Arm
    upper_arm_length = 200
    upper_arm_body = pymunk.Body(body_type=pymunk.Body.STATIC)
    upper_arm_body.position = config["arm_center"]
    upper_arm_body.angle = np.deg2rad(-45)
    upper_arm_line = pymunk.Segment(upper_arm_body, (0, 0), (-upper_arm_length, 0), 5)
    upper_arm_line.sensor = True  # Disable collision

    space.add(upper_arm_body)
    space.add(upper_arm_line)

    # Lower Arm
    lower_arm_body = pymunk.Body(0, 0)  # Pymunk will calculate moment based on mass of attached shape
    lower_arm_body.position = config["arm_center"]
    lower_arm_body.angle = np.deg2rad(config["lower_arm_starting_angle"])
    elbow_extension_length = 20
    lower_arm_start = (-elbow_extension_length, 0)
    lower_arm_line = pymunk.Segment(
        lower_arm_body,
        lower_arm_start,
        (config["lower_arm_length"], 0),
        5
    )
    lower_arm_line.mass = config["lower_arm_mass"]
    lower_arm_line.friction = 1.0

    space.add(lower_arm_body)
    space.add(lower_arm_line)

    # Pivot (Elbow)
    elbow_body = pymunk.Body(body_type=pymunk.Body.STATIC)
    elbow_body.position = config["arm_center"]
    elbow_joint = pymunk.PivotJoint(elbow_body, lower_arm_body, config["arm_center"])
    space.add(elbow_joint)

    # Spring (Brachialis Muscle)
    brach_spring = pymunk.constraint.DampedSpring(
        upper_arm_body,
        lower_arm_body,
        (-(upper_arm_length * (1 / 2)), 0),  # Connect half way up the upper arm
        (config["lower_arm_length"] / 5, 0),  # Connect near the bottom of the lower arm
        config["brach_rest_length"],
        config["brach_stiffness"],
        config["brach_damping"]
    )
    space.add(brach_spring)

    # Spring (Tricep Muscle)
    tricep_spring = pymunk.constraint.DampedSpring(
        upper_arm_body,
        lower_arm_body,
        (-(upper_arm_length * (3 / 4)), 0),
        lower_arm_start,
        config["tricep_rest_length"],
        config["tricep_stiffness"],
        config["tricep_damping"]
    )
    space.add(tricep_spring)

    # Elbow stop (prevent under/over extension)
    elbow_stop_point = pymunk.Circle(
        upper_arm_body,
        radius=5,
        offset=(-elbow_extension_length, -3)
    )
    elbow_stop_point.friction = 1.0
    space.add(elbow_stop_point)

    return brach_spring, tricep_spring


def main():
    pygame.init()
    screen_width = screen_height = 600
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Curl Sim")
    clock = pygame.time.Clock()

    space = pymunk.Space()
    space.gravity = (0.0, -900.0)

    config = {
        "arm_center": (screen_width / 2, screen_height / 2),
        "lower_arm_length": 170,
    }
    brach, tricep = add_arm(space, config)

    ####################### Render Loop #####################
    debug = True
    ball_rain = False
    balls = []
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    draw_options.flags = 1  # Disable constraint drawing
    if debug:
        draw_options.flags = 3  # Enable constraint drawing

    ticks_to_next_ball = 10
    frames = 0
    stiffness_delta = 50
    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit(0)
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                sys.exit(0)

        ticks_to_next_ball -= 1
        if ticks_to_next_ball <= 0 and ball_rain:
            ticks_to_next_ball = 25
            ball_shape = add_ball(space)
            balls.append(ball_shape)
            if len(balls) > 10:
                space.remove(balls.pop(0))

        # Advance the simulation
        space.step(1/50.0)

        # Redraw all objects
        screen.fill((255, 255, 255))
        space.debug_draw(draw_options)
        pygame.display.flip()

        clock.tick(50)
        frames += 1

        #####################################################
        # Sensorimotor

        # Get sensory data

        # Activate motor units

        #####################################################
        # Vary world conditions
        if frames % 50 == 0:
            if debug:
                print("Tricep spring stiffness: ", tricep.stiffness)
            tricep.stiffness += stiffness_delta

        if frames % 1000 == 0:
            stiffness_delta *= -1


if __name__ == '__main__':
    main()
