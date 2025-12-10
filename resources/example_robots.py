import numpy as np
from system import joint, system
from control import control
from resources import example_bodies

def monoped():
    body_length = 1
    body_height = 1
    leg_link_length = 0.8
    foot_radius = 0.1
    body_mass = 2
    leg_mass = 0.5

    body = example_bodies.robot_body(width=body_length, height=body_height, mass=body_mass)
    thigh = example_bodies.leg_link(length=leg_link_length, width=2*foot_radius, mass=leg_mass)
    shin = example_bodies.leg_link(length=leg_link_length, width=2*foot_radius, mass=leg_mass)

    hip = joint.RJoint(body, np.array([0, -body_height/4]),
                       thigh, np.array([-leg_link_length/2, 0]),
                       offset=-np.pi/2)
    knee = joint.RJoint(thigh, np.array([leg_link_length/2, 0]),
                        shin, np.array([-leg_link_length/2, 0]))
    
    model = system.System(body, [hip, knee])
    leg_controller = control.LegController([knee, hip], foot_radius=foot_radius, foot_anchor=np.array([leg_link_length/2, 0]))
    model.controller = control.BodyController(body, [leg_controller], kp=10, kd=10)

    body_y = foot_radius + leg_link_length*np.sqrt(2) - hip.anchor_parent[1] + 1e-5
    model.set_state(0, body_y, 0, [np.pi/4, -np.pi/2])
    
    return model

def wheeled_monoped():
    body_length = 1
    body_height = 1
    leg_link_length = 0.8
    wheel_radius = 0.2
    body_mass = 2
    leg_mass = 0.5
    wheel_mass = 0.2

    body = example_bodies.robot_body(width=body_length, height=body_height, mass=body_mass)
    thigh = example_bodies.leg_link(length=leg_link_length, mass=leg_mass)
    shin = example_bodies.leg_link(length=leg_link_length, mass=leg_mass)
    wheel = example_bodies.wheel(radius=wheel_radius, mass=wheel_mass)

    hip = joint.RJoint(body, np.array([0, -body_height/4]),
                       thigh, np.array([-leg_link_length/2, 0]),
                       offset=-np.pi/2)
    knee = joint.RJoint(thigh, np.array([leg_link_length/2, 0]),
                        shin, np.array([-leg_link_length/2, 0]))
    ankle = joint.RJoint(shin, np.array([leg_link_length/2, 0]),
                         wheel, np.array([0, 0]))
    
    model = system.System(body, [hip, knee, ankle])
    leg_controller = control.LegController([ankle, knee, hip], foot_radius=wheel_radius, foot_anchor=np.array([0, 0]))
    model.controller = control.BodyController(body, [leg_controller])

    body_y = wheel_radius + leg_link_length*np.sqrt(2) - hip.anchor_parent[1] + 1e-5
    model.set_state(0, body_y, 0, [np.pi/4, -np.pi/2, 0])
    
    return model

def biped():
    body_length = 2
    leg_link_length = 0.8
    foot_radius = 0.1
    body_mass = 10
    leg_mass = 2

    body = example_bodies.robot_body(width=body_length, mass=body_mass)
    left_thigh = example_bodies.leg_link(length=leg_link_length, width=2*foot_radius, mass=leg_mass)
    left_shin = example_bodies.leg_link(length=leg_link_length, width=2*foot_radius, mass=leg_mass)
    right_thigh = example_bodies.leg_link(length=leg_link_length, width=2*foot_radius, mass=leg_mass)
    right_shin = example_bodies.leg_link(length=leg_link_length, width=2*foot_radius, mass=leg_mass)

    left_hip = joint.RJoint(body, np.array([-body_length/2+0.2, 0]),
                           left_thigh, np.array([-leg_link_length/2, 0]),
                           offset=-np.pi/2)
    left_knee = joint.RJoint(left_thigh, np.array([leg_link_length/2, 0]),
                            left_shin, np.array([-leg_link_length/2, 0]))
    right_hip = joint.RJoint(body, np.array([body_length/2-0.2, 0]),
                            right_thigh, np.array([-leg_link_length/2, 0]),
                            offset=-np.pi/2)
    right_knee = joint.RJoint(right_thigh, np.array([leg_link_length/2, 0]),
                             right_shin, np.array([-leg_link_length/2, 0]))
    
    model = system.System(body, [left_hip, left_knee, right_hip, right_knee])
    lleg_controller = control.LegController([left_knee, left_hip], foot_radius=foot_radius, foot_anchor=np.array([leg_link_length/2, 0]))
    rleg_controller = control.LegController([right_knee, right_hip], foot_radius=foot_radius, foot_anchor=np.array([leg_link_length/2, 0]))
    model.controller = control.BodyController(body, [lleg_controller, rleg_controller])
    
    body_y = foot_radius + leg_link_length*np.sqrt(2) - left_hip.anchor_parent[1] + 1e-5
    model.set_state(0, body_y, 0, [np.pi/4, -np.pi/2, -np.pi/4, np.pi/2])
    
    return model

def wheeled_biped():
    body_length = 2
    leg_link_length = 0.8
    wheel_radius = 0.2
    body_mass = 10
    leg_mass = 2
    wheel_mass = 0.2

    body = example_bodies.robot_body(width=body_length, mass=body_mass)
    left_thigh = example_bodies.leg_link(length=leg_link_length, mass=leg_mass)
    left_shin = example_bodies.leg_link(length=leg_link_length, mass=leg_mass)
    right_thigh = example_bodies.leg_link(length=leg_link_length, mass=leg_mass)
    right_shin = example_bodies.leg_link(length=leg_link_length, mass=leg_mass)
    left_wheel = example_bodies.wheel(radius=wheel_radius, mass=wheel_mass)
    right_wheel = example_bodies.wheel(radius=wheel_radius, mass=wheel_mass)

    left_hip = joint.RJoint(body, np.array([-body_length/2+0.2, 0]),
                           left_thigh, np.array([-leg_link_length/2, 0]),
                           offset=-np.pi/2)
    left_knee = joint.RJoint(left_thigh, np.array([leg_link_length/2, 0]),
                            left_shin, np.array([-leg_link_length/2, 0]))
    right_hip = joint.RJoint(body, np.array([body_length/2-0.2, 0]),
                            right_thigh, np.array([-leg_link_length/2, 0]),
                            offset=-np.pi/2)
    right_knee = joint.RJoint(right_thigh, np.array([leg_link_length/2, 0]),
                             right_shin, np.array([-leg_link_length/2, 0]))
    left_ankle = joint.RJoint(left_shin, np.array([leg_link_length/2, 0]),
                             left_wheel, np.array([0, 0]),
                             offset=np.pi) # TODO: check this (added recently)
    right_ankle = joint.RJoint(right_shin, np.array([leg_link_length/2, 0]),
                              right_wheel, np.array([0, 0]),
                              offset=np.pi) # TODO: check this (added recently)
    
    model = system.System(body, [left_hip, left_knee, left_ankle, right_hip, right_knee, right_ankle])
    lleg_controller = control.LegController([left_ankle, left_knee, left_hip], foot_radius=wheel_radius, foot_anchor=np.array([leg_link_length/2, 0])) # TODO: shouldn't the foot anchor be 0?
    rleg_controller = control.LegController([right_ankle, right_knee, right_hip], foot_radius=wheel_radius, foot_anchor=np.array([leg_link_length/2, 0])) # TODO: shouldn't the foot anchor be 0?
    model.controller = control.BodyController(body, [lleg_controller, rleg_controller])

    body_y = wheel_radius + leg_link_length*np.sqrt(2) - left_hip.anchor_parent[1] + 1e-5
    model.set_state(0, body_y, 0, [np.pi/4, -np.pi/2, 0, -np.pi/4, np.pi/2, 0])
    
    return model

def triped():
    body_length = 2.5
    body_height = 0.8
    leg_link_length = 0.8
    foot_radius = 0.1
    body_mass = 15
    leg_mass = 2

    body = example_bodies.robot_body(width=body_length, height=body_height, mass=body_mass)
    left_thigh = example_bodies.leg_link(length=leg_link_length, width=2*foot_radius, mass=leg_mass)
    left_shin = example_bodies.leg_link(length=leg_link_length, width=2*foot_radius, mass=leg_mass)
    middle_thigh = example_bodies.leg_link(length=leg_link_length, width=2*foot_radius, mass=leg_mass)
    middle_shin = example_bodies.leg_link(length=leg_link_length, width=2*foot_radius, mass=leg_mass)
    right_thigh = example_bodies.leg_link(length=leg_link_length, width=2*foot_radius, mass=leg_mass)
    right_shin = example_bodies.leg_link(length=leg_link_length, width=2*foot_radius, mass=leg_mass)

    left_hip = joint.RJoint(body, np.array([-body_length/2+0.2, -body_height/4]),
                           left_thigh, np.array([-leg_link_length/2, 0]),
                           offset=-np.pi/2)
    left_knee = joint.RJoint(left_thigh, np.array([leg_link_length/2, 0]),
                            left_shin, np.array([-leg_link_length/2, 0]))
    middle_hip = joint.RJoint(body, np.array([0, -body_height/4]),
                           middle_thigh, np.array([-leg_link_length/2, 0]),
                           offset=-np.pi/2)
    middle_knee = joint.RJoint(middle_thigh, np.array([leg_link_length/2, 0]),
                            middle_shin, np.array([-leg_link_length/2, 0]))
    right_hip = joint.RJoint(body, np.array([body_length/2-0.2, -body_height/4]),
                            right_thigh, np.array([-leg_link_length/2, 0]),
                            offset=-np.pi/2)
    right_knee = joint.RJoint(right_thigh, np.array([leg_link_length/2, 0]),
                             right_shin, np.array([-leg_link_length/2, 0]))
    
    model = system.System(body, [left_hip, left_knee, middle_hip, middle_knee, right_hip, right_knee])
    lleg_controller = control.LegController([left_knee, left_hip], foot_radius=foot_radius, foot_anchor=np.array([leg_link_length/2, 0]))
    mleg_controller = control.LegController([middle_knee, middle_hip], foot_radius=foot_radius, foot_anchor=np.array([leg_link_length/2, 0]))
    rleg_controller = control.LegController([right_knee, right_hip], foot_radius=foot_radius, foot_anchor=np.array([leg_link_length/2, 0]))
    model.controller = control.BodyController(body, [lleg_controller, mleg_controller, rleg_controller])
    
    body_y = foot_radius + leg_link_length*np.sqrt(2) - left_hip.anchor_parent[1] + 1e-5
    model.set_state(0, body_y, 0, [np.pi/4, -np.pi/2, np.pi/4, -np.pi/2, -np.pi/4, np.pi/2])
    
    return model

