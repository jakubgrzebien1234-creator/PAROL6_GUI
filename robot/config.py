import numpy as np


URDF_PATH = 'assets/PAROL6.urdf'


STL_FILES = [
    'BaseMAIN.ply', 'L1_p.ply', 'L2_p.ply', 'L3_p.ply', 
    'L4_p.ply', 'L5_p.ply', 'VacGrip_p.ply'
]

BASE_EXTRA_FILES = [
     'Estop.ply',
     'supply.ply',
     'dell.ply',
     'pumpstand.ply',
     'pump.ply'
]

BASE_EXTRA_OFFSETS = {
    'pump.ply': (0, 0, -0.02), 
}

LINK_NAMES = [
    'base_link', 'L1', 'L2', 'L3', 'L4', 'L5', 'L6'
]
ACTIVE_LINKS_MASK = [False, True, True, True, True, True, True, False]

S_MATRIX = np.diag([1.0, 1.0, 1.0, 1.0])
STANDBY_ANGLES_DEG = [0, 0, 0, 0, 0, 0]
ANIM_STEPS_TOTAL = 100
ANIM_INTERVAL_MS = 30
DEBOUNCE_MS = 20

SLIDER_RANGE_MIN = 0
SLIDER_RANGE_MAX = 1000
DEFAULT_XYZ_GUI = [0.0, 150.0, 250.0] 
RANGES_XYZ = [(-500.0, 600.0), (-550.0, 550.0), (0.0, 600.0)]
RANGES_RPY = (-360.0, 360.0)

TOOL_STL_MAP = {
    "CHWYTAK_MALY": "VacGrip_p.ply",       
    "CHWYTAK_DUZY": "Stepper_Gripper.ply", 
}

