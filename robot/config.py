import numpy as np

# --- Ustawienia Ścieżek ---
URDF_PATH = 'assets/PAROL6.urdf'
STL_FILES = [
    'base_link_p.ply', 'L1_p.ply', 'L2_p.ply', 'L3_p.ply', 
    'L4_p.ply', 'L5_p.ply', 'Stepper_Gripper_p.ply'
]
LINK_NAMES = [
    'base_link', 'L1', 'L2', 'L3', 'L4', 'L5', 'L6'
]

# --- Ustawienia Kinematyki ---
# Maska: False dla stałej bazy, True dla 6 ruchomych osi
ACTIVE_LINKS_MASK = [False, True, True, True, True, True, True]
# Macierz transformacji do sceny 3D (flip Y)
S_MATRIX = np.diag([1.0, -1.0, 1.0, 1.0])
# Pozycja startowa (w stopniach)
STANDBY_ANGLES_DEG = [0, 0, 0, 0, 0, 0]

# --- Ustawienia Animacji ---
ANIM_STEPS_TOTAL = 15
ANIM_INTERVAL_MS = 30
DEBOUNCE_MS = 20

# --- Ustawienia GUI ---
SLIDER_RANGE_MIN = 0
SLIDER_RANGE_MAX = 1000
# Domyślne wartości XYZ w GUI (w układzie użytkownika)
DEFAULT_XYZ_GUI = [0.0, 150.0, 250.0] 
RANGES_XYZ = [(-300.0, 300.0), (-300.0, 300.0), (0.0, 500.0)]
RANGES_RPY = (-360.0, 360.0)
