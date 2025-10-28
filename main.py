import sys
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph.opengl as gl
import numpy as np
from ikpy.chain import Chain
import trimesh
import xml.etree.ElementTree as ET
from PyQt5.QtGui import QMatrix4x4, QVector4D, QVector3D
from OpenGL import GL

# === IMPORTUJEMY TWÓJ MODUŁ KOMUNIKACYJNY ===
import serial.tools.list_ports
import robot.communication as comm
# ============================================


def matrix_to_qtransform(matrix):
    """Konwertuje macierz transformacji 4x4 NumPy na QMatrix4x4."""
    m = QMatrix4x4()
    for i in range(4):
        m.setRow(i, QVector4D(float(matrix[i, 0]), float(matrix[i, 1]), float(matrix[i, 2]), float(matrix[i, 3])))
    return m


# --- Wczytanie URDF ---
urdf_path = 'assets/PAROL6.urdf'
try:
    chain = Chain.from_urdf_file(urdf_path)
    chain.max_iterations = 200
    chain.convergence_limit = 1e-4
    print("URDF wczytany pomyślnie.")
except Exception as e:
    print(f"Błąd wczytywania URDF: {e}")
    sys.exit(1)


# --- Wczytanie originów visual z URDF ---
tree = ET.parse(urdf_path)
root = tree.getroot()
visual_origins = {}
for link in root.findall('link'):
    name = link.attrib['name']
    visual = link.find('visual')
    if visual is not None:
        origin = visual.find('origin')
        if origin is not None:
            xyz = [float(x) for x in origin.attrib.get('xyz', '0 0 0').split()]
            rpy = [float(r) for r in origin.attrib.get('rpy', '0 0 0').split()]
            visual_origins[name] = (xyz, rpy)


# --- Lista STL i nazw linków ---
stl_files = ['base_link_p.ply', 'L1_p.ply', 'L2_p.ply', 'L3_p.ply', 'L4_p.ply', 'L5_p.ply', 'Stepper_Gripper_p.ply']
link_names = ['base_link', 'L1', 'L2', 'L3', 'L4', 'L5', 'L6']


# --- Aplikacja Qt ---
app = QtWidgets.QApplication(sys.argv)
window = QtWidgets.QWidget()
window.setWindowTitle("PAROL6 3D GUI – sterowanie XYZ + UART")
main_layout = QtWidgets.QHBoxLayout(window)

# --- Panel sterowania ---
control_widget = QtWidgets.QWidget()
control_widget.setFixedWidth(200)
control_layout = QtWidgets.QVBoxLayout(control_widget)

# === SEKCJA UART ===
uart_group = QtWidgets.QGroupBox("Komunikacja UART")
uart_layout = QtWidgets.QVBoxLayout()
port_combo = QtWidgets.QComboBox()
available_ports = [port.device for port in serial.tools.list_ports.comports()]
port_combo.addItems(available_ports if available_ports else ["Brak portów"])
uart_layout.addWidget(QtWidgets.QLabel("Port COM:"))
uart_layout.addWidget(port_combo)
uart_status_label = QtWidgets.QLabel("Status: Gotowy do wysyłania")
uart_status_label.setStyleSheet("color: green;")
uart_layout.addWidget(uart_status_label)
uart_group.setLayout(uart_layout)
control_layout.addWidget(uart_group)


# --- Panel XYZ ---

xyz_group = QtWidgets.QGroupBox("Cel efektora (mm)")
xyz_layout = QtWidgets.QGridLayout()
xyz_inputs = {}

labels_xyz = ['X', 'Y', 'Z']
ranges_xyz = [(-300.0, 300.0), (-300.0, 300.0), (0.0, 500.0)]

for i, label in enumerate(labels_xyz):
    lbl = QtWidgets.QLabel(f"{label}:")
    inp = QtWidgets.QLineEdit()
    validator = QtGui.QDoubleValidator(ranges_xyz[i][0], ranges_xyz[i][1], 3)
    validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
    inp.setValidator(validator)
    mid_val = (ranges_xyz[i][0] + ranges_xyz[i][1]) / 2.0
    inp.setText(f"{mid_val:.1f}")
    xyz_layout.addWidget(lbl, i, 0)
    xyz_layout.addWidget(inp, i, 1)
    xyz_inputs[label] = inp


set_xyz_btn = QtWidgets.QPushButton("Ustaw Cel (XYZ)")
xyz_layout.addWidget(set_xyz_btn, len(labels_xyz), 0, 1, 2)
xyz_group.setLayout(xyz_layout)
control_layout.addWidget(xyz_group)

# Standby i reset kamery
btn_layout = QtWidgets.QHBoxLayout()
standby_btn = QtWidgets.QPushButton("Pozycja standby")
btn_layout.addWidget(standby_btn)
reset_view_btn = QtWidgets.QPushButton("Reset kamery")
btn_layout.addWidget(reset_view_btn)
control_layout.addLayout(btn_layout)

control_layout.addStretch()
main_layout.addWidget(control_widget)


# --- Widget 3D ---
view = gl.GLViewWidget()
view.opts['center'] = QVector3D(0, 0, 0.2)
view.opts['msaa'] = True
ax = gl.GLAxisItem()
ax.setSize(0.3, 0.3, 0.3)
view.addItem(ax)

view.setCameraPosition(distance=0.7, elevation=30, azimuth=25)
view.setMinimumSize(800, 600)
view.setBackgroundColor((40, 40, 40))
main_layout.addWidget(view)

# Eksperymentuj z tą wartością, aby znaleźć idealną "delikatność"
# 1.0 = oryginalny, ciemny
# 2.0 - 2.5 = delikatny, mleczny, niski kontrast
# 5.0 = jasny, wysoki kontrast, przepalony
DELICATE_MULTIPLIER = 1
links = []
for stl_file in stl_files:
    mesh_data = trimesh.load(f'assets/{stl_file}')
    mesh_data.fix_normals()
    
    vertex_colors_normalized = mesh_data.visual.vertex_colors.astype(np.float32) / 255.0
    
    # Zastosuj delikatny mnożnik
    delicate_vertex_colors = vertex_colors_normalized * DELICATE_MULTIPLIER

    mesh_item = gl.GLMeshItem(
        vertexes=np.array(mesh_data.vertices),
        faces=np.array(mesh_data.faces), 
        vertexColors=delicate_vertex_colors[:, :3], 
        shader='shaded',
        
        # NAJWAŻNIEJSZA ZMIANA: Włącz gładkie cieniowanie
        smooth=False, 
        
        drawFaces=True
    )
    
    view.addItem(mesh_item)
    links.append(mesh_item)


# --- Pozycja STANDBY ---
standby_angles_deg = [0, 0, 0, 0, 0, 0]
standby_angles = np.array([0] + [np.radians(a) for a in standby_angles_deg])

previous_angles = standby_angles.copy()
target_angles = None
animation_timer = None
anim_step = 0
anim_steps_total = 15
anim_interval_ms = 30
debounce_ms = 20


def apply_transforms_from_angles(angles):
    transforms = chain.forward_kinematics(angles, full_kinematics=True)
    for i, name in enumerate(link_names):
        mesh = links[i]
        T = transforms[i]
        if name in visual_origins:
            xyz_offset, rpy_offset = visual_origins[name]
            T_origin = np.eye(4)
            from scipy.spatial.transform import Rotation as R
            T_origin[:3, :3] = R.from_euler('xyz', rpy_offset).as_matrix()
            T_origin[:3, 3] = xyz_offset
            T = T @ T_origin
        mesh.setTransform(matrix_to_qtransform(T))
    view.update()


def read_target_pose_inputs():
    """Odczytuje pozycję XYZ (mm) i zwraca w metrach."""
    try:
        X_mm = float(xyz_inputs['X'].text())
        Y_mm = float(xyz_inputs['Y'].text())
        Z_mm = float(xyz_inputs['Z'].text())
    except Exception as e:
        print(f"Błąd odczytu danych wejściowych: {e}")
        return np.array([0, 0, 0.25])
    return np.array([X_mm / 1000.0, Y_mm / 1000.0, Z_mm / 1000.0])


def perform_update_robot():
    """Oblicza IK tylko dla XYZ."""
    global previous_angles, target_angles, anim_step, animation_timer

    target_pos_m = read_target_pose_inputs()

    try:
        angles_rad = chain.inverse_kinematics(
            target_position=target_pos_m,
            target_orientation=None,
            orientation_mode=None,
            initial_position=previous_angles
        )
        print("IK znalezione (rad):", np.round(angles_rad, 4))
    except Exception as e:
        print(f"Błąd IK: {e}")
        uart_status_label.setText("Status: Błąd IK")
        uart_status_label.setStyleSheet("color: red;")
        return

    target_angles = angles_rad
    anim_step = 0

    if animation_timer is None or not animation_timer.isActive():
        animation_timer = QtCore.QTimer()
        animation_timer.timeout.connect(step_animation)
        animation_timer.start(anim_interval_ms)
    else:
        anim_step = 0


def step_animation():
    """Interpolacja XYZ."""
    global anim_step, anim_steps_total, previous_angles, target_angles, animation_timer

    if target_angles is None:
        if animation_timer:
            animation_timer.stop()
            animation_timer = None
        uart_status_label.setText("Status: Osiągnięto cel")
        uart_status_label.setStyleSheet("color: green;")
        return

    anim_step += 1
    t = anim_step / anim_steps_total
    interp_angles = previous_angles + t * (target_angles - previous_angles)
    apply_transforms_from_angles(interp_angles)

    selected_port = port_combo.currentText()
    if selected_port != "Brak portów":
        angles_to_send_deg = np.degrees(interp_angles[1:])
        try:
            comm.send_angles(selected_port, angles_to_send_deg)
            uart_status_label.setText(f"Status: W ruchu ({anim_step}/{anim_steps_total})")
            uart_status_label.setStyleSheet("color: blue;")
        except Exception as e:
            print(f"Błąd UART: {e}")
            uart_status_label.setText("Status: Błąd UART")
            uart_status_label.setStyleSheet("color: red;")
            if animation_timer:
                animation_timer.stop()
            animation_timer = None
            return
    else:
        uart_status_label.setText("Status: Brak portu COM")
        uart_status_label.setStyleSheet("color: orange;")

    if anim_step >= anim_steps_total:
        previous_angles = target_angles.copy()
        target_angles = None
        if animation_timer:
            animation_timer.stop()
            animation_timer = None
        uart_status_label.setText("Status: Osiągnięto cel")
        uart_status_label.setStyleSheet("color: green;")


update_pending = False
def schedule_update():
    global update_pending
    if update_pending:
        return
    update_pending = True
    QtCore.QTimer.singleShot(debounce_ms, lambda: (perform_update_robot(), set_update_pending_false()))


def set_update_pending_false():
    global update_pending
    update_pending = False


def go_standby():
    global previous_angles, target_angles, animation_timer
    target_angles = standby_angles.copy()
    anim_step = 0
    if animation_timer is None or not animation_timer.isActive():
        animation_timer = QtCore.QTimer()
        animation_timer.timeout.connect(step_animation)
        animation_timer.start(anim_interval_ms)
    else:
        anim_step = 0
    uart_status_label.setText("Status: Standby")
    uart_status_label.setStyleSheet("color: blue;")


def reset_camera():
    view.setCameraPosition(distance=0.7, elevation=30, azimuth=25)


set_xyz_btn.clicked.connect(schedule_update)
for inp in xyz_inputs.values():
    inp.returnPressed.connect(schedule_update)
# ⬆️ ⬆️ ⬆️ ------------------- ⬆️ ⬆️ ⬆️

standby_btn.clicked.connect(go_standby)
reset_view_btn.clicked.connect(reset_camera)
window.show()
QtCore.QTimer.singleShot(100, go_standby)
app.aboutToQuit.connect(comm.close_serial_port)
sys.exit(app.exec_())