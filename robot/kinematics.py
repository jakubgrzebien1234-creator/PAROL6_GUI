import numpy as np
import warnings
from ikpy.chain import Chain
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

# === TOOL DICTIONARY ===
ROBOT_TOOLS = {
    "CHWYTAK_MALY": {
        # PROSTY CHWYTAK (W OSI):
        # X = 0 (Brak przesunięcia w bok)
        # Y = 0 (Brak przesunięcia w bok)
        # Z = Długość całkowita (np. 30mm bazy + 70mm końcówki = 10cm = 0.10)
        
        # Wpisz tutaj CAŁKOWITĄ długość od tarczy robota do czubka chwytaka:
        "translation": [0.0, 0.0, 0.10],  
        
        "orientation": [0.0, 0.0, 0.0]
    },
    
    "CHWYTAK_DUZY": {
        "translation": [0.0, 0.0, 0.15],   
        "orientation": [0.0, 0.0, 0.0]
    }
}

class RobotKinematics:
    def __init__(self, urdf_path, active_links_mask=None):
        self.chain = None
        self.urdf_path = urdf_path
        self.n_active_joints = 6
        self.visual_origins = {}
        self.joint_limits_rad = [(-np.pi, np.pi)] * 6
        
        self.tool_translation = np.zeros(3) 
        self.tool_rotation_matrix = np.eye(3) 
        self.current_tool = "NONE"

        try:
            print(f"[IK] Loading URDF: {urdf_path}")
            
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", UserWarning)
                self.chain = Chain.from_urdf_file(urdf_path)
            
            # Automatyczna maska
            mask = []
            for link in self.chain.links:
                if link.joint_type == 'fixed':
                    mask.append(False)
                else:
                    mask.append(True)
            
            self.chain.active_links_mask = mask
            self.active_links_mask = mask
            
            # === ZWIĘKSZONA PRECYZJA ===
            self.chain.max_iterations = 500  # Było 100
            self.chain.convergence_limit = 1e-6
            
            self.joint_limits_rad = self._load_active_joint_limits()
            self.visual_origins = self._load_visual_origins(urdf_path)
            
            self.set_tool("CHWYTAK_MALY")
            
            print(f"[IK] Ready. Mask: {mask}")

        except Exception as e:
            print(f"[IK ERROR] {e}")
            self._setup_mock_chain()

    def set_tool(self, tool_name):
        if tool_name not in ROBOT_TOOLS:
            print(f"[IK] Unknown tool: {tool_name}")
            return

        tool_data = ROBOT_TOOLS[tool_name]
        self.tool_translation = np.array(tool_data["translation"])
        rpy = tool_data.get("orientation", [0,0,0])
        self.tool_rotation_matrix = R.from_euler('xyz', rpy).as_matrix()
        
        self.current_tool = tool_name
        print(f"[IK] Tool Set: {tool_name} -> Offset: {self.tool_translation}")

    # ================= KINEMATYKA (DEBUGOWANA) =================

    def forward_kinematics(self, active_angles):
        full_joints = self._active_to_full(active_angles)
        flange_matrix = self.chain.forward_kinematics(full_joints)
        
        R_flange = flange_matrix[:3, :3]
        P_flange = flange_matrix[:3, 3]
        
        # P_tcp = P_flange + (R_flange * Offset)
        offset_global = R_flange @ self.tool_translation
        P_tcp = P_flange + offset_global
        
        tcp_matrix = np.eye(4)
        tcp_matrix[:3, 3] = P_tcp
        tcp_matrix[:3, :3] = R_flange @ self.tool_rotation_matrix
        
        return tcp_matrix

    def inverse_kinematics(self, target_position, target_orientation, initial_guess=None):
        if initial_guess is None: initial_guess = np.zeros(6)
        
        # 1. Obliczamy orientację flanszy
        target_rot_matrix = target_orientation 
        flange_rot_matrix = target_rot_matrix @ np.linalg.inv(self.tool_rotation_matrix)
        
        # 2. Obliczamy pozycję flanszy (Cofamy się o wektor narzędzia)
        # WAŻNE: Wektor narzędzia jest obracany zgodnie z DOCELOWĄ rotacją flanszy
        offset_global = flange_rot_matrix @ self.tool_translation
        target_pos_flange = target_position - offset_global
        
        # === DEBUGGING (Wypisuje w konsoli przy każdym ruchu) ===
        # Odkomentuj jeśli chcesz widzieć obliczenia
        # print(f"DEBUG IK: TCP Target={target_position}")
        # print(f"DEBUG IK: Offset Global={offset_global}")
        # print(f"DEBUG IK: Flange Target={target_pos_flange}")
        
        full_guess = self._active_to_full(initial_guess)
        
        # 3. Solver IKPy
        full_sol = self.chain.inverse_kinematics(
            target_position=target_pos_flange,
            target_orientation=flange_rot_matrix, 
            orientation_mode='all', 
            initial_position=full_guess
        )
        
        return self._full_to_active(full_sol)

    def forward_kinematics_full(self, active_angles):
        full_joints = self._active_to_full(active_angles)
        return self.chain.forward_kinematics(full_joints, full_kinematics=True)

    # ================= HELPERS =================

    def _active_to_full(self, active_joints):
        arr = np.array(active_joints, dtype=float).flatten()
        if len(arr) == 7: arr = arr[1:] 
        if len(arr) != 6: arr = np.resize(arr, 6)
        full = np.zeros(len(self.chain.links))
        curr = 0
        for i, act in enumerate(self.active_links_mask):
            if act and curr < 6:
                full[i] = arr[curr]
                curr += 1
        return full

    def _full_to_active(self, full_vector):
        if self.active_links_mask: return np.compress(self.active_links_mask, full_vector)
        return np.zeros(6)

    def generate_linear_tcp_trajectory(self, start_tf, end_tf, n_points):
        start_pos = start_tf[:3, 3]; end_pos = end_tf[:3, 3]
        t = np.linspace(0, 1, int(n_points))
        smooth_t = (1 - np.cos(t * np.pi)) / 2 
        traj_pos = start_pos + (end_pos - start_pos) * smooth_t[:, np.newaxis]
        
        rot_start = R.from_matrix(start_tf[:3, :3])
        rot_end = R.from_matrix(end_tf[:3, :3])
        slerp = Slerp([0, 1], R.from_matrix([start_tf[:3, :3], end_tf[:3, :3]]))
        traj_rot = slerp(smooth_t).as_matrix()
        
        return traj_pos, traj_rot

    def generate_linear_tcp_trajectory_points(self, start_tf, end_tf, n_points):
        return self.generate_linear_tcp_trajectory(start_tf, end_tf, n_points)

    def joint_trajectory_from_tcp(self, traj_xyz, traj_rot_3x3, initial_guess=None):
        n_points = len(traj_xyz)
        joint_traj = np.zeros((n_points, 6))
        prev = np.zeros(6)
        if initial_guess is not None:
            tmp = np.array(initial_guess)
            if len(tmp)==7: tmp=tmp[1:]
            if len(tmp)==6: prev=tmp
            
        for i in range(n_points):
            try:
                sol = self.inverse_kinematics(traj_xyz[i], traj_rot_3x3[i], prev)
                joint_traj[i] = sol
                prev = sol
            except Exception:
                joint_traj[i] = prev
        return joint_traj

    # --- CONFIG LOADERS ---
    def get_visual_origins(self): return self.visual_origins
    def get_joint_limits(self): return self.joint_limits_rad
    
    def _load_active_joint_limits(self):
        # Limity Parol6 w stopniach
        deg = [
            (-90, 90),  # J1 (Baza)
            (-50, 140),   # J2 (Bark)
            (-100, 70),  # J3 (Łokieć)
            (-100, 180),  # J4 (Obrót ramienia)
            (-120, 110),  # J5 (Pochylenie nadgarstka)
            (-110, 180)   # J6 (Obrót narzędzia - może być większy)
        ]
        return [(np.deg2rad(mn), np.deg2rad(mx)) for mn, mx in deg]

    def _load_visual_origins(self, urdf_path):
        origins = {}
        try:
            tree = ET.parse(urdf_path); root = tree.getroot()
            for link in root.findall('link'):
                vis = link.find('visual')
                if vis:
                    o = vis.find('origin')
                    if o is not None:
                        xyz = [float(x) for x in o.attrib.get('xyz','0 0 0').split()]
                        rpy = [float(r) for r in o.attrib.get('rpy','0 0 0').split()]
                        origins[link.attrib.get('name')] = (xyz, rpy)
        except: pass
        return origins

    def _setup_mock_chain(self):
        self.chain = type('Mock', (object,), {'links': [], 'active_links_mask': [], 'forward_kinematics': lambda *a, **k: np.eye(4), 'inverse_kinematics': lambda *a, **k: np.zeros(8)})()