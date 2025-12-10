import numpy as np
from ikpy.chain import Chain
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R

class RobotKinematics:
    def __init__(self, urdf_path, active_links_mask=None):
        self.chain = None
        self.urdf_path = urdf_path
        
        try:
            # 1. Wczytujemy łańcuch
            self.chain = Chain.from_urdf_file(
                urdf_path,
                active_links_mask=active_links_mask
            )
            
            # 2. Parametry solvera
            self.chain.max_iterations = 100
            self.chain.convergence_limit = 1e-4
            
            # -------------------------------------------------------------------------
            # AUTOKOREKTA MASKI (NAPRAWA BŁĘDU (6,) vs (7,))
            # -------------------------------------------------------------------------
            # Sprawdzamy, ile linków IKPy uważa za "aktywne"
            detected_active = [x for x in self.chain.active_links_mask if x]
            
            # Jeśli mamy 7 lub więcej aktywnych stawów, a to robot 6-osiowy,
            # to zazwyczaj ostatni link (narzędzie) został błędnie uznany za silnik.
            if len(detected_active) > 6:
                print(f"UWAGA: Wykryto {len(detected_active)} aktywnych stawów. Wymuszam tryb 6-osiowy.")
                
                # Tworzymy nową maskę:
                # Zazwyczaj struktura to: [Base(F), J1(T), J2(T), J3(T), J4(T), J5(T), J6(T), Tool(F/T?)]
                # Znajdujemy indeksy True i wyłączamy wszystko powyżej 6-tego silnika.
                
                new_mask = list(self.chain.active_links_mask)
                active_count = 0
                for i in range(len(new_mask)):
                    if new_mask[i]:
                        active_count += 1
                        if active_count > 6:
                            new_mask[i] = False # Wyłącz nadmiarowe osie (np. gripper)
                
                self.chain.active_links_mask = new_mask
            
            # -------------------------------------------------------------------------

            print(f"URDF wczytany. Linki ogółem: {len(self.chain.links)}")
            
            # Aktualizacja indeksów po ewentualnej korekcie
            self.active_joints_indices = [i for i, x in enumerate(self.chain.active_links_mask) if x]
            self.n_active_joints = len(self.active_joints_indices)
            
            print(f"Liczba aktywnych napędów (po korekcie): {self.n_active_joints}")
            print(f"Maska aktywności: {self.chain.active_links_mask}")

            if self.n_active_joints != 6:
                print("OSTRZEŻENIE: Liczba osi jest inna niż 6! Może to powodować błędy w GUI.")

            self.joint_limits_rad = self._load_active_joint_limits()
            self.visual_origins = self._load_visual_origins(urdf_path)

        except Exception as e:
            print(f"Krytyczny błąd wczytywania URDF: {e}. Przełączam na tryb ATRAPY (Mock).")
            import traceback
            traceback.print_exc()
            self._setup_mock_chain()

    def _setup_mock_chain(self):
        """Konfiguracja atrapy."""
        class MockLink:
            name = "mock"
            bounds = (-np.pi, np.pi)
        
        class MockChain:
            def __init__(self):
                self.links = [MockLink() for _ in range(8)]
                # Standardowa maska dla 6 osi: Base=False, 1-6=True, Tool=False
                self.active_links_mask = [False, True, True, True, True, True, True, False]

            def forward_kinematics(self, full_joints, full_kinematics=False):
                identity = np.eye(4)
                if full_kinematics: return [identity] * len(self.links)
                return identity

            def inverse_kinematics(self, target_position, target_orientation=None, orientation_mode=None, initial_position=None):
                return np.zeros(len(self.links))

        self.chain = MockChain()
        self.active_links_mask = self.chain.active_links_mask
        self.active_joints_indices = [1, 2, 3, 4, 5, 6]
        self.n_active_joints = 6
        self.joint_limits_rad = [(-3.14, 3.14)] * 6
        self.visual_origins = {}

    def _load_active_joint_limits(self):
        limits = []
        for i, link in enumerate(self.chain.links):
            if self.chain.active_links_mask[i]:
                bounds = getattr(link, 'bounds', (-np.pi, np.pi))
                if bounds is None: bounds = (-np.pi, np.pi)
                limits.append(bounds)
        return limits

    def _load_visual_origins(self, urdf_path):
        origins = {}
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            for link in root.findall('link'):
                name = link.attrib.get('name')
                visual = link.find('visual')
                if visual is not None:
                    origin = visual.find('origin')
                    if origin is not None:
                        xyz = [float(x) for x in origin.attrib.get('xyz', '0 0 0').split()]
                        rpy = [float(r) for r in origin.attrib.get('rpy', '0 0 0').split()]
                        origins[name] = (xyz, rpy)
        except Exception:
            pass
        return origins

    # ================= GETTERY =================
    
    def get_visual_origins(self):
        return self.visual_origins

    def get_joint_limits(self):
        return self.joint_limits_rad

    # ================= KONWERSJE (Z ZABEZPIECZENIAMI) =================
    
    def _active_to_full(self, active_joints):
        # Jeśli wejście ma 6 elementów, a maska oczekuje np. 7, musimy to obsłużyć.
        # Ale tutaj mamy już n_active_joints skorygowane w __init__.
        
        # Konwersja na array, żeby uniknąć błędów listy
        active_arr = np.array(active_joints, dtype=float)

        if len(active_arr) != self.n_active_joints:
            # Próba ratunku, jeśli worker wysłał 6, a my jednak mamy 7 (lub odwrotnie)
            if len(active_arr) > self.n_active_joints:
                active_arr = active_arr[:self.n_active_joints]
            elif len(active_arr) < self.n_active_joints:
                # Dopełnij zerami
                padding = np.zeros(self.n_active_joints - len(active_arr))
                active_arr = np.concatenate((active_arr, padding))

        full_vector = np.zeros(len(self.chain.links))
        np.place(full_vector, self.chain.active_links_mask, active_arr)
        return full_vector

    def _full_to_active(self, full_vector):
        """Wybiera z pełnego wektora tylko aktywne stawy."""
        active = np.compress(self.chain.active_links_mask, full_vector)
        
        # Zabezpieczenie: jeśli z jakiegoś powodu active ma 7 elementów, a my chcemy 6
        if len(active) > self.n_active_joints:
            active = active[:self.n_active_joints]
            
        return active

    # ================= KINEMATYKA =================

    def forward_kinematics(self, active_angles):
        full_joints = self._active_to_full(active_angles)
        return self.chain.forward_kinematics(full_joints)

    def forward_kinematics_full(self, active_angles):
        full_joints = self._active_to_full(active_angles)
        return self.chain.forward_kinematics(full_joints, full_kinematics=True)

    def inverse_kinematics(self, target_pos, target_orient_3x3, initial_active_joints=None):
        if initial_active_joints is not None:
            initial_full = self._active_to_full(initial_active_joints)
        else:
            initial_full = np.zeros(len(self.chain.links))

        full_solution = self.chain.inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_orient_3x3,
            orientation_mode='all', 
            initial_position=initial_full
        )

        return self._full_to_active(full_solution)

    def continuous_ik(self, target_pos, target_orient_3x3, prev_active_angles):
        # Upewniamy się, że prev_active_angles ma dobry wymiar
        prev_active_angles = np.array(prev_active_angles)
        if len(prev_active_angles) != self.n_active_joints:
             # Szybka naprawa wymiaru wejściowego
             if len(prev_active_angles) > self.n_active_joints:
                 prev_active_angles = prev_active_angles[:self.n_active_joints]
             else:
                 pad = np.zeros(self.n_active_joints - len(prev_active_angles))
                 prev_active_angles = np.concatenate((prev_active_angles, pad))

        sol_active_A = self.inverse_kinematics(target_pos, target_orient_3x3, prev_active_angles)

        # Rozwiązanie alternatywne
        alt_init = prev_active_angles.copy()
        if len(alt_init) >= 6:
            alt_init[0] = -alt_init[0] 
            alt_init[3] = -alt_init[3] 
            alt_init[5] = -alt_init[5] 
            
        sol_active_B = self.inverse_kinematics(target_pos, target_orient_3x3, alt_init)

        # Teraz bezpiecznie liczymy różnicę, bo sol i prev mają ten sam wymiar dzięki _full_to_active i ifom
        diffA = np.sum(np.abs(self._angle_difference_vector(sol_active_A, prev_active_angles)))
        diffB = np.sum(np.abs(self._angle_difference_vector(sol_active_B, prev_active_angles)))

        return sol_active_A if diffA < diffB else sol_active_B

    # ================= TRAJEKTORIE =================

    def generate_linear_tcp_trajectory(self, start_xyz, end_xyz, n_points):
        A = np.array(start_xyz, dtype=float)
        B = np.array(end_xyz, dtype=float)
        return np.linspace(A, B, n_points)

    def joint_trajectory_from_tcp(self, traj_xyz, traj_orient_3x3=True, initial_guess=None):
        n_points = len(traj_xyz)
        joint_traj = np.zeros((n_points, self.n_active_joints))

        if initial_guess is not None:
            prev = np.array(initial_guess)
            # Zabezpieczenie wymiaru initial_guess
            if len(prev) > self.n_active_joints:
                prev = prev[:self.n_active_joints]
        else:
            prev = np.zeros(self.n_active_joints)

        for i in range(n_points):
            pos = traj_xyz[i]
            orient = traj_orient_3x3[i] if (traj_orient_3x3 is not None) else None
            
            try:
                sol = self.continuous_ik(pos, orient, prev)
                joint_traj[i] = sol
                prev = sol 
            except Exception as e:
                print(f"Błąd IK w punkcie {i}: {e}. Używam poprzednich kątów.")
                joint_traj[i] = prev

        return joint_traj

    def compute_joint_vel_acc(self, joint_traj, dt):
        unwrapped = np.unwrap(joint_traj, axis=0)
        vel = np.gradient(unwrapped, dt, axis=0)
        acc = np.gradient(vel, dt, axis=0)
        return vel, acc

    def _angle_difference_vector(self, a, b):
        # Tu następował crash. Teraz wymiary a i b są pilnowane.
        diff = (a - b + np.pi) % (2 * np.pi) - np.pi
        return diff