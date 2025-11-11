import numpy as np
from ikpy.chain import Chain
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R

class RobotKinematics:
    """
    Obsługuje wszystkie obliczenia kinematyczne (FK, IK)
    na podstawie pliku URDF.
    Dodano funkcję continuous IK oraz generowanie trajektorii liniowej
    z obliczaniem prędkości i przyspieszeń przegubowych.
    """
    def __init__(self, urdf_path, active_links_mask):
        try:
            self.chain = Chain.from_urdf_file(
                urdf_path,
                active_links_mask=active_links_mask
            )
            self.chain.max_iterations = 200
            self.chain.convergence_limit = 1e-6
            print("URDF wczytany pomyślnie.")

            self.joint_limits_rad = self._load_joint_limits()
            self.visual_origins = self._load_visual_origins(urdf_path)

        except Exception as e:
            print(f"Krytyczny błąd wczytywania URDF: {e}. Używam atrapy.")
            # Stworzymy atrapy
            class MockLink:
                name = "mock"
                has_rotation = True
                bounds = (-np.pi, np.pi)
            class MockChain:
                links = [MockLink() for i in range(7)]
                def forward_kinematics(self, angles, full_kinematics=False):
                    identity = np.eye(4)
                    if full_kinematics: return [identity] * len(self.links)
                    return identity
                def inverse_kinematics(self, *args, **kwargs):
                    return np.zeros(7)
            self.chain = MockChain()
            self.joint_limits_rad = [(-1.7, 1.7), (-0.98, 1.0), (-2.0, 1.3), (-2.0, 2.0), (-2.1, 2.1), (-3.1, 3.1)]
            self.visual_origins = {}

    def _load_joint_limits(self):
        """Pobiera limity stawów z łańcucha IKPy."""
        limits = []
        for link in self.chain.links[1:]:  # Pomiń bazę
            if getattr(link, 'has_rotation', True):
                bounds = getattr(link, 'bounds', (-np.pi, np.pi))
                limits.append(bounds)
            else:
                limits.append((-np.pi, np.pi))
        print(f"Wczytano limity stawów (rad): {limits}")
        return limits

    def _load_visual_origins(self, urdf_path):
        """Parsuje URDF w poszukiwaniu tagów 'visual' > 'origin'."""
        origins = {}
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            for link in root.findall('link'):
                name = link.attrib['name']
                visual = link.find('visual')
                if visual is not None:
                    origin = visual.find('origin')
                    if origin is not None:
                        xyz = [float(x) for x in origin.attrib.get('xyz', '0 0 0').split()]
                        rpy = [float(r) for r in origin.attrib.get('rpy', '0 0 0').split()]
                        origins[name] = (xyz, rpy)
        except Exception as e:
            print(f"Błąd parsowania XML URDF dla 'visual': {e}")
        return origins

    def get_joint_limits(self):
        return self.joint_limits_rad

    def get_visual_origins(self):
        return self.visual_origins

    def forward_kinematics(self, angles):
        """Zwraca macierz 4x4 tylko dla efektora końcowego."""
        return self.chain.forward_kinematics(angles)

    def forward_kinematics_full(self, angles):
        """Zwraca listę macierzy 4x4 dla wszystkich linków (do rysowania)."""
        return self.chain.forward_kinematics(angles, full_kinematics=True)

    def inverse_kinematics(self, target_pos, target_orient_3x3, initial_pos):
        """Oblicza kinematykę odwrotną za pomocą IKPy."""
        return self.chain.inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_orient_3x3,
            orientation_mode='all',
            initial_position=initial_pos
        )

    def continuous_ik(self, target_pos, target_orient_3x3, prev_angles):
        """
        Oblicza IK zachowując ciągłość ruchu stawów (minimalizuje różnicę względem prev_angles).
        """
        # Rozwiązanie główne
        solA = self.inverse_kinematics(target_pos, target_orient_3x3, prev_angles)

        # Alternatywna gałąź (odwrócone niektóre stawy dla nadgarstka)
        alt_init = prev_angles.copy()
        alt_init[0] = -alt_init[0]  # J1
        alt_init[3] = -alt_init[3]  # J4
        alt_init[5] = -alt_init[5]  # J6
        solB = self.inverse_kinematics(target_pos, target_orient_3x3, alt_init)

        # Wybór rozwiązania minimalizującego różnicę względem poprzednich kątów
        diffA = np.sum(np.abs(self._angle_difference_vector(solA, prev_angles)))
        diffB = np.sum(np.abs(self._angle_difference_vector(solB, prev_angles)))
        chosen = solA if diffA < diffB else solB

        # Clip do limitów stawów
        for i, (low, high) in enumerate(self.joint_limits_rad):
            chosen[i] = np.clip(chosen[i], low, high)

        return chosen

    # ----------------------------- NOWE FUNKCJE: TRAJEKTORIA I D/D2 -----------------------------
    def generate_linear_tcp_trajectory(self, start_xyz, end_xyz, n_points):
        """
        Generuje listę punktów TCP (x,y,z) liniowo mędzy start a end (równomiernie w przestrzeni).
        Zwraca tablicę shape (n_points, 3).
        """
        A = np.array(start_xyz, dtype=float)
        B = np.array(end_xyz, dtype=float)
        traj = np.array([A + (B - A) * i / (n_points - 1) for i in range(n_points)])
        return traj

    def joint_trajectory_from_tcp(self, traj_xyz, traj_orient_3x3=None, initial_guess=None):
        """
        Dla każdego punktu TCP oblicza IK i zwraca tablicę kątów przegubów.
        traj_xyz: (N,3)
        traj_orient_3x3: albo None (użyj obecnej orientacji) albo lista/array macierzy 3x3 dla każdego punktu
        initial_guess: wektor początkowy q0
        Zwraca tablicę shape (N, n_joints)
        """
        n_points = len(traj_xyz)
        n_joints = len(self.joint_limits_rad)
        joint_traj = np.zeros((n_points, n_joints))

        prev = initial_guess if initial_guess is not None else np.zeros(n_joints)
        for i in range(n_points):
            pos = traj_xyz[i]
            orient = traj_orient_3x3[i] if (traj_orient_3x3 is not None) else None
            sol = self.continuous_ik(pos, orient, prev)
            joint_traj[i] = sol[:n_joints]
            prev = joint_traj[i]

        return joint_traj

    def compute_joint_vel_acc(self, joint_traj, dt):
        """
        Oblicza prędkości i przyspieszeń kątowych z trajektorii kątów.

        - joint_traj: array (N, n_joints) w radianach
        - dt: przyrost czasu między kolejnymi punktami (s)

        Zwraca: (velocities, accelerations)
          velocities: (N, n_joints) rad/s
          accelerations: (N, n_joints) rad/s^2

        Uwaga: używamy unwrap (najkrótsza zmiana katowa) przed różniczkowaniem,
        aby uniknąć skoków o 2*pi.
        """
        # Unwrap po osi czasowej, period = 2*pi
        unwrapped = np.unwrap(joint_traj, axis=0)

        # prędkości: pierwsza pochodna (central differences via np.gradient)
        vel = np.gradient(unwrapped, dt, axis=0)

        # przyspieszenia: druga pochodna
        acc = np.gradient(vel, dt, axis=0)

        return vel, acc

    # ----------------------------- PRAKTYCZNE FUNKCJE POMOCNICZE -----------------------------
    def _angle_difference_vector(self, a, b):
        """Zwraca wektor najkrótszych różnic kątowych (a-b) w przedziale [-pi, pi]."""
        diff = (a - b + np.pi) % (2 * np.pi) - np.pi
        return diff


if __name__ == '__main__':
    # Przykład użycia
    urdf = 'robot.urdf'
    active_mask = None  # dopasuj do swojego URDF / IKPy

    rk = RobotKinematics(urdf, active_mask)

    start = [0.3, 0.2, 0.4]
    end = [0.5, 0.2, 0.4]
    n = 101
    dt = 0.01  # 10 ms

    tcp_traj = rk.generate_linear_tcp_trajectory(start, end, n)
    joint_traj = rk.joint_trajectory_from_tcp(tcp_traj)

    vel, acc = rk.compute_joint_vel_acc(joint_traj, dt)

    print('Trajektoria TCP -> liczba punktów:', tcp_traj.shape)
    print('Trajektoria kątowa -> shape:', joint_traj.shape)
    print('Przykładowe prędkości (rad/s) pierwszego kroku:', vel[0])
    print('Przykładowe przyspieszenia (rad/s^2) pierwszego kroku:', acc[0])

    # Możesz dalej przeliczyć vel/acc na kroki (microsteps/s) i ustawiać VMAX/AMAX w TMC5160.
