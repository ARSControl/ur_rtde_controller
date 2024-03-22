import ctypes, os, shutil
import numpy as np
from spatialmath import SE3

# Get Lib Path
from pathlib import Path
PACKAGE_PATH = f'{str(Path(__file__).resolve().parents[1])}'
LIB_PATH = f'{PACKAGE_PATH}/lib'

def delete_pycache_folders(project_dir:str):

    """ Delete Python `__pycache__` Folders Function """

    # Walk Through the Project Folders
    for root, dirs, files in os.walk(project_dir):

        if "__pycache__" in dirs:

            # Get `__pycache__` Path
            pycache_folder = os.path.join(root, "__pycache__")
            print(f"Deleting {pycache_folder}")

            # Delete `__pycache__`
            try: shutil.rmtree(pycache_folder)
            except Exception as e: print(f"An error occurred while deleting {pycache_folder}: {e}")

class Kinematic_Wrapper:

    def __init__(self, robot_name:str='ur10e'):

        if robot_name.lower() == 'ur3':

            # Load UR3 Kinematic Shared Libraries into Ctypes
            self.jacobian         = ctypes.CDLL(f'{LIB_PATH}/ur3_kinematic/compute_UR3_jacobian.so').compute_UR3_jacobian
            self.jacobian_dot     = ctypes.CDLL(f'{LIB_PATH}/ur3_kinematic/compute_UR3_jacobian_dot.so').compute_UR3_jacobian_dot
            self.jacobian_dot_dq  = ctypes.CDLL(f'{LIB_PATH}/ur3_kinematic/compute_UR3_jacobian_dot_dq.so').compute_UR3_jacobian_dot_dq
            self.direct_kinematic = ctypes.CDLL(f'{LIB_PATH}/ur3_kinematic/compute_UR3_direct_kinematic.so').compute_UR3_direct_kinematic

        elif robot_name.lower() == 'ur5':

            # Load UR5 Kinematic Shared Libraries into Ctypes
            self.jacobian         = ctypes.CDLL(f'{LIB_PATH}/ur5_kinematic/compute_UR5_jacobian.so').compute_UR5_jacobian
            self.jacobian_dot     = ctypes.CDLL(f'{LIB_PATH}/ur5_kinematic/compute_UR5_jacobian_dot.so').compute_UR5_jacobian_dot
            self.jacobian_dot_dq  = ctypes.CDLL(f'{LIB_PATH}/ur5_kinematic/compute_UR5_jacobian_dot_dq.so').compute_UR5_jacobian_dot_dq
            self.direct_kinematic = ctypes.CDLL(f'{LIB_PATH}/ur5_kinematic/compute_UR5_direct_kinematic.so').compute_UR5_direct_kinematic

        elif robot_name.lower() == 'ur10':

            # Load UR10 Kinematic Shared Libraries into Ctypes
            self.jacobian         = ctypes.CDLL(f'{LIB_PATH}/ur10_kinematic/compute_UR10_jacobian.so').compute_UR10_jacobian
            self.jacobian_dot     = ctypes.CDLL(f'{LIB_PATH}/ur10_kinematic/compute_UR10_jacobian_dot.so').compute_UR10_jacobian_dot
            self.jacobian_dot_dq  = ctypes.CDLL(f'{LIB_PATH}/ur10_kinematic/compute_UR10_jacobian_dot_dq.so').compute_UR10_jacobian_dot_dq
            self.direct_kinematic = ctypes.CDLL(f'{LIB_PATH}/ur10_kinematic/compute_UR10_direct_kinematic.so').compute_UR10_direct_kinematic

        elif robot_name.lower() == 'ur3e':

            # Load UR3e Kinematic Shared Libraries into Ctypes
            self.jacobian         = ctypes.CDLL(f'{LIB_PATH}/ur3e_kinematic/compute_UR3e_jacobian.so').compute_UR3e_jacobian
            self.jacobian_dot     = ctypes.CDLL(f'{LIB_PATH}/ur3e_kinematic/compute_UR3e_jacobian_dot.so').compute_UR3e_jacobian_dot
            self.jacobian_dot_dq  = ctypes.CDLL(f'{LIB_PATH}/ur3e_kinematic/compute_UR3e_jacobian_dot_dq.so').compute_UR3e_jacobian_dot_dq
            self.direct_kinematic = ctypes.CDLL(f'{LIB_PATH}/ur3e_kinematic/compute_UR3e_direct_kinematic.so').compute_UR3e_direct_kinematic

        elif robot_name.lower() == 'ur5e':

            # Load UR5e Kinematic Shared Libraries into Ctypes
            self.jacobian         = ctypes.CDLL(f'{LIB_PATH}/ur5e_kinematic/compute_UR5e_jacobian.so').compute_UR5e_jacobian
            self.jacobian_dot     = ctypes.CDLL(f'{LIB_PATH}/ur5e_kinematic/compute_UR5e_jacobian_dot.so').compute_UR5e_jacobian_dot
            self.jacobian_dot_dq  = ctypes.CDLL(f'{LIB_PATH}/ur5e_kinematic/compute_UR5e_jacobian_dot_dq.so').compute_UR5e_jacobian_dot_dq
            self.direct_kinematic = ctypes.CDLL(f'{LIB_PATH}/ur5e_kinematic/compute_UR5e_direct_kinematic.so').compute_UR5e_direct_kinematic

        elif robot_name.lower() == 'ur10e':

            # Load UR5e Kinematic Shared Libraries into Ctypes
            self.jacobian         = ctypes.CDLL(f'{LIB_PATH}/ur10e_kinematic/compute_UR10e_jacobian.so').compute_UR10e_jacobian
            self.jacobian_dot     = ctypes.CDLL(f'{LIB_PATH}/ur10e_kinematic/compute_UR10e_jacobian_dot.so').compute_UR10e_jacobian_dot
            self.jacobian_dot_dq  = ctypes.CDLL(f'{LIB_PATH}/ur10e_kinematic/compute_UR10e_jacobian_dot_dq.so').compute_UR10e_jacobian_dot_dq
            self.direct_kinematic = ctypes.CDLL(f'{LIB_PATH}/ur10e_kinematic/compute_UR10e_direct_kinematic.so').compute_UR10e_direct_kinematic

        elif robot_name.lower() == 'ur16e':

            # Load UR16e Kinematic Shared Libraries into Ctypes
            self.jacobian         = ctypes.CDLL(f'{LIB_PATH}/ur16e_kinematic/compute_UR16e_jacobian.so').compute_UR16e_jacobian
            self.jacobian_dot     = ctypes.CDLL(f'{LIB_PATH}/ur16e_kinematic/compute_UR16e_jacobian_dot.so').compute_UR16e_jacobian_dot
            self.jacobian_dot_dq  = ctypes.CDLL(f'{LIB_PATH}/ur16e_kinematic/compute_UR16e_jacobian_dot_dq.so').compute_UR16e_jacobian_dot_dq
            self.direct_kinematic = ctypes.CDLL(f'{LIB_PATH}/ur16e_kinematic/compute_UR16e_direct_kinematic.so').compute_UR16e_direct_kinematic

        else: raise ValueError(f'Robot Name Must be either "ur3", "ur5", "ur10", "ur3e", "ur5e", "ur10e" or "ur16e", given: {robot_name}')

        # Define the Argument and Return types for the function
        self.jacobian.argtypes = [np.ctypeslib.ndpointer(float), np.ctypeslib.ndpointer(float)]
        self.jacobian_dot.argtypes = [np.ctypeslib.ndpointer(float), np.ctypeslib.ndpointer(float), np.ctypeslib.ndpointer(float)]
        self.jacobian_dot_dq.argtypes = [np.ctypeslib.ndpointer(float), np.ctypeslib.ndpointer(float), np.ctypeslib.ndpointer(float)]
        self.direct_kinematic.argtypes = [np.ctypeslib.ndpointer(float), np.ctypeslib.ndpointer(float)]

    def compute_jacobian(self, q:np.ndarray) -> np.ndarray:

        """ Compute Jacobian """

        assert isinstance(q, np.ndarray), f'Joint States Must be a Numpy Array, given: {type(q)}'
        assert len(q) == 6, f'Joint States Must be a 6-Dimensional Vector, given: {len(q)}'

        result = np.zeros((36,))
        self.jacobian(result, q.astype(np.double))

        return result.reshape(6,6)

    def compute_jacobian_dot(self, q:np.ndarray, dq:np.ndarray) -> np.ndarray:

        """ Compute Jacobian Derivative """

        assert isinstance(q, np.ndarray), f'Joint States Must be a Numpy Array, given: {type(q)}'
        assert isinstance(dq, np.ndarray), f'Joint States Must be a Numpy Array, given: {type(dq)}'
        assert len(q) == 6, f'Joint States Must be a 6-Dimensional Vector, given: {len(q)}'
        assert len(dq) == 6, f'Joint Vel Must be a 6-Dimensional Vector, given: {len(dq)}'

        result = np.zeros((36,))
        self.jacobian_dot(result, q.astype(np.double), dq.astype(np.double))
        return result.reshape(6,6)

    def compute_jacobian_dot_dq(self, q:np.ndarray, dq:np.ndarray) -> np.ndarray:

        """ Compute Jacobian Derivative @ dq """

        assert isinstance(q, np.ndarray), f'Joint States Must be a Numpy Array, given: {type(q)}'
        assert isinstance(dq, np.ndarray), f'Joint States Must be a Numpy Array, given: {type(dq)}'
        assert len(q) == 6, f'Joint States Must be a 6-Dimensional Vector, given: {len(q)}'
        assert len(dq) == 6, f'Joint Vel Must be a 6-Dimensional Vector, given: {len(dq)}'

        result = np.zeros((6,))
        self.jacobian_dot_dq(result, q.astype(np.double), dq.astype(np.double))
        return result.reshape(6,)

    def compute_forward_kinematic(self, q:np.ndarray) -> np.ndarray:

        """ Compute Forward Kinematic """

        assert isinstance(q, np.ndarray), f'Joint States Must be a Numpy Array, given: {type(q)}'
        assert len(q) == 6, f'Joint States Must be a 6-Dimensional Vector, given: {len(q)}'

        result = np.zeros((16,))
        self.direct_kinematic(result, q.astype(np.double))
        return SE3(result.reshape(4,4).tolist())
