from math import pi
from termcolor import colored
from typing import Union, List, Tuple, Callable, Optional

# Import Peter Corke Robotics Toolbox
import roboticstoolbox as rtb, numpy as np
from roboticstoolbox.robot import DHRobot, RevoluteDH
from roboticstoolbox.robot.IK import IKSolution
from roboticstoolbox.tools.types import ArrayLike, NDArray
from roboticstoolbox.tools.trajectory import Trajectory, jtraj, ctraj

# Import Spatial Math
from spatialmath import SE3, SO3
from scipy.spatial.transform import Rotation
from scipy.interpolate import CubicSpline

# Import ROS2 Libraries
from geometry_msgs.msg import Pose

class UR_Toolbox():

    def __init__(self, robot_parameters:dict, complete_debug:bool=False, debug:bool=False):

        """ Initialize Robot Toolbox """

        # Set Debug Flags
        self.complete_debug, self.debug = complete_debug, debug or complete_debug

        # Create Robot - Robotic Toolbox
        self.robot = self.create_robot(robot_parameters)

        # Load Kinematic Functions
        self.fkine, self.J, self.J_dot = self.load_kinematic_functions(robot_parameters['robot'])

    def create_robot(self, robot_parameters:dict, symbolic:bool=False) -> DHRobot:

        """ Create Robot Model """

        # Robot Parameters
        robot_model, tool = robot_parameters['robot'], robot_parameters['tool'],
        self.payload, self.reach, self.tcp_speed = robot_parameters['payload'], robot_parameters['reach'], robot_parameters['tcp_speed']
        self.stopping_time, self.stopping_distance, self.position_repeatability = robot_parameters['stopping_time'], robot_parameters['stopping_distance'], robot_parameters['position_repeatability']
        self.maximum_power, self.operating_power, self.operating_temperature = robot_parameters['maximum_power'], robot_parameters['operating_power'], robot_parameters['operating_temperature']
        self.ft_range, self.ft_precision, self.ft_accuracy = robot_parameters['ft_range'], robot_parameters['ft_precision'], robot_parameters['ft_accuracy']
        a, d, alpha, offset = robot_parameters['a'], robot_parameters['d'], robot_parameters['alpha'], robot_parameters['theta']
        mass, center_of_mass = robot_parameters['mass'], [robot_parameters['center_of_mass'][i:i+3] for i in range(0, len(robot_parameters['center_of_mass']), 3)]
        self.q_lim, self.qd_lim, self.qdd_lim = robot_parameters['q_limits'], robot_parameters['q_dot_limits'], robot_parameters['q_ddot_limits']

        # Compute Tool Transformation Matrix
        tool_transform = np.eye(4)
        tool_transform[:3, 3] = tool[:3]
        tool_transform[:3, :3] = Rotation.from_euler('xyz', tool[3:], degrees=False).as_matrix()

        # Debug Print
        if self.complete_debug:

            # Robot Parameters
            print(colored(f'Robot Parameters:\n', 'yellow'))
            for param in robot_parameters.keys(): print(colored(f'    {param}: ', 'green'), f'{robot_parameters[param]}')

            # Tool Transformation Matrix
            print(colored(f'\nTool Transformation Matrix:\n\n{SE3(tool_transform)}\n', 'green'))

        # Create Robot Model
        return DHRobot(
            [RevoluteDH(a=a_n, d=d_n, alpha=alpha_n, offset=offset_n, q_lim=[-q_lim_n, q_lim_n], m=mass_n, r=center_of_mass_n.tolist()) 
             for a_n, d_n, alpha_n, offset_n, mass_n, center_of_mass_n, q_lim_n in zip(a, d, alpha, offset, mass, center_of_mass, self.q_lim)],
            name=robot_model, manufacturer='Universal Robot', symbolic=symbolic, tool=SE3(tool_transform))

    def load_kinematic_functions(self, robot_model:str) -> Tuple[Callable[[List[float]], np.ndarray], Callable[[List[float]], np.ndarray], Callable[[List[float], List[float]], np.ndarray]]:

        """ Load Kinematic Functions """

        # Import Kinematic Functions Wrapper
        from kinematic_wrapper import Kinematic_Wrapper

        try:

            # Load Functions
            kin = Kinematic_Wrapper(robot_model)
            return kin.compute_forward_kinematic, kin.compute_jacobian, kin.compute_jacobian_dot

        except ValueError:

            # Return Robotics Toolbox Functions
            print(colored('Kinematic Functions Not Found, Using Robotics Toolbox Functions', 'red'))
            return self.use_toolbox_functions()

    def use_toolbox_functions(self) -> Tuple[Callable[[List[float]], np.ndarray], Callable[[List[float]], np.ndarray], Callable[[List[float]], np.ndarray]]:

        """ Use Robotics Toolbox FK, J, J_dot Functions """

        # Return Toolbox Functions
        return self.robot.fkine, self.robot.jacob0, self.robot.jacob0_dot

    def ForwardKinematic(self, joint_positions:Union[List[float], np.ndarray], end:str=None, start:str=None) -> SE3:

        """ Forward Kinematics Using Peter Corke Robotics Toolbox """

        # Type Assertion
        assert type(joint_positions) in [List[float], np.ndarray], f"Joint Positions must be a ArrayLike | {type(joint_positions)} given"
        assert len(joint_positions) == 6, f"Joint Positions Length must be 6 | {len(joint_positions)} given \nJoint Positions:\n{joint_positions}"

        # Return Forward Kinematic
        return self.fkine(joint_positions)

    def InverseKinematic(self, pose:Union[Pose, NDArray, SE3], actual_pose:List[float]=[0, -pi/2, pi/2, -pi/2, -pi/2, 0]) -> Optional[np.ndarray]:

        """ Inverse Kinematics Using Peter Corke Robotics Toolbox """

        # Type Assertion
        assert type(pose) in [Pose, np.ndarray, SE3], f"Pose must be a Pose, NumPy Array or SE3 | {type(pose)} given"

        # Convert Pose to SE3 Matrix
        if type(pose) is Pose: pose = self.pose2matrix(pose)
        if type(pose) is np.ndarray: pose = self.array2matrix(pose)

        # Compute Inverse Kinematics
        ik:IKSolution = self.robot.ikine_NR(pose, q0=actual_pose)

        # Return Inverse Kinematics
        return ik.q if ik.success else None

    def Jacobian(self, joint_positions:ArrayLike) -> np.ndarray:

        """ Get Jacobian Matrix """

        # Type Assertion
        assert type(joint_positions) in [List[float], np.ndarray], f"Joint Positions must be a ArrayLike | {type(joint_positions)} given"
        assert len(joint_positions) == 6, f"Joint Positions Length must be 6 | {len(joint_positions)} given \nJoint Positions:\n{joint_positions}"

        return self.J(joint_positions)

    def JacobianDot(self, joint_positions:ArrayLike, joint_velocities:ArrayLike) -> np.ndarray:

        """ Get Jacobian Derivative Matrix """

        # Type Assertion
        assert type(joint_positions) in [List[float], np.ndarray], f"Joint Positions must be a ArrayLike | {type(joint_positions)} given"
        assert type(joint_velocities) in [List[float], np.ndarray], f"Joint Velocities must be a ArrayLike | {type(joint_velocities)} given"
        assert len(joint_positions) == 6, f"Joint Positions Length must be 6 | {len(joint_positions)} given \nJoint Positions:\n{joint_positions}"
        assert len(joint_velocities) == 6, f"Joint Velocities Length must be 6 | {len(joint_velocities)} given \nJoint Velocities:\n{joint_velocities}"

        return self.J_dot(joint_positions, joint_velocities)

    def JacobianInverse(self, joint_positions:ArrayLike) -> np.ndarray:

        """ Get Jacobian Inverse Matrix """
        return np.linalg.inv(self.Jacobian(joint_positions))

    def getMaxJointVelocity(self) -> np.ndarray:

        """ Get Max Joint Velocity """

        return self.robot.qlim

    def plot(self, joint_pos:ArrayLike=[0, -pi/2, pi/2, -pi/2, -pi/2, 0]):

        """ Plot Robot Peter Corke Robotics Toolbox """

        # TODO: Enhance Simulation (add following trajectory)
        self.robot.plot(joint_pos, block=True)

        # https://github.com/petercorke/robotics-toolbox-python

        # import swift
        # import roboticstoolbox as rtb
        # import spatialmath as sm
        # import numpy as np

        # env = swift.Swift()
        # env.launch(realtime=True)

        # panda = rtb.models.Panda()
        # panda.q = panda.qr

        # Tep = panda.fkine(panda.q) * sm.SE3.Trans(0.2, 0.2, 0.45)

        # arrived = False
        # env.add(panda)

        # dt = 0.05

        # while not arrived:

        #     v, arrived = rtb.p_servo(panda.fkine(panda.q), Tep, 1)
        #     panda.qd = np.linalg.pinv(panda.jacobe(panda.q)) @ v
        #     env.step(dt)

        # # Uncomment to stop the browser tab from closing
        # # env.hold()

    def plan_trajectory(self, start:Union[List[float], Pose, SE3], end:Union[List[float], Pose, SE3], duration:int=10, sampling_freq:int=500) -> Trajectory:

        """ Plan Trajectory with Peter Corke Robotics Toolbox """

        # Type Assertion
        assert type(sampling_freq) is int, f"Sampling Frequency must be an Integer | {type(sampling_freq)} given"
        assert type(duration) in [int, float], f"Duration must be a Int or Float | {type(duration)} given"

        # Convert Geometry Pose, SE3 Matrix to Joint Position Array
        start_joint_pos = self.InverseKinematic(start).q if type(start) in [Pose, SE3] else start
        end_joint_pos   = self.InverseKinematic(end).q   if type(end)   in [Pose, SE3] else end

        # Assertion
        assert len(start_joint_pos) == 6, f"Start Joint Positions Length must be 6 | {len(start_joint_pos)} given"
        assert len(end_joint_pos) == 6, f"End Joint Positions Length must be 6 | {len(end_joint_pos)} given"
        assert duration > 0, f"Duration must be greater than 0 | {duration} given"
        assert sampling_freq > 0, f"Sampling Frequency must be greater than 0 | {sampling_freq} given"

        # Create Time Vector (steps = duration * sampling_freq)
        time_vector = np.linspace(0, duration, duration * sampling_freq)

        # Return Trajectory
        return jtraj(np.array(start_joint_pos), np.array(end_joint_pos), time_vector)

    def plan_cartesian_trajectory(self, start_pose:Pose, end_pose:Pose, duration:float=10.0, steps:int=1000) -> List[SE3]:

        """ Plan Cartesian Trajectory with Peter Corke Robotics Toolbox """

        # Type Assertion
        assert type(steps) is int, f"Steps must be an Integer | {type(steps)} given"
        assert type(duration) in [int, float], f"Duration must be a Int or Float | {type(duration)} given"

        # Convert Geometry Pose to SE3 Matrix
        if type(start_pose) is Pose: start_pose = self.pose2matrix(start_pose)
        if type(end_pose)   is Pose: end_pose   = self.pose2matrix(end_pose)

        # Assertion
        assert duration > 0, f"Duration must be greater than 0 | {duration} given"
        assert steps > 0, f"Steps must be greater than 0 | {steps} given"

        # Create Time Vector
        time_vector = np.linspace(0, duration, steps)

        # Return Cartesian Trajectory (Array of SE3)
        return ctraj(start_pose, end_pose, time_vector)

    def joint2cartesianTrajectory(self, joint_trajectory:Trajectory) -> Tuple[List[SE3], np.ndarray, np.ndarray]:

        """ Convert Joint Trajectory to Cartesian Trajectory """

        # Type Assertion
        assert type(joint_trajectory) is Trajectory, f"Joint Trajectory must be a Trajectory | {type(joint_trajectory)} given"

        # Initialize Cartesian Trajectory
        cartesian_positions, cartesian_velocities, cartesian_accelerations = [], [], []

        # Print Joint Trajectory Shape
        if self.complete_debug: print(f'Joint Trajectory Shape | Positions: {joint_trajectory.q.shape} | Velocities: {joint_trajectory.qd.shape} | Accelerations: {joint_trajectory.qdd.shape}')

        # Convert Joint to Cartesian Positions
        for q, q_dot, q_ddot in zip(np.asarray(joint_trajectory.q, dtype=np.double), np.asarray(joint_trajectory.qd, dtype=np.double), np.asarray(joint_trajectory.qdd, dtype=np.double)):

            # Convert Joint Position to Cartesian Position (x = ForwardKinematic(q))
            x = self.ForwardKinematic(q)
            cartesian_positions.append(x)

            # Convert Joint Velocity to Cartesian Velocity (x_dot = Jacobian(q) * q_dot)
            x_dot = self.Jacobian(q) @ q_dot
            cartesian_velocities.append(x_dot)

            # Convert Joint Acceleration to Cartesian Acceleration (x_ddot = Jacobian(q) * q_ddot + JacobianDot(q, q_dot) * q_dot)
            x_ddot = self.Jacobian(q) @ q_ddot + self.JacobianDot(q, q_dot) @ q_dot
            cartesian_accelerations.append(x_ddot)

        if self.complete_debug: print(f'Cartesian Trajectory Shape | Positions: {np.array(cartesian_positions).shape} | Velocities: {np.array(cartesian_velocities).shape} | Accelerations: {np.array(cartesian_accelerations).shape}\n')
        if self.debug: print (colored('Cartesian Positions:', 'green'),     f'\n\n {np.array(cartesian_positions)}\n')
        if self.debug: print (colored('Cartesian Velocities:', 'green'),    f'\n\n {np.array(cartesian_velocities)}\n')
        if self.debug: print (colored('Cartesian Accelerations:', 'green'), f'\n\n {np.array(cartesian_accelerations)}\n')

        return cartesian_positions, np.array(cartesian_velocities), np.array(cartesian_accelerations)

    def get_cartesian_goal(self, splines:Tuple[CubicSpline, CubicSpline, CubicSpline], current_time:float=0.0, scaling_factor:float=1.0, rate:int=500, override_pose:Optional[Pose]=None) -> Tuple[List[SE3], np.ndarray, np.ndarray]:

        """ Convert Joint Trajectory Spline Point to Cartesian Trajectory Point """

        # Compute Time based on Scaling Factor
        time = current_time + scaling_factor / rate

        # Check Last Time Point
        time = min(splines[0].x[-1], time)

        # Get q, qd, qdd from Splines
        q, q_dot, q_ddot = splines[0](time), splines[1](time), splines[2](time)

        # Convert Joint to Cartesian Position, Velocity, Acceleration | x = ForwardKinematic(q) | x_dot = Jacobian(q) * q_dot | x_ddot = Jacobian(q) * q_ddot + JacobianDot(q, q_dot) * q_dot
        x, x_dot, x_ddot = self.ForwardKinematic(q), np.array(self.Jacobian(q) @ q_dot), np.array(self.Jacobian(q) @ q_ddot + self.JacobianDot(q, q_dot) @ q_dot)

        # Override Pose if time >= last_spline_time
        if override_pose is not None and type(override_pose) is Pose and time >= splines[0].x[-1]: x = self.pose2matrix(override_pose)

        return (x, x_dot, x_ddot), time

    def trajectory2spline(self, joint_trajectory:Trajectory) -> Tuple[CubicSpline, CubicSpline, CubicSpline]:

        """ Convert Joint Trajectory to Spline """

        # Type Assertion
        assert type(joint_trajectory) is Trajectory, f"Joint Trajectory must be a Trajectory | {type(joint_trajectory)} given"
        assert len(joint_trajectory.q) == len(joint_trajectory.qd) == len(joint_trajectory.qdd), f"Joint Trajectory Lengths must be Equal | {len(joint_trajectory.q)} | {len(joint_trajectory.qd)} | {len(joint_trajectory.qdd)} given"
        assert len(joint_trajectory.t) == len(joint_trajectory.q), f"Joint Trajectory Time Length must be Equal to Joint Trajectory Length | {len(joint_trajectory.t)} | {len(joint_trajectory.q)} given"

        # Create Cubic Spline for q, qd, qdd
        spline_q, spline_qd, spline_qdd = CubicSpline(joint_trajectory.t, joint_trajectory.q, axis=0), CubicSpline(joint_trajectory.t, joint_trajectory.qd, axis=0), \
                                          CubicSpline(joint_trajectory.t, joint_trajectory.qdd, axis=0)

        # Spline Plot
        if self.complete_debug: self.spline_plot([spline_q, spline_qd, spline_qdd], ['q','qd','qdd'], joint_trajectory.t)

        # Return Splines
        return spline_q, spline_qd, spline_qdd

    def spline_plot(self, splines:List[CubicSpline], names:List[str], time:np.ndarray, num_point:int=1000):

        """ Plot Splines """

        import matplotlib.pyplot as plt

        # Compute New Time Vector - New Vector from Spline
        new_time = np.linspace(time[0], time[-1], 1000)
        new_vectors = [spline(new_time) for spline in splines]

        # Plot Spline
        plt.figure(figsize=(10, 6))

        for i, name in enumerate(names):

            plt.subplot(3, 1, i+1)
            plt.plot(new_time, new_vectors[i], linewidth=2)
            plt.title(f'Cubic Spline for {name}')
            plt.xlabel('Time')
            plt.ylabel(f'{name}')

        plt.tight_layout()
        plt.show()

    def pose2array(self, pose:Pose) -> np.ndarray:

        """ Convert Pose to Numpy Array (6x1) """

        # Type Assertion
        assert type(pose) is Pose, f"Pose must be a Pose | {type(pose)} given | {pose}"

        # Convert Position to Numpy Array
        numpy_pose = np.zeros((6,))
        numpy_pose[:3] = [pose.position.x, pose.position.y, pose.position.z]

        # Convert Orientation to Numpy Array
        orientation:Rotation = Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        numpy_pose[3:] = orientation.as_rotvec()

        return numpy_pose

    def pose2matrix(self, pose:Pose) -> SE3:

        """ Convert Pose to SE3 Transformation Matrix """

        # Type Assertion
        assert type(pose) is Pose, f"Pose must be a Pose | {type(pose)} given | {pose}"

        # Convert Pose to Rotation Matrix and Translation Vector -> Create Transformation Matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_matrix()
        transformation_matrix[:3, 3] = np.array([pose.position.x, pose.position.y, pose.position.z])

        assert transformation_matrix.shape == (4, 4), f'Transformation must a 4x4 Matrix | {transformation_matrix.shape} obtained'
        return SE3(transformation_matrix)

    def matrix2pose(self, matrix:Union[np.ndarray, SE3]) -> Pose:

        """ Convert SE3 Transformation Matrix to Pose """

        # Convert Numpy Array to SE3 Matrix
        if type(matrix) is np.ndarray: matrix = SE3(matrix)

        # Type Assertion
        assert type(matrix) is SE3, f"Matrix must be a SE3 | {type(matrix)} given | {matrix}"

        # Convert Rotation Matrix to Quaternion (x,y,z,w)
        quaternion = Rotation.from_matrix(matrix.R).as_quat()

        # Create Pose
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = matrix.t
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion

        return pose

    def matrix2array(self, matrix:SE3) -> np.ndarray:

        """ Convert SE3 Transformation Matrix to NumPy Array (6x1) """

        # Type Assertion
        assert type(matrix) is SE3, f"Matrix must be a SE3 | {type(matrix)} given | {matrix}"

        # Convert Rotation Matrix to Rotation
        rotation:Rotation = Rotation.from_matrix(matrix.R)

        # Create Numpy Pose Array
        numpy_array = np.zeros((6,))
        numpy_array[:3] = matrix.t
        numpy_array[3:] = rotation.as_rotvec()

        return numpy_array

    def matrix2numpy(self, matrix:SE3) -> np.ndarray:

        """ Convert SE3 Transformation Matrix to NumPy Matrix (4x4) """

        # Type Assertion
        assert type(matrix) is SE3, f"Matrix must be a SE3 | {type(matrix)} given | {matrix}"

        # Convert to Numpy
        return matrix.A

    def array2matrix(self, array:np.ndarray) -> SE3:

        """ Convert NumPy Array to SE3 Transformation Matrix """

        # Type Assertion
        assert type(array) is np.ndarray, f"Array must be a NumPy Array | {type(array)} given | {array}"
        assert len(array) == 6, f"Array Length must be 6 | {len(array)} given | {array}"

        # Create Transformation Matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, 3] = array[:3]
        transformation_matrix[:3, :3] = Rotation.from_rotvec(array[3:]).as_matrix()

        # Convert to SE3 Matrix
        return SE3(transformation_matrix)
