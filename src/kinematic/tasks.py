from invoke import task

# Get Lib Path
from pathlib import Path
LIB_PATH = f'{str(Path(__file__).resolve().parents[2])}/lib'

EIGEN_PATH = '/usr/include/eigen3'

LIB_NAMES = [
    f'{LIB_PATH}/ur3_kinematic/compute_UR3_direct_kinematic.so',
    f'{LIB_PATH}/ur3_kinematic/compute_UR3_jacobian.so',
    f'{LIB_PATH}/ur3_kinematic/compute_UR3_jacobian_dot.so',
    f'{LIB_PATH}/ur3_kinematic/compute_UR3_jacobian_dot_dq.so',
    f'{LIB_PATH}/ur5_kinematic/compute_UR5_direct_kinematic.so',
    f'{LIB_PATH}/ur5_kinematic/compute_UR5_jacobian.so',
    f'{LIB_PATH}/ur5_kinematic/compute_UR5_jacobian_dot.so',
    f'{LIB_PATH}/ur5_kinematic/compute_UR5_jacobian_dot_dq.so',
    f'{LIB_PATH}/ur10_kinematic/compute_UR10_direct_kinematic.so',
    f'{LIB_PATH}/ur10_kinematic/compute_UR10_jacobian.so',
    f'{LIB_PATH}/ur10_kinematic/compute_UR10_jacobian_dot.so',
    f'{LIB_PATH}/ur10_kinematic/compute_UR10_jacobian_dot_dq.so',
    f'{LIB_PATH}/ur3e_kinematic/compute_UR3e_direct_kinematic.so',
    f'{LIB_PATH}/ur3e_kinematic/compute_UR3e_jacobian.so',
    f'{LIB_PATH}/ur3e_kinematic/compute_UR3e_jacobian_dot.so',
    f'{LIB_PATH}/ur3e_kinematic/compute_UR3e_jacobian_dot_dq.so',
    f'{LIB_PATH}/ur5e_kinematic/compute_UR5e_direct_kinematic.so',
    f'{LIB_PATH}/ur5e_kinematic/compute_UR5e_jacobian.so',
    f'{LIB_PATH}/ur5e_kinematic/compute_UR5e_jacobian_dot.so',
    f'{LIB_PATH}/ur5e_kinematic/compute_UR5e_jacobian_dot_dq.so',
    f'{LIB_PATH}/ur10e_kinematic/compute_UR10e_direct_kinematic.so',
    f'{LIB_PATH}/ur10e_kinematic/compute_UR10e_jacobian.so',
    f'{LIB_PATH}/ur10e_kinematic/compute_UR10e_jacobian_dot.so',
    f'{LIB_PATH}/ur10e_kinematic/compute_UR10e_jacobian_dot_dq.so',
    f'{LIB_PATH}/ur16e_kinematic/compute_UR16e_direct_kinematic.so',
    f'{LIB_PATH}/ur16e_kinematic/compute_UR16e_jacobian.so',
    f'{LIB_PATH}/ur16e_kinematic/compute_UR16e_jacobian_dot.so',
    f'{LIB_PATH}/ur16e_kinematic/compute_UR16e_jacobian_dot_dq.so',
]

SOURCES = [
    'ur3_kinematic/compute_UR3_direct_kinematic_so.cpp',
    'ur3_kinematic/compute_UR3_jacobian_so.cpp',
    'ur3_kinematic/compute_UR3_jacobian_dot_so.cpp',
    'ur3_kinematic/compute_UR3_jacobian_dot_dq_so.cpp',
    'ur5_kinematic/compute_UR5_direct_kinematic_so.cpp',
    'ur5_kinematic/compute_UR5_jacobian_so.cpp',
    'ur5_kinematic/compute_UR5_jacobian_dot_so.cpp',
    'ur5_kinematic/compute_UR5_jacobian_dot_dq_so.cpp',
    'ur10_kinematic/compute_UR10_direct_kinematic_so.cpp',
    'ur10_kinematic/compute_UR10_jacobian_so.cpp',
    'ur10_kinematic/compute_UR10_jacobian_dot_so.cpp',
    'ur10_kinematic/compute_UR10_jacobian_dot_dq_so.cpp',
    'ur3e_kinematic/compute_UR3e_direct_kinematic_so.cpp',
    'ur3e_kinematic/compute_UR3e_jacobian_so.cpp',
    'ur3e_kinematic/compute_UR3e_jacobian_dot_so.cpp',
    'ur3e_kinematic/compute_UR3e_jacobian_dot_dq_so.cpp',
    'ur5e_kinematic/compute_UR5e_direct_kinematic_so.cpp',
    'ur5e_kinematic/compute_UR5e_jacobian_so.cpp',
    'ur5e_kinematic/compute_UR5e_jacobian_dot_so.cpp',
    'ur5e_kinematic/compute_UR5e_jacobian_dot_dq_so.cpp',
    'ur10e_kinematic/compute_UR10e_direct_kinematic_so.cpp',
    'ur10e_kinematic/compute_UR10e_jacobian_so.cpp',
    'ur10e_kinematic/compute_UR10e_jacobian_dot_so.cpp',
    'ur10e_kinematic/compute_UR10e_jacobian_dot_dq_so.cpp',
    'ur16e_kinematic/compute_UR16e_direct_kinematic_so.cpp',
    'ur16e_kinematic/compute_UR16e_jacobian_so.cpp',
    'ur16e_kinematic/compute_UR16e_jacobian_dot_so.cpp',
    'ur16e_kinematic/compute_UR16e_jacobian_dot_dq_so.cpp',
]

@task
def build(ctx):

    for i in range(len(LIB_NAMES)):
        ctx.run(f"g++ -o3 -I{EIGEN_PATH} -shared -o {LIB_NAMES[i]} -fPIC {SOURCES[i]}")
