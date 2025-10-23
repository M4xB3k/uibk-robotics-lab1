from __future__ import annotations
from typing import List
import numpy as np
# DO NOT ADD ANY OTHER IMPORT


class UnitQuaternion:

    def __init__(self, eta: float, epsilon: np.ndarray):
        """
        Unit quaternion.
        :param eta: scalar part of the unit quaternion
        :param epsilon: 3 element vector of the unit quaternion
        """
        self.eta = eta
        self.epsilon = epsilon

    def __matmul__(self, other: UnitQuaternion) -> UnitQuaternion:
        """
        Composition of quaternions [not necessary for this assignment]
        :param other:
        :return:
        """
        raise NotImplementedError()

    def invert(self) -> UnitQuaternion:
        """
        Inversion of quaternions [not necessary for this assignment]
        :param other:
        :return:
        """
        raise NotImplementedError()

    def as_matrix(self) -> np.ndarray:
        """
        Return a 3x3 rotation matrix from the unit quaternion.
        :return:
        """
        raise NotImplementedError()

    def as_vector(self):
        """
        Returns a vector where the first three elements are the vector part, and the last is the scalar part of the
        unit quaternion
        :return:
        """
        raise NotImplementedError()

    def error(self, desired_uq: UnitQuaternion) -> np.ndarray:
        """
        Computer a 3-element array containing the error of the desired unit quaternion.
        This method is useful for the inverse kinematic algorithm
        :param desired_uq:
        :return:
        """
        raise NotImplementedError()

    @staticmethod
    def from_matrix(matrix: np.ndarray) -> UnitQuaternion:
        """
        Returns a unit quaternion from a rotation or homogeneous transformation matrix.
        :param matrix:
        :return:
        """
        raise NotImplementedError()


class RobotKinematics:

    def __init__(self, dh_parameters: List[dict], position_gains: float = 1.0, orientation_gains: float = 0.1):
        """
        Robot kinematics updates the transformation matrixes given a particular joint configuration and compute the
        joint velocity for a desired pose expressed in ZYZ intrinsic Euler angles.

        Robot kinematics needs a list of DH parameters, where the DH parameters are expressed using dictionaries
        containing the keys: 'a', 'alpha', 'd', 'theta', and 'joint' (which defines if the joint is revolute or
        prismatic)
        :param dh_parameters: list of dictionaries containing the DH parameters
        :param position_gains: how important is the desired position
        :param orientation_gains: how important is the desired orientation (set to zero for debugging when needed)
        """
        self.dh_parameters = dh_parameters  # DH parameters
        self.dof = len(dh_parameters)  # number of degrees of freedom == number of joints
        self.relative_transformations = []  # list of "relative" transformation matrixes, i.e., T_1^0, T_2^1, ...
        self.cumulative_transformations = []  # list of "total" transformation matrixes, i.e., T_1^0, T_2^0, ...
        self.jacobian = np.zeros((6, 7))
        self.K = None  # type: np.ndarray
        # 6x6 diagonal matrix. 3 first element in the diagonal correspont to the position, the last three to
        # the orientation. Implement it. TODO

    def update(self, joint_configuration: np.ndarray):
        """
        Update self.tansformations with T_0^0, T_1^0, T_2^0, ..., T_e^0
        :param joint_configuration: array containing all the joint configurations, ordered from the first to the last
        :return: None.
        """
        self.relative_transformations = [self.get_homogeneous_transformation(i, joint_configuration)
                                         for i in range(self.dof + 1)]
        self.cumulative_transformations = self.get_cumulative_transformation(joint_configuration)

    def get_homogeneous_transformation(self, n, joint_configuration: np.ndarray) -> np.ndarray:
        """
        This function returns T_{i}^{i-1}, expect when i=0, then returns T_0^0
        :param dh_parameters: Denavit-Hartenberg parameters organized as list of dictionaries
        :param n: number of the transformation.
        :param joint_configuration: (joint configuration in radiants)
        :return: 4x4 homogeneous transformation matrix
        """
        raise NotImplementedError()

    def get_cumulative_transformation(self, joint_configuration: np.ndarray) -> List[np.ndarray]:
        """
        Returns a list of T_0^0, T_1^0, T_2^0, T_3^0, ..., T_e^0
        :param joint_configuration:
        :return: list of homogeneous transformation matrixes
        """
        raise NotImplementedError()

    @staticmethod
    def get_zyz_euler(homogeneour_transformation: np.ndarray) -> np.ndarray:
        """
        returns Euler angles in the range -pi/2, pi/2 from a homogeneous transformation
        :param homogeneour_transformation:
        :return: a 3-element column array
        """
        raise NotImplementedError()

    @staticmethod
    def get_uq_from_euler(euler: np.ndarray) -> UnitQuaternion:
        """
        Get the unit quaternion from intrinsic ZYZ angles
        :param euler: ZYZ euler rotation
        :return: Unit quaternion rotation
        """
        raise NotImplementedError()

    def get_uq_pose(self):
        """
        Get the end-effector pose in unit quaternions
        :return:
        """
        raise NotImplementedError()

    def get_geometric_jacobian(self) -> np.ndarray:
        """
        Get the geometric Jacobian
        :param joint_configuration:
        :return: 6xn ndarray
        """
        raise NotImplementedError()

    def get_elementary_ik(self, desired_position, desired_euler, damping_coefficient):
        """
        This function return the joint velocity for a particular desired pose expressed with ZYZ intrinsic Euler angles
        The damping coefficient serves to avoid inversion of singular matrixes.

        The inverse kinematics is done by using the quaternion error (see page 140 Siciliano et al.).
        :param desired_position: 3-element array
        :param desired_euler: 3-element array ZYZ intrinsic Euler
        :param damping_coefficient:
        :return:
        """
        raise NotImplementedError()
