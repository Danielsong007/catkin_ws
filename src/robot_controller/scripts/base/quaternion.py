import numpy as np
import math

class quaternion:
    """
    quaternion class with fucntions:
    
    norm: return the norm of the quaternion
    mul: quaternion multiplication
    type: multiplication
    
    """
    
    # static parameters
    eps = 1e-6

    def __init__(self, s = 1, x = 0, y = 0, z = 0):
        """
        Construction function, default is an identity quaternion
        
        :return: a constructed quaternion
        :type quaternion
        :param s: real part coorinate
        :type: float
        :param x: imaginary part coorinate of i
        :type: float
        :param y: imaginary part coorinate of j
        :type: float
        :param z: imaginary part coorinate of k
        :type: float
        """
        self.s, self.x, self.y, self.z = s, x, y, z
        return
    
    def norm(self):
        """
        Norm of the quaternion
        
        :return: norm
        :type: float
        """
        return np.linalg.norm(np.array([self.s, self.x, self.y, self.z]))

    def mul(self, q):
        """
        Quaternion multiplication
        
        :return: multiplication result
        :type: quaternion
        :param q: the quaternion on the right side
        :type: quaternion
        """
        s1 = self.s
        v1 = np.array([self.x, self.y, self.z])
        s2 = q.s
        v2 = np.array([q.x, q.y, q.z])
        v = s1 * v2 + s2 * v1 + np.cross(v1, v2)
        return quaternion(s1 * s2 - np.dot(v1, v2), v[0], v[1], v[2])

    def inverse(self):
        """
        Inverse of the quaternion

        :return: the inverse quaternion
        :type: quaternion
        """
        nm = self.norm()
        if abs(nm) < quaternion.eps:
            print("The norm is zero, and cannot solve the inverse.")
        return quaternion(self.s / nm, -self.x / nm, -self.y / nm, -self.z / nm)

    def conversions(self, p):
        """
        Vector rotation

        :return: the rotated vector
        :type: numpy.ndarry
        """
        if abs(self.norm() - 1) > quaternion.eps:
            print('This is not a unit quaternion, which means it cannot regard as a rotation operator.')
            return p

        len_p = len(p)
        if len_p < 2 or len_p > 4:
            print('This size of P is not correct, it must be a 3x1 or 4x1 vector.')
            return p
        
        # Simplify p' = q p q^-1 to p' = p + 2v x (v x p + sp)
        v = np.array([self.x, self.y, self.z])
        return p + 2 * np.cross(v, np.cross(v, p) + self.s * p) 

    def print(self):
        """
        Print quaternion to show it
        """
        print('quaternion: ', self.s, '<', self.x, ',', self.y, ',', self.z, '>')

if __name__ == '__main__':
    # help(quaternion)

    # initialization test
    print('test construction function:')
    q1 = quaternion()
    q1.print()
    q2 = quaternion(1, 2, 3, 4)
    q2.print()

    # multiplication test
    print('\ntest quaternion multiple function:')
    q3 = q1.mul(q2)
    q3.print()
    print()

    # inverse test
    print('\ntest quaternion inverse function:')
    q3.inverse()
    q3.print()
    print()

    # point conversions test
    print('\ntest rotation vector function:')
    v1 = np.array([1, 2, 3])
    q4 = quaternion(1/math.sqrt(2), 1/math.sqrt(2), 0, 0)
    v2 = q4.conversions(v1)
    print('rotation around x-axis, point from', v1, 'to', v2)

    q5 = quaternion(1/math.sqrt(2), 0, 1/math.sqrt(2), 0)
    v3 = q5.conversions(v1)
    print('rotation around y-axis, point from', v1, 'to', v3)

    q6 = quaternion(1/math.sqrt(2), 0, 0, 1/math.sqrt(2))
    v4 = q6.conversions(v1)
    print('rotation around z-axis, point from', v1, 'to', v4)
    
    q7 = quaternion(1/2, 1/2, 1/2, 1/2)
    v5 = q7.conversions(v1)
    print('rotation around all axes, point from', v1, 'to', v5)