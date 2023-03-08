import numpy

#####robotics class #####
class curi_robotics:
    def __init__(self, joint_size, joint_type, a, alpha, d, theta):
        self.JOINT_SIZE = joint_size
        self.A = a
        self.ALPHA = alpha
        self.D = d
        self.THETA = theta
        if joint_type == []:
            self.JOINT_TYPE = numpy.zeros((joint_size))
        else:
            self.JOINT_TYPE = joint_type
        return
    
    # other functions
    def RotX(self, theta):
        ans = numpy.array([[ 1,                 0,                 0],
                           [ 0, +numpy.cos(theta), -numpy.sin(theta)],
                           [ 0, +numpy.sin(theta), +numpy.cos(theta)]])
        return ans
    
    def RotY(self, theta):
        ans = numpy.array([[ +numpy.cos(theta), 0, +numpy.sin(theta)],
                           [                 0, 1,                 0],
                           [ -numpy.sin(theta), 0, +numpy.cos(theta)]])
        return ans
    
    def RotZ(self, theta):
        ans = numpy.array([[ +numpy.cos(theta), -numpy.sin(theta), 0],
                           [ +numpy.sin(theta), +numpy.cos(theta), 0],
                           [                 0,                 0, 1]])
        return ans
    
    def RPY2Mat(self, RPY): # [x, y, z] = [roll, pitch, yaw]
        return numpy.dot(self.RotZ(RPY[2]), numpy.dot(self.RotY(RPY[1]), self.RotX(RPY[0])))
    
    def Mat2RPY(self, mat):
        sy = numpy.sqrt(mat[0, 0] * mat[0, 0] + mat[1, 0] * mat[1, 0]);
        if sy > 1e-6:
            x = numpy.arctan2(+mat[2, 1], +mat[2, 2])
            y = numpy.arctan2(-mat[2, 0], +sy)
            z = numpy.arctan2(+mat[1, 0], +mat[0, 0])
        else:
            x   = numpy.arctan2(-mat[1, 2], +mat[1, 1])
            y = numpy.arctan2(-mat[2, 0], +sy)
            z   = 0
        return numpy.array([x, y, z]) # [x, y, z] = [roll, pitch, yaw]
    
    def Mat2AxisAngle(self, R):
        acosinput = (numpy.trace(R) - 1) / 2.0;
        if acosinput >= 1:
            return numpy.array([0.0, 0.0, 0.0])
        elif acosinput <= -1:
            if numpy.linalg.norm(1 + R[2, 2]) > 1e-6:
                omg = (1 / numpy.sqrt(2 * (1 + R[2, 2]))) * numpy.array([R[0, 2], R[1, 2], 1 + R[2, 2]])
            elif numpy.linalg.norm(1 + R[1, 1]) > 1e-6:
                omg = (1 / numpy.sqrt(2 * (1 + R[1, 1]))) * numpy.array([R[0, 1], 1 + R[1, 1], R[2, 1]])
            else:
                omg = (1 / numpy.sqrt(2 * (1 + R[0, 0]))) * numpy.array([1 + R[0, 0], R[1, 0], R[2, 0]])
            so3mat = self.VecToso3(numpy.pi * omg)
        else:
            theta = numpy.arccos(acosinput)
            so3mat = theta / 2.0 / numpy.sin(theta) * (R - R.transpose())
        # print('Axis and Angle', theta, so3mat)
        return self.so3ToVec(so3mat)
    
    def AxisAngle2Mat(self, omgtheta):
        theta = numpy.linalg.norm(omgtheta)
        if theta < 1e-6:
            return numpy.eye(3)
        else:
            so3mat = self.VecToso3(omgtheta)
            omgmat = so3mat / theta
            return numpy.eye(3) + numpy.sin(theta) * omgmat + (1 - numpy.cos(theta)) * numpy.dot(omgmat, omgmat)
    
    def VecToso3(self, omg):
        return numpy.array([[ 0, -omg[2], omg[1]], [ omg[2], 0, -omg[0]], [ -omg[1], omg[0], 0]])
    
    def so3ToVec(self, so3mat):
        return numpy.array([so3mat[2, 1], so3mat[0, 2], so3mat[1, 0]])
    
    def InvT(self, T):
        R = T[0:3, 0:3]
        p = T[0:3, 3]
        ans = numpy.zeros((4, 4))
        ans[0:3, 0:3] = R.transpose()
        ans[0:3, 3] = -numpy.dot(R.transpose(), p)
        return ans
    
    # modify DH method (Creig`s book)
    def A1(self, theta, d):
        ans = numpy.array([[+numpy.cos(theta), -numpy.sin(theta), 0, 0],
                           [+numpy.sin(theta), +numpy.cos(theta), 0, 0],
                           [                0,                 0, 1, d],
                           [                0,                 0, 0, 1]])
        return ans
    
    def A2(self, alpha, a):
        ans = numpy.array([[1,                 0,                 0, a],
                           [0, +numpy.cos(alpha), -numpy.sin(alpha), 0],
                           [0, +numpy.sin(alpha), +numpy.cos(alpha), 0],
                           [0,                 0,                 0, 1]])
        return ans
    
    # modify DH method (Creig's book)
    #
    # i-1         i         
    #  +----------+  Oi
    #             |         i+1
    #             +----------+  Qi+1  
    #                       
    def MDH(self, a, alpha, d, theta):
        return numpy.dot(self.A2(alpha, a), self.A1(theta, d))
    
    def MFK(self, theta, id=-1):
        if id == -1:
            id = self.JOINT_SIZE
        T = numpy.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        for k in range(id):
            if self.JOINT_TYPE[k] == 0:
                T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k], self.THETA[k]+theta[k]))
            else:
                T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k]+theta[k], self.THETA[k]))
        return T
    
    def MDK(self, theta, id=-1):
        if id == -1:
            id = self.JOINT_SIZE
        Te = self.MFK(theta, id)
        T = numpy.zeros((4, 4))
        J = numpy.zeros((6, id))
        T = numpy.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        for k in range(0, id):
            if self.JOINT_TYPE[k] == 0:
                T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k], self.THETA[k]+theta[k]))
                J[0:3, k] = numpy.cross(T[0:3, 2], Te[0:3, 3] - T[0:3, 3])
                J[3:6, k] = T[0:3, 2]
            else:
                T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k]+theta[k], self.THETA[k]))
                J[0:3, k] = T[0:3, 2]
        return J
    
    def FunOriErrAR(self, Rc, Rt):
        Re = numpy.dot(Rc.transpose(), Rt)
        e = 0.5 * numpy.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
        eo = numpy.dot(Rc, e)
        return eo
    
    def MIK(self, Rt, Pt, q, iterate_times = 50):
        q_ans = q.copy()
        Tc = self.MFK(q)
        dv = Pt - Tc[0:3, 3]
        dw = self.FunOriErrAR(Tc[0:3, 0:3], Rt[0:3, 0:3])
        count = 0
        while (numpy.linalg.norm(dv) > 1e-5 or numpy.linalg.norm(dw) > 1e-3) and count < iterate_times:
            J = self.MDK(q)
            if abs(numpy.linalg.matrix_rank(J)) < 6:
                print('singularity')
                return q_ans
            
            dx = numpy.array([dv[0], dv[1], dv[2], dw[0], dw[1], dw[2]])
            dq = numpy.dot(numpy.linalg.pinv(J), dx) * 0.5
            
            # methord 1 set robot to 6 dof
            if self.JOINT_SIZE > 6:
                dq[6] = 0
            q = q + dq.flatten()
            
            Tc = self.MFK(q)
            dv = Pt - Tc[0:3, 3]
            dw = self.FunOriErrAR(Tc[0:3, 0:3], Rt[0:3, 0:3])
            count = count + 1
        
        print('iterates ', count, 'times')
        if count >= iterate_times:
            print('iterates more than ' + str(iterate_times) + ' times')
            return q
        return q

    def get_adimitance_control_state(force):
        k = [[], [], [], [], [], []]
        v_max = [0, 0, 0, 0, 0, 0]
        C = [0, 0, 0, 0, 0, 0]
        v = [0, 0, 0, 0, 0, 0]
        for i in range(6):
            v[i] = self.admintance_control_bspline(force[i], v_max[i], k[i, 1], k[i, 2], k[i, 3], k[i, 4], C)
        
    def admintance_control_bspline(f, v_max, k1, k2, k3, k4, C):
        v = 0
        if f < -k4:
            v = -v_max
        elif f < -k3:
            v = -v_max + C * numpy.power(f + k4, 3)
        elif f < -k2:
            v = 3 * C * numpy.power(k2 - k1, 2) * (f + k2) - C * numpy.power(k2 - k1, 3)
        elif f < -k1:
            v = C * numpy.power(f + k1, 3)
        elif f < k1:
            v = 0
        elif f < k2:
            v = C * numpy.power(f - k1, 3)
        elif f < k3:
            v = 3 * C * numpy.power(k2 - k1, 2) * (f - k2) - C * numpy.power(k2 - k1, 3)
        elif f < k4:
            v = v_max + C * numpy.power(f - k4, 3)
        else:
            v = v_max

        return v

    def draw(self, q, ax, show_frame=False):
        scale = 0.01
        ax.bar3d(-5*scale, -5*scale, -3*scale, 10*scale, 10*scale, 3*scale, color='gray')
        T = numpy.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        P = T[0:3, 3]
        R = T[0:3, 0:3]
        ax.quiver(-0.3, 0.3, 0, R[0, 0]*scale*5, R[1, 0]*scale*5, R[2, 0]*scale*5, length=0.1, normalize=True, color='r')
        ax.quiver(-0.3, 0.3, 0, R[0, 1]*scale*5, R[1, 1]*scale*5, R[2, 1]*scale*5, length=0.1, normalize=True, color='g')
        ax.quiver(-0.3, 0.3, 0, R[0, 2]*scale*5, R[1, 2]*scale*5, R[2, 2]*scale*5, length=0.1, normalize=True, color='b')
        ax.text(-0.3 + R[0, 0]*scale*15, 0.3 + R[1, 0]*scale*15, R[2, 0]*scale*15, 'x')
        ax.text(-0.3 + R[0, 1]*scale*15, 0.3 + R[1, 1]*scale*15, R[2, 1]*scale*15, 'y')
        ax.text(-0.3 + R[0, 2]*scale*10, 0.3 + R[1, 2]*scale*10, R[2, 2]*scale*10, 'z')
        for k in range(0, 6):
            T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k], q[k]));
            Q = P
            P = T[0:3, 3]
            R = T[0:3, 0:3]
            # draw line
            ax.plot([Q[0], P[0]], [Q[1], P[1]], [Q[2], P[2]], linewidth=5, color='orange')
            
            # draw cylinder
            n = 20;
            u = numpy.linspace(0, 2*numpy.pi, n)
            x = numpy.array([numpy.cos(u)*scale*3, numpy.cos(u)*scale*3])
            y = numpy.array([numpy.sin(u)*scale*3, numpy.sin(u)*scale*3])
            z = numpy.array([[-scale*3]*n, [+scale*3]*n])

            for j in range(0, n):
                xx0 = R[0, 0]*x[0][j] + R[0, 1]*y[0][j] + R[0, 2]*z[0][j] + P[0]
                yy0 = R[1, 0]*x[0][j] + R[1, 1]*y[0][j] + R[1, 2]*z[0][j] + P[1]
                zz0 = R[2, 0]*x[0][j] + R[2, 1]*y[0][j] + R[2, 2]*z[0][j] + P[2]

                xx1 = R[0, 0]*x[1][j] + R[0, 1]*y[1][j] + R[0, 2]*z[1][j] + P[0]
                yy1 = R[1, 0]*x[1][j] + R[1, 1]*y[1][j] + R[1, 2]*z[1][j] + P[1]
                zz1 = R[2, 0]*x[1][j] + R[2, 1]*y[1][j] + R[2, 2]*z[1][j] + P[2]

                x[0][j] = xx0
                y[0][j] = yy0
                z[0][j] = zz0

                x[1][j] = xx1
                y[1][j] = yy1
                z[1][j] = zz1
            # ax.plot_surface(x, y, z, color='#5555FF88')
            ax.plot_surface(x, y, z, color='pink')
            
            # draw coordinate
            if show_frame == True:
                ax.quiver(P[0], P[1], P[2], R[0, 0]*scale*5, R[0, 1]*scale*5, R[0, 2]*scale*5, length=0.1, normalize=True, color='r')
                ax.quiver(P[0], P[1], P[2], R[1, 0]*scale*5, R[1, 1]*scale*5, R[1, 2]*scale*5, length=0.1, normalize=True, color='g')
                ax.quiver(P[0], P[1], P[2], R[2, 0]*scale*5, R[2, 1]*scale*5, R[2, 2]*scale*5, length=0.1, normalize=True, color='b')

        #ax.set_xlabel('X (m)')
        ax.set_xlim(-0.5, 0.5)
        #ax.set_ylabel('Y (m)')
        ax.set_ylim(-0.5, 0.5)
        #ax.set_zlabel('Z (m)')
        ax.set_zlim(-0.3, 0.7)
        ax.view_init(azim=135)
        return ax
