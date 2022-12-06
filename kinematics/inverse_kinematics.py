'''In this exercise you need to implement inverse kinematics for NAO's legs
* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
from math import atan2
import numpy as np



class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics
        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = {}
        # YOUR CODE HERE
        #print(transform)
        lambda_ = 1
        max_step = 0.1
        target = np.matrix(self.from_trans(transform)).T
        #print(target)
        
        for chain in self.chains:
            for joint in self.chains[chain]:
                joint_angles[joint] = self.perception.joint[joint]
        #print(joint_angles)

        for i in range(1000):
            Ts = self.forward_kinematics(joint_angles)


        return joint_angles

    def from_trans(self, m):
        #print("test")
        thetaX = 0
        thetaY = 0
        thetaZ = 0
        if m[0,0] == 1:
            thetaX = atan2(m[2,1], m[1,1])
        elif m[1,1] == 1:
            thetaY = atan2(m[0,2], m[0,0])
        elif m[2,2] == 1:
            thetaZ = atan2(m[1,0], m[0,0])
        return [m[3,0], m[3,1], m[3,2], thetaX, thetaY, thetaZ]

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE

        joints = self.inverse_kinematics(effector_name, transform)

        names = self.chains[effector_name]
        #print(names)
        times = list()
        keys = list()

        for i in range(len(names)):
            times.append([1, 5])
            keys.append([[self.perception.joint[names[i]], [3,0,0], [3,0,0]], [self.perception.joint[names[i]], [3,0,0], [3,0,0]]])
        #print(keys)

        self.keyframes = (names, times, keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()