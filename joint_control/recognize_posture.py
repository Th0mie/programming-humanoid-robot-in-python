'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import leftBackToStand
import os
import pickle


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.dir = os.path.dirname(__file__) #absolute dir the script is in
        pose_path = os.path.join(self.dir, 'robot_pose.pkl')
        self.posture_classifier = pickle.load(open(pose_path, 'rb'))  # LOAD YOUR CLASSIFIERR

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        # YOUR CODE HERE
        #print(perception.joint['LHipYawPitch'])
        posture = []
        joints = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']

        for i in range(len(joints)):
            posture.append(perception.joint[joints[i]])

        posture.append(perception.imu[0])
        posture.append(perception.imu[1])

        prediction = self.posture_classifier.predict([posture])
        #print(prediction[0])
        #print(os.path.join(self.dir, 'robot_pose_data'))
        pose_name = os.path.join(self.dir, 'robot_pose_data')

        return os.listdir(pose_name)[prediction[0]]

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
