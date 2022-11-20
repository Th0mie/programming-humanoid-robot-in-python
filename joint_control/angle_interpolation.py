'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import rightBackToStand


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.time = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        #target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def pointFinder(self, percent, A, B):
        point = [0,0]
        point[0] = A[0] - ((A[0] - B[0]) * percent)
        point[1] = A[1] - ((A[1] - B[1]) * percent)
        return point
        #m = (y1-y2)/(x1-x2)
        #b = (x1*y2 - x2*y1)/(x1-x2)
        #return m*point+b

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        #[float angle, [int InterpolationType, float dTime, float dAngle], [int InterpolationType, float dTime, float dAngle]]

        #print(keyframes)
        names = keyframes[0]
        times = keyframes[1]
        keys = keyframes[2]
        maxT = 0
        for i in range(len(times)):
            for j in range(len(times[i])):
                if times[i][j] > maxT:
                    maxT = times[i][j]

        if self.time == -1:
            self.time = perception.time
        dTime = perception.time - self.time

        #print(names)
        #i = 0
        #loop for each joint
        for n in range(len(names)):
            keyframeTime = times[n]
            #loop for each time
            for t in range(len(times[n])-1):
                #i = i+1
                #print(len(names))
                #print(n)
                #print(len(times[n]))
                #print(t)
                if keyframeTime[t] < dTime < keyframeTime[t+1]:
                    p0 = keys[n][t][0]
                    p1 = keys[n][t][0] + keys[n][t][2][2]
                    p2 = keys[n][t+1][0] + keys[n][t][1][2]
                    p3 = keys[n][t+1][0]
                    d = (dTime-times[n][t])/(times[n][t+1]-times[n][t])
                    target_joints[names[n]] = (1-d) ** 3 * p0 + 3 * (1-d) ** 2 * d * p1 + 3 * (1-d) * d ** 2 * p2 + d ** 3 * p3
                    #m = (keys[n][t][0]-keys[n][t+1][0])/(times[n][t]-times[n][t+1])
                    #b = (times[n][t]*keys[n][t+1][0] - times[n][t+1]*keys[n][t][0])/(times[n][t]-times[n][t+1])
                    #target_joints[names[n]] = m*dTime+b
                    #target_joints[names[n]] = self.pointFinder(dTime, times[n][t], keys[n][t][0], keys[n][t][2][1], keys[n][t][2][2])
                    #percent = (dTime-times[n][t])/(times[n][t+1]-times[n][t])
                    #target_joints[names[n]] = self.pointFinder(percent, [times[n][t], keys[n][t][0]], [times[n][t+1], keys[n][t+1][0]])[1]
                    #pointA = self.pointFinder(percent, [times[n][t], keys[n][t][0]], [keys[n][t][2][1]+times[n][t], keys[n][t][2][2]])
                    #pointB = self.pointFinder(percent, [keys[n][t][2][1]+times[n][t], keys[n][t][2][2]], [keys[n][t+1][1][1]+times[n][t+1], keys[n][t+1][1][2]])
                    #pointC = self.pointFinder(percent, [keys[n][t+1][2][1]+times[n][t+1], keys[n][t+1][2][2]], [times[n][t+1], keys[n][t+1][0]])
                    #pointD = self.pointFinder(percent, pointA, pointB)
                    #pointE = self.pointFinder(percent, pointB, pointC)
                    #target_joints[names[n]] = self.pointFinder(percent, pointD, pointE)[1]


                    #prozent = (dTime-times[n][t])/(times[n][t+1]-times[n][t])

                else:
                    continue

        if 'LHipYawPitch' in target_joints: target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        if dTime > maxT:
            self.time = -1
        #print('STOPPPPPPPPPPPPP')
        #if 'HeadPitch' in target_joints:
        #    print(target_joints['HeadPitch'])
        #    print(dTime)
        
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
