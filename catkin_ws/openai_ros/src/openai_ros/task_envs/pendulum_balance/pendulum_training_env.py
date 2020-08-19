from gym import spaces
from openai_ros.robot_envs import pendulum_robot_env
from gym import utils
from gym.envs.registration import register
from gym import error, spaces
from collections import deque
import rospy
import math
import numpy as np

# The path is __init__.py of openai_ros, where we import the MovingCubeOneDiskWalkEnv directly
timestep_limit_per_episode = 1000 # Can be any Value

register(
        id='PendulumBalance-v0',
        entry_point='openai_ros.task_envs.pendulum_balance.pendulum_training_env:MyTrainingEnv',
        max_episode_steps=1000,
    )

class MyTrainingEnv(pendulum_robot_env.PendulumEnv):
    def __init__(self):

        self.get_params()
        self.action_space = spaces.Discrete(self.n_actions)
        self.obs_last100 = deque(maxlen = 10)
        '''
        
        self.action_space = spaces.Discrete(self.n_actions)
        self.action_incr = []
        self.step_val = self.effort_resolution
        while(self.step_val <= self.max_base_effort):
            self.action_incr.append(self.step_val)
            self.action_incr.append(-self.step_val)
            self.step_val += self.effort_resolution
            self.step_val = round(self.step_val,3)
        '''
        high = np.array([
            self.max_base_pose_x * 2,
            np.finfo(np.float32).max,
            self.max_pole_angle * 2,
            np.finfo(np.float32).max])
        self.observation_space = spaces.Box(-high, high)

        pendulum_robot_env.PendulumEnv.__init__(
            self
            )

    def get_params(self):
        #get configuration parameters
        self.n_actions = rospy.get_param('/robo/n_actions')
        self.min_pole_angle = rospy.get_param('/robo/min_pole_angle')
        self.max_pole_angle = rospy.get_param('/robo/max_pole_angle')
        self.max_base_velocity = rospy.get_param('/robo/max_base_velocity')
        self.max_base_effort = rospy.get_param('/robo/max_base_effort')
        self.effort_resolution = rospy.get_param('/robo/effort_resolution')
        self.min_base_pose_x = rospy.get_param('/robo/min_base_pose_x')
        self.max_base_pose_x = rospy.get_param('/robo/max_base_pose_x')
        self.pos_step = rospy.get_param('/robo/pos_step')
        self.running_step = rospy.get_param('/robo/running_step')
        self.init_pos = rospy.get_param('/robo/init_pos')
        self.wait_time = rospy.get_param('/robo/wait_time')
        
        

    def _set_action(self, action):

        # Take action
        #self.pos[0] += self.action_incr[action]
        # Take action
        if action == 0: #LEFT
            rospy.logdebug("GO LEFT...")
            self.pos[0] -= self.pos_step
        elif action == 1: #RIGHT
            rospy.logdebug("GO RIGHT...")
            self.pos[0] += self.pos_step
        elif action == 2: #LEFT BIG
            rospy.logdebug("GO LEFT BIG...")
            self.pos[0] -= self.pos_step * 10
        elif action == 3: #RIGHT BIG
            rospy.logdebug("GO RIGHT BIG...")
            self.pos[0] += self.pos_step * 10


#        if action == 0: #LEFT
#            rospy.logdebug("GO LEFT...")
#            self.pos[0] -= self.pos_step
#            self.incr = -1
#        elif action == 1: #RIGHT
#            rospy.logdebug("GO RIGHT...")
#            self.pos[0] += self.pos_step
#            self.incr = 1
#        elif action == 2: #LEFT BIG
#            rospy.logdebug("GO LEFT BIG...")
#            self.pos[0] -= self.pos_step * 2
#        elif action == 3: #RIGHT BIG
#            rospy.logdebug("GO RIGHT BIG...")
#            self.pos[0] += self.pos_step * 2
#        elif action == 4: # NO CHANGE
#			rospy.logdebug("NO CHANGE...")

        # Apply action to simulation.
        rospy.logdebug("MOVING TO POS=="+str(self.pos))

        # 1st: unpause simulation
        #rospy.logdebug("Unpause SIM...")
        #self.gazebo.unpauseSim()

        self.move_joints(self.pos)
        rospy.logdebug("Wait for some time to execute movement, time="+str(self.running_step))
        rospy.sleep(self.running_step) #wait for some time
        rospy.logdebug("DONE Wait for some time to execute movement, time=" + str(self.running_step))

        # 3rd: pause simulation
        #rospy.logdebug("Pause SIM...")
        #self.gazebo.pauseSim()

    def _get_obs(self):

        data = self.joints
        #       pole_position                pole_velocity              arm angle                 arm velocity
        #obs = [round(data.position[1],1), round(data.velocity[1],1), round(data.position[0],1), round(data.velocity[0],1)]
        obs = [data.position[1], data.velocity[1], data.position[0], data.velocity[0]]
        return np.array(obs)

    def _is_done(self, observations):
        done = False
        data = self.joints

        rospy.logdebug("BASEPOSITION=="+str(observations[0]))
        rospy.logdebug("POLE ANGLE==" + str(observations[2]))
        if (self.min_base_pose_x >= observations[2] or observations[2] >= self.max_base_pose_x): #check if the base is still within the ranges of (-2, 2)
            rospy.logdebug("Base Outside Limits==>min="+str(self.min_base_pose_x)+",pos="+str(observations[0])+",max="+str(self.max_base_pose_x))
            done = True
        if (self.min_pole_angle >= observations[0] or observations[0] >= self.max_pole_angle): #check if pole has toppled over
            rospy.logdebug(
                "Pole Angle Outside Limits==>min=" + str(self.min_pole_angle) + ",pos=" + str(observations[2]) + ",max=" + str(
                    self.max_pole_angle))
            done = True

        rospy.logdebug("FINISHED get _is_done")

        return done

    def _compute_reward(self, observations, done):

        """
        Gives more points for staying upright, gets data from given observations to avoid
        having different data than other previous functions
        :return:reward
        """
        rospy.logdebug("START _compute_reward")
        self.obs_last100.append(observations[0])
        mean_obs = np.mean(self.obs_last100)
        if not done:
            for x in range(1,11):
                if abs(mean_obs) < x*0.0174533:              #Less than 1 degree
                    reward = 11.0-x
                    break 
            reward = 1.0
        elif self.steps_beyond_done is None:
            # Pole just fell!
            self.steps_beyond_done = 0
            reward = 1.0
        else:
            if self.steps_beyond_done == 0:
                logger.warning("You are calling 'step()' even though this environment has already returned done = True. You should always call 'reset()' once you receive 'done = True' -- any further steps are undefined behavior.")
            self.steps_beyond_done += 1
            reward = 0.0

        rospy.logdebug("END _compute_reward")

        return reward

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        self.steps_beyond_done = None

    def _set_init_pose(self):
        """
        Sets joints to initial position [0,0,0]
        :return:
        """

        self.check_publishers_connection()

        # Reset Internal pos variable
        self.init_internal_vars(self.init_pos)
        self.move_joints(self.pos)
