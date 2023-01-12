# SARL* Internship

## 0. Contents
#### 1. [Basic Parameters](https://github.com/nabihandres/sarl_dif_env/blob/master/SARL*_Internship.md#1-basic-parameters-1)
#### 2. [Reward Function](https://github.com/nabihandres/sarl_dif_env/blob/master/SARL*_Internship.md#2-reward-function-1)
#### 3. [Simulation Environments](https://github.com/nabihandres/sarl_dif_env/blob/master/SARL*_Internship.md#3-simulation-environments-1)
#### 4. [Value Network Model](https://github.com/nabihandres/sarl_dif_env/blob/master/SARL*_Internship.md#4-value-network-model-1)
#### 5. [Obstacle Detection](https://github.com/nabihandres/sarl_dif_env/blob/master/SARL*_Internship.md#5-obstacle-detection-1)
#### 6. [SARL* Parameters](https://github.com/nabihandres/sarl_dif_env/blob/master/SARL*_Internship.md#6-sarl-parameters-1)
#### 7. [Experiment](https://github.com/nabihandres/sarl_dif_env/blob/master/SARL*_Internship.md#7-experiment-1)
#### 8. [Reference](https://github.com/nabihandres/sarl_dif_env/blob/master/SARL*_Internship.md#8-reference-1)
  
  
## 1. Basic Parameters

  The basic training parameters can be found in "env.config", "policy.config", "train.config" under "~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/configs/".  
  **Note**: We will change some parameters in these config files for our training.

### 1-1. policy.config
  ```config
   # policy configurations for robot

   [rl]
   # discount factor
   gamma = 0.9
   
   
   [om]
   cell_num = 4 
   cell_size = 1
   om_channel_size = 3
   
   
   [action_space]
   # Differential kinematics
   kinematics = unicycle
   # action space size is speed_samples * rotation_samples + 1
   # intervals in Vx of Action space
   speed_samples = 5
   # intervals in Rz of Action space
   rotation_samples = 7
   # for Vx of action space : Vx = v_pref * (e^(i/5)-1)/(e-1)
   sampling = exponential
   query_env = true
   
   
   [cadrl] 
   mlp_dims = 150, 100, 100, 1
   multiagent_training = false
   
   
   [lstm_rl]
   global_state_dim = 50
   mlp1_dims = 150, 100, 100, 50
   mlp2_dims = 150, 100, 100, 1
   multiagent_training = true
   with_om = false
   with_interaction_module = true
   
  
   [srl]
   mlp1_dims = 150, 100, 100, 50
   mlp2_dims = 150, 100, 100, 1
   multiagent_training = true
   with_om = false
   
   # for deep neural network dimension
   [sarl]
   mlp1_dims = 150, 100
   mlp2_dims = 100, 50
   attention_dims = 100, 100, 1
   mlp3_dims = 150, 100, 100, 1
   multiagent_training = true
   with_om = false
   with_global_state = false
  ```
  * gamma(defalut=0.9): discount factor
  * kinematics(holonomic, unicycle (defalut=unicycle)): Kinematics of the robot 
  * speed_samples(defalut=5): The number of samples of Vx in action space
  * rotation_samples(defalut=7): The number of samples of Wz in action space
  * mlp1_dims, mlp2_dims, attention_dims, mlp3_dims: The dimensions of multi-layer perceptron (MLP) (Refer to [SARL* paper](https://ieeexplore.ieee.org/document/8961764) Value Network part in page 690)

    Action space is defined in "~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/policy/cadrl.py" as following.

   ```python
    def build_action_space(self, v_pref):
       """
       Action space consists of 42 uniformly sampled actions in permitted range and 42 randomly sampled actions.
       """
       holonomic = True if self.kinematics == 'holonomic' else False
       speeds = [0] + [(np.exp((i + 1) / self.speed_samples) - 1) / (np.e - 1) * v_pref for i in range(self.speed_samples)]
       if holonomic:
           rotations = list(np.linspace(0, np.pi * 2, num=15, endpoint=False))
       else:
           rotations = list(np.linspace(-np.pi / 4, np.pi / 4, num=self.rotation_samples))

       action_space = [ActionXY(0, 0) if holonomic else ActionRot(0, 0)]
       for rotation, speed in itertools.product(rotations, speeds):
           if holonomic:
               action_space.append(ActionXY(speed * np.cos(rotation), speed * np.sin(rotation)))
           else:
               action_space.append(ActionRot(speed, rotation))

       self.speeds = speeds
       self.rotations = rotations
       self.action_space = action_space
   ```  
   
   
### 1-2. train.config
  ```config
  [trainer]
  batch_size = 100


  [imitation_learning]
  il_episodes = 12000
  il_policy = orca
  il_epochs = 50
  il_learning_rate = 0.01
  # increase the safety space in ORCA demonstration for robot
  safety_space = 0.15


  [train]
  rl_learning_rate = 0.001
  # number of batches to train at the end of training episode
  train_batches = 100
  # training episodes in outer loop
  train_episodes = 20000
  # number of episodes sampled in one training episode
  sample_episodes = 6
  target_update_interval = 50
  evaluation_interval = 1000
  # the memory pool can roughly store 2K episodes, total size = episodes * 50
  capacity = 100000
  epsilon_start = 0.5
  epsilon_end = 0.1
  epsilon_decay = 4000
  checkpoint_interval = 1000
  ```

  * batch_size(defalut=100): The batch size of training
  * il_episodes(defalut=12000): The training episodes of imitation learning 
  * il_policy(defalut=orca): Imitation learning policy
  * il_learning_rate(default=0.01): The learning rate of imitation learning 
  * rl_learning_rate(default=0.001): The learning rate of reinforcement learning.
  * train_batches(default=100): The number of batches to train at the end of training episode
  * train_episodes(default=20000): The training episodes of reinforcement learning  
  * sample_episodes(default=6): The number of episodes sampled in one training episode


### 1-3. env.config
  ```config
  [env]
  # navigation time limit
  time_limit = 60
  # delta t
  time_step = 0.25
  val_size = 600 
  test_size = 3000
  # true if randomize human's radius and v_pref
  randomize_attributes = true
  ```

  * time_limit(defalut=25) [sec]: time limitaion for a episode; If you reduce v_pref, It is better to increase time_limit.
  * time_step(defalut=0.25) [sec]: time interval of simulation
  * val_size(default=600): the number of simulation to evaluate the model
  * test_size(default=3000): the number of simulation for final test
  * randomize_attributes: true if randomize human's radius and v_pref


  ```config
   [reward]
   # When reach the goal
   success_reward = 10
   # When robot collide to human
   collision_penalty = -20
   # Discomfortable distance to human
   discomfort_dist = 0.25
   discomfort_penalty_factor = 2.5
  ```

  * success_reward: The reward when robot reach the goal
  * collision_penalty: The reward when robot collide to a human
  * discomfort_dist(d_c) [m]: The minimum comfortable distance that humans can tolerate 
  * discomfort_penalty_factor: The factor of discomfortable distance reward (Refer to [SARL* paper](https://ieeexplore.ieee.org/document/8961764) reward function in page 689)  
 

  ```config
   [sim]
   # humans lie on circle and move to opposite side
   train_val_sim = circle_crossing
   test_sim = circle_crossing
   square_width = 12
   circle_radius = 4
   human_num = 5
  ```

  * train_val_sim: the way human agents move
  * test_sim: the way human agents move when visualize
  * circle_radius(defalut=4) [m]: The circle radius of environment; If you reduce circle_radius, you have to reduce radius of agents. Otherwise, training will not be progressed.
  * human_num(defalut=5): The number of human agents


  ```config
   # humans and robot agents.
   [humans]
   visible = true
   policy = orca
   radius = 0.3
   v_pref = 1
   sensor = coordinates
  ```  

  ```config
   [robot]
   visible = false
   # it will use the argument --policy 
   policy = none
   radius = 0.3
   v_pref = 1 #0.5
   sensor = coordinates
  ```  
## 2. Reward Function

I set the reward funtion as below.
  * Success: 10
  * Collision: -0.25
  * Discomfortable distance( $d_t < 0.2m$ ): $(d_t - 0.2) * 0.5$
  * Otherwise: $1 - d_{t+1}/d_t$

The reward function is defined in following three files.

### 2-1. env.config  
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/configs/env.config
  ```config
   [reward]
   # When reach the goal
   success_reward = 10
   # When robot collide to human
   collision_penalty = -0.25
   # Discomfortable distance to human
   discomfort_dist = 0.2
   discomfort_penalty_factor = 0.5
  ```
  
### 2-2. crowd_sim.py  
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_sim/envs/crowd_sim.py
   ```python
    def step(self, action, update=True):
        ...
	
        # check if reaching the goal
        end_position = np.array(self.robot.compute_position(action, self.time_step))
        start_dg = norm(self.robot.get_position() - np.array(self.robot.get_goal_position()))
        end_dg = norm(end_position - np.array(self.robot.get_goal_position()))
        reaching_goal = end_dg < self.robot.radius

        if self.global_time >= self.time_limit - 1:
            reward = -10
            done = True
            info = Timeout()
        elif collision:
            reward = self.collision_penalty
            done = True
            info = Collision()
        elif reaching_goal:
            reward = self.success_reward
            done = True
            info = ReachGoal()
        elif dmin < self.discomfort_dist:
            # only penalize agent for getting too close if it's visible
            # adjust the reward based on FPS
            reward = (dmin - self.discomfort_dist) * self.discomfort_penalty_factor * self.time_step
            done = False
            info = Danger(dmin)
        else:
            reward = (1 - end_dg/start_dg) * self.time_step
            done = False
            info = Nothing()
   ```

### 2-3. sarl.py  
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/policy/sarl.py
   ```python
    def predict(self, state):
	...
                    #reward = self.compute_reward(next_self_state, next_human_states) # old
                    reward = self.compute_reward_dg(state.self_state, next_self_state, next_human_states) # new
	...

    # old
    def compute_reward(self, nav, humans):
        # collision detection
        dmin = float('inf')
        collision = False
        if len(humans):
            for i, human in enumerate(humans):
                dist = np.linalg.norm((nav.px - human.px, nav.py - human.py)) - nav.radius - human.radius
                if dist < 0:
                    collision = True
                    break
                if dist < dmin:
                    dmin = dist
        # check if reaching the goal
        #######################################################
        dg = np.linalg.norm((nav.px - nav.gx, nav.py - nav.gy))
        ########################################################
        reaching_goal = dg < nav.radius
        # Equation (2)
        if collision:
            reward = self.env.collision_penalty # = -0.25
        elif reaching_goal:
            reward = 10
        elif dmin < self.env.discomfort_dist:
            #reward = (dmin - self.env.discomfort_dist) * self.env.discomfort_penalty_factor * self.env.time_step
            reward = (dmin - self.env.discomfort_dist) * self.env.discomfort_penalty_factor
        else: # give reward when the robot moves toward the goal.
            reward = 0
            #reward = 1 - 0.32*dg*dg # function 1
            #reward = 1 - 0.16*dg*dg # function 2
            #reward = 1 / (1 + np.exp(dg - 3)) #function 3
            #reward = 1 / (1 + np.exp(2.5*dg - 4)) #function 4
            #reward = 1 / (1 + np.exp(dg - 4)) #function 5

        return reward

    # new
    def compute_reward_dg(self, prev_nav, nav, humans):
        # collision detection
        dmin = float('inf')
        collision = False
        if len(humans):
            for i, human in enumerate(humans):
                dist = np.linalg.norm((nav.px - human.px, nav.py - human.py)) - nav.radius - human.radius
                if dist < 0:
                    collision = True
                    break
                if dist < dmin:
                    dmin = dist
        
        # compute dg at time t and t + 1
        start_dg = np.linalg.norm((prev_nav.px - prev_nav.gx, prev_nav.py - prev_nav.gy))
        end_dg = np.linalg.norm((nav.px - nav.gx, nav.py - nav.gy))

        # check if reaching the goal
        reaching_goal = end_dg < nav.radius

        # check if timeout
        Timeout = False
        self.global_time += self.time_step
        if self.global_time >= self.env.time_limit - 1 :
            Timeout = True

        if collision:
            reward = self.env.collision_penalty
        elif Timeout:
            reward = -10
        elif reaching_goal:
            reward = 10
        elif dmin < self.env.discomfort_dist:
            reward = (dmin - self.env.discomfort_dist) * self.env.discomfort_penalty_factor * self.env.time_step
        else: 
            reward = (1 - end_dg/start_dg) * self.env.time_step

        return reward
   ```

## 3. Simulation Environments
### 3-1. Simulation scenarios

I made new simulation scenarios.
  * no_obstacle
  * static
  * dynamic
  * mixed

The simulation environments are defined in following files.  

#### 3-1-1 env.config  
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/configs/env.config  

  ```config
  [env]
  # navigation time limit
  time_limit = 60
  # delta t
  time_step = 0.25
  val_size = 600 
  test_size = 3000
  # true if randomize human's radius and v_pref
  randomize_attributes = true
  
  ...
  
   [sim]
   # humans lie on circle and move to opposite side
   train_val_sim = circle_crossing
   test_sim = circle_crossing
   square_width = 12
   circle_radius = 4
   human_num = 10
   
   # humans and robot agents.
   [humans]
   visible = true
   policy = orca
   radius = 0.3
   v_pref = 1
   sensor = coordinates
   
   [robot]
   visible = false
   # it will use the argument --policy 
   policy = none
   radius = 0.3
   v_pref = 0.4
   sensor = coordinates
  ```
  
  * Limitation time is 60s and time step is 0.25s.
  * When 'randomize_attributes' is true, the radius and velocity of human agents are arbitrarily determined.
  * When 'randomize_attributes' is false, the radius of human agents is 0.3m and the velocity of human agents is 1m/s.
  * Moving way of human agents for training and testing is implemented in all four in order(no, static, dynamic, mixed). It has nothing to do with this file. For more details, refer to below [3-2-3](https://github.com/nabihandres/sarl_dif_env/blob/master/SARL*_Internship.md#3-2-3-reseting-simulation-scenario).
  * 'square_width' is nothing in my simulation environments.
  * The number of human agents is ten.
  * The policy of human agents is 'orca'
  * The radius of robot agent is 0.3m and the velocity of robot agent is 0.4m/s.

When 'randomize_attributes' is true, the range of velocity and radius of human agents is defined in 'agent.py'.  

~sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_sim/envs/utils/agent.py  
   ```python
   def sample_random_attributes(self):
        """
        Sample agent radius and v_pref attribute from certain distribution
        :return:
        """
        self.v_pref = np.random.uniform(0.5, 1.2)
        self.radius = np.random.uniform(0.01, 0.61)
   ```

#### 3-1-2 crowd_sim.py  
~sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_sim/envs/crowd_sim.py  

   ```python
    def generate_random_human_position2(self, human_num, rule):
        if rule == 'no':
            self.humans = []
            for i in range(human_num):
                self.humans.append(self.generate_fake_human())

        elif rule == 'dynamic':
            self.humans = []
            for i in range(human_num):
                self.humans.append(self.generate_circle_crossing_human())

        elif rule == 'static':
            self.humans = []
            total_num = 0
            group_num = random.randrange(1,5)
            for i in range(group_num-1):
                while total_num <= human_num-(group_num-(1+i)):
                    member_num = random.randrange(1,6)
                    if total_num + member_num <= human_num-(group_num-(1+i)):
                        total_num += member_num
                        break
                self.humans += self.generate_group(member_num)
            self.humans += self.generate_group(human_num-total_num)
        
        elif rule == 'mixed':
            self.humans = []
            total_num = 0
            group_num = random.randrange(1,5)
            for i in range(group_num):
                while total_num < human_num-(group_num-(1+i)):
                    member_num = random.randrange(1,6)
                    if total_num + member_num < human_num-(group_num-(1+i)):
                        total_num += member_num
                        break
                self.humans += self.generate_group(member_num)
            for i in range(human_num-total_num):
                self.humans.append(self.generate_circle_crossing_human())

        else:
            raise ValueError("Rule doesn't exist")
	    
   ```	    

The starting position, arrival position, and speed of each human agent are determined by the following functions.
  *  generate_fake_human (for no obstacle simulation environments)
  *  generate_circle_crossing_human (for dynamic and mixed simulation environments)
  *  generate_group, generate_group_member (for static and mixed simulation environments)

   ```python
    def generate_group(self, member_num):
        while True:
            group = []
            center_px = random.uniform(-4, 4)
            center_py = random.uniform(-3, 3)
            angle = np.random.random() * np.pi * 2
            for j in range(member_num):
                human = self.generate_group_member(center_px, center_py, angle, member_num, j, group)
                if human == False:
                    group_made = False
                    break
                else:
                    group.append(human)
                    if len(group) == member_num:
                        group_made = True
                        break
            if group_made:
                return group
                

    def generate_group_member(self, center_px, center_py, angle, member_num, j, group):
        human = Human(self.config, 'humans')
        if self.randomize_attributes:
            human.random_radius()
        human.v_pref = 0
        j_angle = angle + np.pi * 2 * j / member_num                
        px = (0.2+member_num*0.1) * np.cos(j_angle) + center_px
        py = (0.2+member_num*0.1) * np.sin(j_angle) + center_py         
        collide = False
        for agent in [self.robot] + self.humans + group:
            min_dist = human.radius + agent.radius
            if norm((px - agent.px, py - agent.py)) < min_dist or \
                norm((px - agent.gx, py - agent.gy)) < min_dist:
                collide = True
                break
        if collide:
            return False
        else:
            human.set(px, py, px, py, 0, 0, 0)
            return human
	
	...
    
    def generate_fake_human(self):
        human = Human(self.config, 'humans')
        human.v_pref = 0
        human.radius = 0.3
        while True:
            angle = np.random.random() * np.pi * 2
            # add some noise to simulate all the possible cases robot could meet with human
            px = (20) * np.cos(angle)
            py = (20) * np.sin(angle)
            collide = False
            for agent in [self.robot] + self.humans:
                min_dist = human.radius + agent.radius + self.discomfort_dist
                if norm((px - agent.px, py - agent.py)) < min_dist or \
                        norm((px - agent.gx, py - agent.gy)) < min_dist:
                    collide = True
                    break    # jump out of 'for' loop
            if not collide:
                break        # jump out of 'while' loop
        human.set(px, py, px, py, 0, 0, 0)
        return human

	...

    def generate_circle_crossing_human(self):
        human = Human(self.config, 'humans')
        if self.randomize_attributes:
            human.sample_random_attributes()
        while True:
            angle = np.random.random() * np.pi * 2
            # add some noise to simulate all the possible cases robot could meet with human
            px_noise = (np.random.random() - 0.5) * human.v_pref
            py_noise = (np.random.random() - 0.5) * human.v_pref
            px = (self.circle_radius * human.v_pref / self.robot.v_pref) * np.cos(angle) + px_noise
            py = (self.circle_radius * human.v_pref / self.robot.v_pref) * np.sin(angle) + py_noise
            #px = (self.circle_radius) * np.cos(angle) + px_noise
            #py = (self.circle_radius) * np.sin(angle) + py_noise
            collide = False
            for agent in [self.robot] + self.humans:
                min_dist = human.radius + agent.radius + self.discomfort_dist
                if norm((px - agent.px, py - agent.py)) < min_dist or \
                        norm((px - agent.gx, py - agent.gy)) < min_dist:
                    collide = True
                    break    # jump out of 'for' loop
            if not collide:
                break        # jump out of 'while' loop
        human.set(px, py, -px, -py, 0, 0, 0)
        return human
    
   ```

#### 3-1-3 result 
  * no

  * static

  * dynamic

  * mixed
  

### 3-2. Simulation Execution Principle  
The simulation execution principle is as follows.  
1. Run train or test.  
2. Run the function 'run_k_episodes'.  
3. Reset simulation scenario.  
4. Simulation starts.    

![Train](https://user-images.githubusercontent.com/83470394/189515742-74367a6b-29a7-496d-b95f-2691a3d59f1d.png)  

#### 3-2-1. Running Train or Test.  
There are two ways to run the simulation.  
One is to train, and the other one is to test.  
The contents of training is defined in 'train.py' and the contents of testing is defined in 'test.py' under  
"~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/".  
See [train](https://github.com/nabihandres/sarl_dif_env/blob/master/SARL*_Internship.md#3-3-1-training) and [test](https://github.com/nabihandres/sarl_dif_env/blob/master/SARL*_Internship.md#3-3-2-testing) for details.  

  * Train  
   	To train and make value network model, enter the following command.  
   	 ```
    	 cd ~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav
    	 python train.py --policy sarl
   	 ```

  * Test  
  	To test, enter the following command.  
   	 ```
    	 cd ~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav
    	 python test.py --policy sarl --model_dir data/output_new4_0.4_f1 --phase test
   	 ```
   	To visualize one episode, enter the following command.  
   	 ```
   	     cd ~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav
    	 python test.py --policy sarl --model_dir data/output_new4_0.4_f1 --phase test --visualize --test_case 0
   	 ```


#### 3-2-2. Function 'run_k_episodes'  
In both 'train.py' and 'test.py', simulations is run through the function 'run_k_episodes'.  
For example,  

~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/train.py  
   ```python
   	...
   	explorer.run_k_episodes(sample_episodes, 'train', update_memory=True, episode=episode)
	...
   ```  

~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/test.py  
   ```python
   	...
	explorer.run_k_episodes(env.case_size[args.phase], args.phase, print_failure=True)
	...
   ```  

Function 'run_k_episodes' is defined in 'explorer.py'  

~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/utils/explorer.py  
   ```python
    def run_k_episodes(self, k, phase, update_memory=False, imitation_learning=False, episode=None,
                       print_failure=False):
	
	...
	
        for i in range(k):
            ob = self.env.reset(i,phase)
            logging.info("running %s/%s episode" %(i+1,k)+ ", simulation environment: " + str(self.env.test_sim))
	
	...
	
   ```
In the code above, 'self.env' is the simulation environment built on 'crowd_sim.py'.  
And as many episodes as k are executed by for loop.   
And using the function 'reset' in 'crowd_sim.py', simulation scenario is reset on a per episode.  
**Note**: Parameter 'i' is entered in 'reset' function.  

#### 3-2-3. Reseting Simulation Scenario
Function 'reset' is defined in 'crowd_sim.py'.  

~sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_sim/envs/crowd_sim.py  

I made simulation scenario be decided according to parameter 'number' of 'reset' function.  

   ```python
    def reset(self, number, phase='test', test_case=None):
        """
        Set px, py, gx, gy, vx, vy, theta for robot and humans
        :return:
        """
        if self.robot is None:
            raise AttributeError('robot has to be set!')
        assert phase in ['train', 'val', 'test']
        if test_case is not None:
            self.case_counter[phase] = test_case
        self.global_time = 0
        if phase == 'test':
            self.human_times = [0] * self.human_num
        else:
            self.human_times = [0] * (self.human_num if self.robot.policy.multiagent_training else 1)
        if not self.robot.policy.multiagent_training:
            self.train_val_sim = 'static'
	
	...
	
        if number %4 == 0:
            self.train_val_sim = 'no'
            self.test_sim = 'no'
        elif number %4 == 1:
            self.train_val_sim = 'static'
            self.test_sim = 'static'
        elif number %4 == 2:
            self.train_val_sim = 'dynamic'
            self.test_sim = 'dynamic'
        elif number %4 == 3:
            self.train_val_sim = 'mixed'
            self.test_sim = 'mixed'
	
	...

        return ob
   
   ```
**Note**: So, when training is run by 'run_k_episodes' function, four scenarios are executed in order.  


### 3-3. Numbers for Simulation  
#### 3-3-1. Training  
The contents of training is defined in 'train.py'.  

~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/train.py  

Five kinds of simulations are executed during the training.  
   * Imitation Learning  
   * Filling the memory pool with some RL experience  
   * Evaluating the model at every interval  
   * Sampling episodes into memory and optimizing over the generated memory  
   * Final testing  

**1. Imitation Learning**   
When we start new training or overwrite existing model, Imitation Learning starts for the first with orca policy.  

~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/train.py  

Start new training by putting new path of value network model into 'args.output_dir'.  
   ```python
    def main():
    	parser = argparse.ArgumentParser('Parse configuration file')
	...
    	parser.add_argument('--output_dir', type=str, default='data/output_new4_0.4_f1')
	...
    	args = parser.parse_args()
   
   ```  
or   

overwrite existing model by choosing 'y'.  
   ```python
    if os.path.exists(args.output_dir):
        key = raw_input('Output directory already exists! Overwrite the folder? (y/n)')   # python2
   ```   
   
The number of episodes for imitation learning is defined in 'train.config'.   
And I set the number of episodes for imitation learning 12000.   

~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/train.py  
   ```python
    else:
        il_episodes = train_config.getint('imitation_learning', 'il_episodes')
	...
        explorer.run_k_episodes(il_episodes, 'train', update_memory=True, imitation_learning=True)
	...
   ```   
   
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/configs/train.config   
   ```config
    [imitation_learning]
    il_episodes = 12000
    ...
   ```

**2. Filling the memory pool with some RL experience**   
When you resume training existing model, Imitation learing is omitted and start from filling the memory pool with some RL experience.  

~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/train.py  

Put the path which you want ot resume training into 'args.output_dir'. And put 'True' into 'args.resume'.  
   ```python
    def main():
    	parser = argparse.ArgumentParser('Parse configuration file')
	...
	parser.add_argument('--output_dir', type=str, default='data/output_new4_0.4_f1')
	...
    	parser.add_argument('--resume', default=True, action='store_true')
	...
    	args = parser.parse_args()
   ```   
And choose 'n'.  
   ```python
    if os.path.exists(args.output_dir):
        key = raw_input('Output directory already exists! Overwrite the folder? (y/n)')   # python2
   ```   

I set the number of episodes for filling the memory pool with some RL experience 400.  
   ```python
    # fill the memory pool with some RL experience
    if args.resume:
        robot.policy.set_epsilon(epsilon_end)
        explorer.run_k_episodes(400, 'train', update_memory=True, episode=0)
        logging.info('Experience set size: %d/%d', len(memory), memory.capacity)
   ```   

**3. Evaluating the model at every interval**  
Tests are performed to evaluate the model at specific intervals.  

~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/train.py  
   ```python
    # evaluate the model
    if episode % evaluation_interval == 0:
        explorer.run_k_episodes(env.case_size['val'], 'val', episode=episode)
   ```
The 'evaluation_interval' is defined in 'train.config' and I set it to 1000.  
  
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/configs/train.config   
   ```config
    [train]
    ...
    evaluation_interval = 1000
    ...
   ```  
   
And the number of test episodes at intervals is defined in 'env.config' and I set it to 400.  

~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/configs/env.config  
   ```config
    [env]
    ...
    val_size = 400 
    ...
   ```

**4. Sampling episodes into memory and optimizing over the generated memory**  
Per training episode includes several sample episodes to optimize over the generated memory.  

~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/train.py  
   ```python
    while episode < train_episodes:
	...
        # sample k episodes into memory and optimize over the generated memory
        explorer.run_k_episodes(sample_episodes, 'train', update_memory=True, episode=episode)
        trainer.optimize_batch(train_batches)
        episode += 1
	...
   ```  
   
The numbers of trainings and sample episodes are defined in 'train.config' as 'training_episodes' and 'sample_episodes' respectively.  
I set 'training_episodes' to 20000 and 'sample_episodes' to 4.  
So four scenarios are executed 20,000 times in turn during training.  

~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/configs/train.config  
   ```python
    [train]
    ...  
    train_episodes = 20000
    # number of episodes sampled in one training episode
    sample_episodes = 4
    ...
   ```

**5. Final testing**   
At the end of the training, the last test is run to evaluate the model.   

~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/train.py   
   ```python
    ...
 
    # final test
    explorer.run_k_episodes(env.case_size['test'], 'test', episode=episode)
 
   ```  

The number of final testing episodes is defined in 'env.config' and I set it to 2000.  

~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/configs/env.config  
   ```config
    [env]
    ...
    test_size = 2000
    ...
   ```

#### 3-3-2. Testing  
The contents of testing is defined in 'test.py'.  
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/test.py  

**1. Visualizing**  

**2. Testing multiple episodes**  




## 4. Value Network Model  
Using contents 1, 2, 3, I made Value Network models.  
Models' folders are under  
"~/sarl_ws/src/sarl_dif_env/sarl_star_ros/CrowdNav/crowd_nav/data/".  
The difference between these models is number of human agents.  

### 4-1. output_new4_0.4_f1
  * number of human agents: 10

### 4-2. output_new5_0.4_f1
  * number of human agents: 5

### 4-3. output_new6_0.4_f1
  * number of human agents: 15

## 5. Obstacle Detection  

### 5-1. Information of Obstacles demanded for SARL*
   The information used in sarl_star_node is  
   * Obstacles' position x
   * Obstacles' position y
   * Obstacles' velocity x
   * Obstacles' velocity y
   * Obstacels' radius

### 5-2. Using 2D Lidar - [obstacle_detector](https://github.com/tysik/obstacle_detector)
#### 5-2-1. Setting [obstacle_detector](https://github.com/tysik/obstacle_detector)  

I set the fixed frame and target frame in the launch file to connect obstacle_detector package with sarl_star_ros package.  
Fixed frame id is 'map/' and target frame id is 'base_scan/'.  
~/sarl_ws/src/sarl_dif_env/obstacle_detector/nodes.launch
   ```launch
   <!-- Reusable launch file for obstacle detection -->
   <launch>
   
     <node name="scans_merger" pkg="obstacle_detector" type="scans_merger_node">
       <param name="active"            value="true"/>
       <param name="publish_scan"      value="true"/>
       <param name="publish_pcl"       value="false"/>

       <param name="ranges_num"        value="1000"/>

       <param name="min_scanner_range" value="0.05"/>
       <param name="max_scanner_range" value="10.0"/>

       <param name="min_x_range"       value="-10.0"/>
       <param name="max_x_range"       value="10.0"/>
       <param name="min_y_range"       value="-10.0"/>
       <param name="max_y_range"       value="10.0"/>
   
       <param name="fixed_frame_id"   value="map"/>
       <param name="target_frame_id"   value="base_scan"/>
     </node>

     <node name="obstacle_extractor" pkg="obstacle_detector" type="obstacle_extractor_node">
       <param name="active"               value="true"/>
       <param name="use_scan"             value="true"/>
       <param name="use_pcl"              value="fasle"/>

       <param name="use_split_and_merge"    value="true"/>
       <param name="circles_from_visibles"  value="true"/>
       <param name="discard_converted_segments" value="true"/>
       <param name="transform_coordinates"  value="true"/>

       <param name="min_group_points"     value="5"/>

       <param name="max_group_distance"   value="0.1"/>
       <param name="distance_proportion"  value="0.00628"/>
       <param name="max_split_distance"   value="0.2"/>
       <param name="max_merge_separation" value="0.2"/>
       <param name="max_merge_spread"     value="0.2"/>
       <param name="max_circle_radius"    value="0.6"/>
       <param name="radius_enlargement"   value="0.3"/>

       <param name="frame_id"             value="map"/>
     </node>

     <node name="obstacle_tracker" pkg="obstacle_detector" type="obstacle_tracker_node">
       <param name="active"                  value="true"/>

       <param name="loop_rate"               value="100.0"/>
       <param name="tracking_duration"       value="2.0"/>
       <param name="min_correspondence_cost" value="0.6"/>
       <param name="std_correspondence_dev"  value="0.15"/>
       <param name="process_variance"        value="0.1"/>  
       <param name="process_rate_variance"   value="0.1"/>  
       <param name="measurement_variance"    value="1.0"/> 

       <param name="frame_id"                value="map"/>

       <remap from="tracked_obstacles" to="obstacles"/>
     </node>
 
     <!--<node name="obstacle_publisher" pkg="obstacle_detector" type="obstacle_publisher_node">
       <param name="active"           value="true"/>
       <param name="reset"            value="false"/>

       <param name="fusion_example"   value="false"/>
       <param name="fission_example"  value="false"/>

       <param name="loop_rate"        value="10.0"/>
       <param name="radius_margin"    value="0.25"/>

       <rosparam param="x_vector">[-3.0, -2.5, -2.5, -1.0, -1.0, -0.5, 2.5, 0.2, 2.0, 4.5, 4.0, 1.5]</rosparam>
       <rosparam param="y_vector">[1.5, 0.0, -2.5, 3.0, 1.0, -4.0, -3.0, -0.9, 0.0, 0.0, 2.0, 2.0]</rosparam>
       <rosparam param="r_vector">[0.5, 0.5, 1.5, 0.5, 0.7, 0.5, 1.5, 0.7, 0.7, 1.0, 0.5, 1.0]</rosparam>
       <rosparam param="vx_vector">[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]</rosparam>
       <rosparam param="vy_vector">[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]</rosparam>

       <param name="frame_id"         value="map"/>
     </node>-->

   </launch>
   ```
   
And I added the command for launching obstacle_detector in sarl* launch files.
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/launch/sarl_star_navigation.launch
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/launch/sarl_star_turtlebot3_sarl_world.launch

   ```launch
   <launch>

     <!-- Define laser type-->
     <arg name="laser_type" default="hokuyo" />
   ...

     <!-- Obstacle Detector-->
     <include file="$(find obstacle_detector)/launch/nodes.launch" />

   ... 
   </launch>
   ```
  
#### 5-2-2. Topic and Message
In obstacle_detector package, 'obstacle_tracker' node publishes information of obstacles.  
The topic which obstacle_tracker node publishes is '/obstacles'.  
The message which obstacle_tracker node publishes with /obstacles topic is 'Obstacles'.  

![Screenshot from 2022-07-27 17-26-08](https://user-images.githubusercontent.com/83470394/181199546-f72c8359-49d9-4aa9-8f2a-1d095391a108.png)

  * 'Obstacles' msg  
    ~/sarl_ws/src/sarl_dif_env/obstacle_detector/msg/Obstacles.msg  
    ![Screenshot from 2022-07-27 17-34-23](https://user-images.githubusercontent.com/83470394/181201387-84b3e182-ac2e-44a8-a73d-63664d05f59f.png)
    
    We use only 'circles' in Obstacles.msg
    
  * 'CircleObstacle' msg  
    ~/sarl_ws/src/sarl_dif_env/obstacle_detector/msg/CircleObstacle.msg  
    ![Screenshot from 2022-07-27 17-34-46](https://user-images.githubusercontent.com/83470394/181202186-bbf6889e-f9b1-4cbb-8b70-6b0c0cbfe36e.png)

    - 'center' is the position of obstacles.
    - 'velocity' is the velocity of obstacles.
    - 'true_radius' is the velocity of obstacles.

#### 5-2-3. Subscribing Information of Obstacles  
I made 'sarl_star_node' subscribe the information of obstacles which obstacle_tracker publishes.
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/scripts/sarl_star_node.py
   ```python
        # ROS subscribers
	...
        self.objects_sub = rospy.Subscriber('/obstacles', Obstacles, self.update_objects)
	...
   ```
   ```python
   def update_objects(self, msg):
        # observable state: px,py,vx,vy,radius
        self.IsObReceived = True
        self.objects = list()
        self.ob = list()
        for p in msg.circles:
            # dist = np.linalg.norm(np.array([self.px,self.py])-np.array([p.position.x,p.position.y]))
            human = Human(p.center.x, p.center.y, p.velocity.x, p.velocity.y, p.true_radius)
            self.objects.append(human)
        for human in self.objects:
            self.ob.append(human.get_observable_state())
   ```


#### 5-2-4. Result  
  * Gazebo and Rviz  
    ![Screenshot from 2022-07-27 17-11-14](https://user-images.githubusercontent.com/83470394/181196382-3ffb32cd-d81c-46e3-ab34-00438288e0dc.png)

  * rqt_graph Full Screen  
    ![Screenshot from 2022-07-27 17-07-30](https://user-images.githubusercontent.com/83470394/181195933-089f359c-0f1f-4007-8aeb-d16ea1a972fe.png)  
 
  
  * rqt_graph Zoom in  
    ![Screenshot from 2022-07-27 17-08-51](https://user-images.githubusercontent.com/83470394/181196006-16b85ff2-83cb-4414-8bd2-92a2e55ba341.png)


### 5-3. Using Depth Camera - [YOLACT](https://github.com/nabihandres/YOLACT_copy)

#### 5-3-1. Installation of [YOLACT pacakge](https://github.com/nabihandres/YOLACT_copy)  
Refer to [here](https://github.com/nabihandres/YOLACT_copy/blob/main/Yolact_ObjectTracking.md).

#### 5-3-2. Publishing Information of Humans  

**1. Messages**  
I made two msg files.
  * obj_tracking.msg  
    ~/sarl_ws/sarl_dif_env/YOLACT_copy/yolact_ROS/msg/obj_tracking.msg  
    ![Screenshot from 2022-07-27 22-51-01](https://user-images.githubusercontent.com/83470394/181264171-c27aa8cd-a72e-43eb-92af-3b12148d1431.png)  
    - 'object_id': id for distinguishing between humans  
    - 'pos': position of a human  
    - 'vel': velocity of a human  
    
  * objects.msg  
    ~/sarl_ws/sarl_dif_env/YOLACT_copy/yolact_ROS/msg/objects.msg  
    ![Screenshot from 2022-07-27 22-51-16](https://user-images.githubusercontent.com/83470394/181264278-862a22b8-6497-4a81-9952-582f91c62074.png)  
    - objects: collection of humans  

**2. Publisher and Topic**  
The information about humans is defined in 'object_track_ROS.py'.  
~/sarl_ws/src/sarl_dif_env/YOLACT_copy/mot/src/object_track_ROS.py  

  * Importing msgs
      ```python
      from yolact_ROS.msg import obj_tracking
      from yolact_ROS.msg import objects
      ```  
   
  * Defining Publisher
      ```python
      if __name__ == '__main__':
   	   rospy.init_node('object_tracking', anonymous=True)
   	   pub = rospy.Publisher('objects', objects, queue_size=10)
	   # rospy.Subscriber("yolact_ros", Segment_centers, forward_view_tracking)
	   rospy.Subscriber("yolact_ros", Segment_centers, bird_eyed_view_tracking)
	   rospy.spin()
      ```
   
  * Publishing Information of Humans  
    ~/sarl_ws/src/sarl_dif_env/YOLACT_copy/mot/src/tracker.py
      ```python
      class Tracks(object):
	"""docstring for Tracks"""
	def __init__(self, detection, trackId, state):
		...
		...
		self.is_updated = True
      ```  
      
    ~/sarl_ws/src/sarl_dif_env/YOLACT_copy/mot/src/object_track_ROS.py
      ```python
	def bird_eyed_view_tracking(msg):
      	
        	...
      
		Objects = objects()
	
			...
		for j in range(len(tracker.tracks)):
			if (len(tracker.tracks[j].trace) > 1):
				...
				
				state_current = tracker.tracks[j].trace_state[-1]
				position_current = (state_current[0,0],state_current[0,1],state_current[0,2],state_current[0,3])

				state_before = tracker.tracks[j].trace_state[-2]
				position_before = (state_before[0,0],state_before[0,1],state_before[0,2],state_before[0,3])

				if position_current[0] == position_before[0]:
					tracker.tracks[j].is_updated = False
				else:
					tracker.tracks[j].is_updated = True
				
				velocity = vel(position_current,position_before)

				if tracker.tracks[j].is_updated:
					msg = obj_tracking()
					msg.object_id = tracker.tracks[j].trackId
					msg.pos.x = position_current[2]
					msg.pos.y = -position_current[0]
					msg.pos.z = position_current[1]
					msg.vel.x = velocity[2]
					msg.vel.y = -velocity[0]
					msg.vel.z = velocity[1]
					Objects.objects.append(msg)
				...
			
		pub.publish(Objects)
		...
      ```  

    **Note**: 'tracker.tracks[j].is_updated' is to erase the information of the missing humans from the camera's perspective and publish only the information of the remaining humans.


#### 5-3-3. Subscribing Information of Humans  
I made the subscriber of the information which 'YOLACT_copy' package publishes.  
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/scripts/sarl_star_node.py
   ```python
        # ROS subscribers
	...
        self.people_sub = rospy.Subscriber('objects', objects, self.update_humans)
	...
   ```
   ```python
    def update_humans(self, msg):
        # observable state: px,py,vx,vy,radius
        self.IsObReceived = True
        self.humans = list()
        self.ob2 = list()
        listener_ob.waitForTransform('/map', '/base_scan', rospy.Time(0), rospy.Duration(10))
        tran, rot = listener_ob.lookupTransform('/map', '/base_scan', rospy.Time(0))
        for object in msg.objects:
            tf_msg = PoseStamped()
            tf_msg.header = msg.header
            tf_msg.header.frame_id = '/base_scan'
            tf_msg.pose.position.x = object.pos.x
            tf_msg.pose.position.y = object.pos.y
            tf_msg.pose.position.z = object.pos.z
            tf_msg.pose.orientation.x = 0
            tf_msg.pose.orientation.y = 0  
            tf_msg.pose.orientation.z = 0
            tf_msg.pose.orientation.w = 1

            q1 = rot
            q2 = list()
            q2.append(object.vel.x)
            q2.append(object.vel.y)
            q2.append(object.vel.z)
            q2.append(0.0)
            output_vel = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_multiply(q1, q2),
                tf.transformations.quaternion_conjugate(q1)
            )[:3]

            tf_pos = listener_ob.transformPose('/map', tf_msg)
            output_pos = [tf_pos.pose.position.x, tf_pos.pose.position.y, tf_pos.pose.position.z]

            human = Human(output_pos[0], output_pos[1], output_vel[0], output_vel[1], HUMAN_RADIUS)
            self.visualize_human(object.object_id, output_pos)
            self.humans.append(human)

        for human in self.humans:
            self.ob2.append(human.get_observable_state())   
   ```  
**Note**: The positions of the humans in the camera coordinate system should be changed to the positions in the map coordinate system.

#### 5-3-4. Visualization Marker  
'sarl_star_node' publishes Markers for visualizing humans.  
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/scripts/sarl_star_node.py  
   ```python
        # ROS publishers
	...
        self.markers_pub_ = rospy.Publisher('/human_marker', Marker, queue_size=10)   
   ```

   ```python
    def update_humans(self, msg):
	...
	
            self.visualize_human(object.object_id, output_pos)
	...
   ```  
   ```python
    def visualize_human(self,id,position_current):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = '/map'
        name = "PEOPLE" + str(id)
        marker.ns = name
        marker.id = id
        marker.type = marker.SPHERE
        marker.pose.position.x = position_current[0]
        marker.pose.position.y = position_current[1]
        marker.pose.position.z = position_current[2]
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1
        marker.color.g = 1
        marker.lifetime = rospy.Duration(1)
        self.markers_pub_.publish(marker)   
   ```

#### 5-3-5. Result


### 5-4. Information of All Obstacles
'self.ob' is the collection of the information of obstacles detected by 'obstacle_detector'.  
'self.ob2' is the collection of the information of obstacles detected by 'YOLACT_copy'.  
They have to be combined.  
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/scripts/sarl_star_node.py  

   ```python
    def planner(self):
	...
	
        else:
            """
            self state: FullState(px, py, vx, vy, radius, gx, gy, v_pref, theta)
            ob:[ObservableState(px1, py1, vx1, vy1, radius1),
                ObservableState(px1, py1, vx1, vy1, radius1),
                   .......                    
                ObservableState(pxn, pyn, vxn, vyn, radiusn)]
            """
            #self.obs = self.ob   # using only obstacle_detector
            self.obs = self.ob+self.ob2   # using both obstacle_detector and Yolact
	    
            if len(self.obs)==0:
                self.obs = [ObservableState(self.px+FAKE_HUMAN_PX, self.py+FAKE_HUMAN_PY, 0, 0, HUMAN_RADIUS)]
   ```

## 6. SARL* Parameters

### 6-1. Package Parameters
Before testing, we have to set some parameters in sarl_star_node.py  
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/scripts/sarl_star_node.py

   ```python
    HUMAN_RADIUS = 0.3
    ROBOT_RADIUS = 0.3
    ROBOT_V_PREF = 0.22 #Turtlebot3 burger
    DISCOMFORT_DIST = 0.2
    TIME_LIMIT = 100
    GOAL_TOLERANCE = 0.6
    FAKE_HUMAN_PX = -1.7
    FAKE_HUMAN_PY = 14.3
   ```
* HUMAN_RADIUS: human radius for the people whom YOLACT_copy package detects.
* ROBOT_RADIUS: our robot radius
* ROBOT_V_PREF: the maximum velocity of our robot
* DISCOMFORT_DIST: the distance between robot and a person which people feel discomfortable
* TIME_LIMIT: limitation time for robot to reach the goal
* GOAL_TOLERANCE: the tolerance for the robot to the goal
* FAKE_HUMAN_PX, FAKE_HUMAN_PY: fake human's position  
  ```python
      def planner(self):
      ...
      
        else:
            """
            self state: FullState(px, py, vx, vy, radius, gx, gy, v_pref, theta)
            ob:[ObservableState(px1, py1, vx1, vy1, radius1),
                ObservableState(px1, py1, vx1, vy1, radius1),
                   .......                    
                ObservableState(pxn, pyn, vxn, vyn, radiusn)]
            """
            #self.obs = self.ob   # using only obstacle_detector
            self.obs = self.ob+self.ob2   # using both obstacle_detector and Yolact
	    
            if len(self.obs)==0:
                self.obs = [ObservableState(self.px+FAKE_HUMAN_PX, self.py+FAKE_HUMAN_PY, 0, 0, HUMAN_RADIUS)]

	...
  ```
  **NOTE**: The reason why we have to set a fake human is that there are fake humans during the training in no obstalce scenario and to make environments same to training environments when there is no obstacle in the real or gazebo situation.
  
### 6-2. Using Value Network Model
We have to set the path of Value Network Model which we use before doing the test.  
'model_dir' is the path of Value Network Model in 'sarl_star_node.py'.  
~/sarl_ws/src/sarl_dif_env/sarl_star_ros/scripts/sarl_star_node.py  

   ```python
   if __name__ == '__main__':
    begin_travel = False
    # set file dirs
    pack_path = rospkg.RosPack().get_path('sarl_star_ros')
    model_dir = pack_path + '/CrowdNav/crowd_nav/data/output_new5_0.4_f1/'
    env_config_file = model_dir + 'env.config'
    policy_config_file = model_dir + 'policy.config'
    if os.path.exists(os.path.join(model_dir, 'resumed_rl_model.pth')):
        model_weights = os.path.join(model_dir, 'resumed_rl_model.pth')
    else:
        model_weights = os.path.join(model_dir, 'rl_model.pth')
   ```

### 6-3. Dynamic Local Goal
SARL* is generating the dynamic local goal.  
The dynamic local goal is defined in 'goal_functions.cpp'.  
'dist_threshold' is the dynamic local goal distance.  
~/sarl_ws/src/sarl_dif_env/navigation/base_local_planner/src/goal_function.cpp  
   ```cpp
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                       costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
   ```
   
After changing dist_threshold, we have to build 'sarl_ws' again.  
   ```
   cd sarl_ws
   catkin_make
   ```
   
  * (1)
   ```cpp
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                       costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);  
   ```
   ![Screenshot from 2022-07-27 22-12-14](https://user-images.githubusercontent.com/83470394/181256117-6c5845f3-4108-4765-adb2-c35aeb4e68fb.png)
   
  * (2)
   ```cpp
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 4.0,
                                       costmap.getSizeInCellsY() * costmap.getResolution() / 4.0);     
   ```
   ![Screenshot from 2022-07-27 22-14-23](https://user-images.githubusercontent.com/83470394/181256179-e0745491-742a-472b-aa06-9b6396d88c74.png)

**Note**: We have to find the optimal dynamic local goal distance by testing. If the velocity of the robot is big, it is fine to have a short distance, but if the velocity is small, the distance should be long because robot has to detour greatly to avoid obstacles. Otherwise, the robot is short-sighted and it can not avoid obstacles.  
![image01](https://user-images.githubusercontent.com/83470394/181258936-929f33bf-08d4-43dc-a5cf-54b53cb3137c.png)



## 7. Experiment  
I did the experiment using turtlebot3 with 'output_new1_0.4_f1' model.  
The results are as shown in the video below.  
It is expected to navigate better using other models(output_new4~6_0.4_f1) and the robot whose velocity is faster.  

### 7-1. No Obstacle  
https://user-images.githubusercontent.com/83470394/182324974-87d10960-6483-4d67-90e3-7927322cf409.mp4  

### 7-2. Static Obstacle  
https://user-images.githubusercontent.com/83470394/182325035-ffe8687e-cb10-47db-850c-07dad21df36c.mp4  

### 7-3. Dynamic Obstacle   
https://user-images.githubusercontent.com/83470394/182325098-c0de134f-7644-4980-83a4-79798e2e830a.mp4 

## 8. Reference

SARL* : https://github.com/LeeKeyu/sarl_star

CrowdNav : https://github.com/vita-epfl/CrowdNav

Obstacle detector : https://github.com/tysik/obstacle_detector

