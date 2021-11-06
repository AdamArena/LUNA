
class MainBrain:
    def drive_off_lander(self): # this function is called to drive the rover off the lander before activating search function
        # set drive velocities
        self.status.update_status('s')
        self.movement = [-0.15, 0]
        self.set_velocities()
        
        time.sleep(2.5) # sleep until enough time has elapsed
        
        self.movement = [0, 0] # stop moving
        self.set_velocities()
        
        time.sleep(1) # sleep a moment longer

        
        
    def load_objects(self):
        self.objects = {'s': [], 'l': [], 'o': [], 'r': []} # global objects dictionary. Ensure it is reset each iteration

        object_codes = ['s', 'l', 'o', 'r'] # identifier codes for each type of object s = sample, l = lander, o = obstacle, r = rock
        objects_local = self.vision.nav_main(True) # get object data and save into a local variable for formatting

        # loop over objects_local variable. each index corresponds to the identifier in the same index in object_codes
        for i in range(4):
            sub_list = objects_local[i] # sub_list corresponds to each sub-list of objects_local

            # ensure sub_list is not None
            if sub_list != None and sub_list != [] and sub_list != [0, 0]:
                if not isinstance(sub_list[0], list): # some sub lists contain none of their own sub lists
                    self.objects[object_codes[i]] += [sub_list + [object_codes[i]]] # append each object individually. Add an identifier to each
                    
                    if (object_codes[i] in ['s', 'r'] and self.goal == 's') or self.goal == object_codes[i]: # if object is a goal, update goal_tracker
                        if self.goal_tracker == None:
                            self.goal_tracker = sub_list + [object_codes[i]] # update goal_tracker to the current object
                            self.tracker_TO = 0
                        else:
                            if sub_list[0] <= self.goal_tracker[0]:
                                self.goal_tracker = sub_list + [object_codes[i]]
                                self.tracker_TO = 0
                                
                            
                
                else: # sub list contains futher sub lists
                    # loop over each object contained within sub_list
                    for obj in sub_list:
                        if obj != None and obj != [0, 0]:
                            self.objects[object_codes[i]] += [obj + [object_codes[i]]] # append each object individually. Add an identifier to each
                            
                            if (object_codes[i] in ['s', 'r'] and self.goal == 's') or self.goal == object_codes[i]: # set goal_tracker as above
                                if self.goal_tracker == None:
                                    self.goal_tracker = obj + [object_codes[i]] # update goal_tracker to the current object
                                    self.tracker_TO = 0
                                else:
                                    if obj[0] <= self.goal_tracker[0]:
                                        self.goal_tracker = obj + [object_codes[i]]
                                        self.tracker_TO = 0
        


    def set_velocities(self):
        self.mobility.set_velocities(*self.movement) # update movement data



    def deposit_sample(self):
        self.movement = [0, 0]
        self.set_velocities()
        
        try:
            self.collector.OpenCollector(True)
            self.has_sample = False
            time.sleep(0.5)
            return True
        except OSError:
            return False
    
    
    
    def collect_sample(self, inch_forward):
        self.status.update_status('c')
        
        if inch_forward:
            self.movement = [0.07, 0]
            self.set_velocities()
            time.sleep(2)
        
        self.movement = [0, 0]
        self.set_velocities()
        self.collector.CloseCollector(False)
        self.has_sample = True
    
    
    
    def flip_rock(self, half_flip):
        self.collector.FlipRock(half_flip)



    def min_max(self, vector, minmax): # if minax == 1 return max else return min
        val = (max(vector) if minmax else min(vector))
        half_n = int(round(len(vector)/2))

        for i in range(len(vector)): # search from the middle index and return the first min or max found
            val_left = vector[half_n - i - 1]
            val_right = vector[half_n + i]
            
            if val_right == val:
                return val, half_n + i
            elif val_left == val:
                return val, half_n - i - 1



    def read_graph(self):
        proximity_limit = 0.3 # if goal was within this range limit when last seen, maintain heading momentarily before stopping

        vector = self.gravity_graph
        half_n = round(self.n/2)

        min_val, min_index = self.min_max(vector, 0)
        max_val, max_index = self.min_max(vector, 1)
        
        error_g = min_index - half_n
        error_o = (-max_index if max_index < half_n else 2*half_n - max_index)
        
        if min_val > -0.05: # no significant goals are visible. <-------------- SEARCH FUNCTION <--------------
            if min_val > 0.2: # significant obstacles search function
                return 0, 0.15
                
            else: # insignificant obstacles search function
                vec1 = self.gravity_graph[0 : 17]
                vec2 = self.gravity_graph[17 : 33]
                vec3 = self.gravity_graph[33 : 50]
                
                min_val_vec1 = self.min_max(vec1, 0)
                min_val_vec2 = self.min_max(vec2, 0)
                min_val_vec3 = self.min_max(vec3, 0)
                
                h1, x1 = min_val_vec1 # hypotenuse and x values
                h2, x2 = min_val_vec2
                h3, x3 = min_val_vec3
                
                x1 -= half_n
                x2 -= 16/2
                x3 = self.n - 17 + x3
                
                y1 = math.sqrt(abs(h1**2 - x1**2))
                y2 = math.sqrt(abs(h2**2 - x2**2))
                y3 = math.sqrt(abs(h3**2 - x3**2))
                
                x_t = x1 + x2 + x3
                y_t = y1 + y2 + y3
                
                return 0, 0.15
            
        elif max_val < -0.05:
            percent = error_g/half_n
            vel = 0.2*(1 - abs(percent))
            rot = 0.5*percent

            return vel, rot

        
        dir_o = 0

        if error_o > 0:
            if error_g > 0:
                if error_o > error_g:
                    dir_o = -1
                else:
                    dir_o = 1
            else:
                dir_o = -1
        else:
            if error_g > 0:
                dir_o = 1
            else:
                if error_o > error_g:
                    dir_o = -1
                else:
                    dir_o = 1


        s = max_val
        rot_o = abs(error_o*0.5*s/half_n)*dir_o
        rot_g = error_g*0.6*(1 - s)/half_n

        rot = rot_o + rot_g
        vel = 0.3*(1 - s*error_o/half_n)

        return vel, rot



    def update_gravity_graph(self):
        gamma_r = 5 # scalar multiple. Can be tweaked to change intensity of object repulsion. 20 is fairly smooth. Larger will mean sharper changes in robot behaviour
        gamma_a = 4 # scalar multiple pertaining to object attraction. Approximately 4 is an ideal value
        theta_ROV = math.pi/3 # angular range of view
        repulsion_offset = 0.3 # distance (in meters) away from obstacles the robot should stay

        zeta = 0.3 # minimum non-zero worth of a sample at the max detectable distance from the rover
        r_max = 1.4 # max detectable distance of an object
        proximity_offset = 0.6 # objects within this range are automatically deemed to have max value
    
        n = self.n # size of gravity_graph array
        d_list = {'s':0.5, 'l':1.8, 'o':0.9, 'r':0.5} # dictionary of approximate maximum object diameters

        self.gravity_graph *= 0 # reset gravity graph

        # loop over all visible objects
        for code in self.objects:
            if self.objects[code] == None or self.objects[code] == []:
                continue
                
            for obj in self.objects[code]:
                [r, theta, identifier] = obj # store object data under meaningful variable names
            
                q = round((theta/theta_ROV + theta_ROV/2)*n/theta_ROV) # the index within the gravity_graph array at which an object is centred
                a = round(2*d_list[identifier]/r) # half the size of the object in pixels on the gravity graph (accounts for distance from the rover)
                    
                q = min(max(q, 0), 50)
    
                if identifier == self.goal or self.goal == 's' and identifier == 'r': # if object is the goal object (can only be 's' or 'l'), it is attractive
                    #self.goal_tracker = [r, theta] # last known location of the goal
                    
                    b = int(max(q - 3*a, 0)) # start index of object in gravity graph (include area of effect)
                    e = int(min(q + 3*a, n)) # end index of object including area of effect
    
                    Ia = 0 # attraction scalar multiple
    
                    # simulate piecewise definition of Ia equation, see documentation
                    if r <= proximity_offset:
                        Ia = 1
                    elif proximity_offset < r and r <= r_max:
                        Ia = (1 - zeta)*(r - r_max)/(proximity_offset - r_max) + zeta # the main Ia function
                    else:
                        continue # Ia is zero so quit, no further calculations are meaningful
    
    
                    # calculate b and e step sizes to use with the construction of the A matrix
                    b_step = 0
                    e_step = 0
                    
                    try:
                        b_step = 1/(q - b)
                    except ZeroDivisionError:
                        b_step = 0
                        
                    try:
                        e_step = 1/(e - q)
                    except ZeroDivisionError:
                        e_step = 0
                        
    
                    # calculate b and e array values to concatenate into A matrix
                    b_vals = numpy.flip(-numpy.arange(q - b)*b_step, axis=0)
                    e_vals = numpy.arange(e - q)*e_step
                    
                    A = numpy.concatenate((b_vals, e_vals)) # build A matrix for use with the distribution function fd
                    fd = 2/(numpy.power(math.e, gamma_a*A) + numpy.power(math.e, -gamma_a*A)) # calculate fd vector
                    M = Ia*fd # multiply attraction scalar by distribution function
                    self.gravity_graph[b : e] -= M # update gravity graph vector to contain new data
    
                elif identifier != 's': # if object is not the goal, it is repulsive
                    b = max(q - 2*a, 0) # the start index of the object on the gravity graph
                    e = min(q + 2*a, n) # the end index of the object
                    
                    Ir = 1/(1 + math.exp(gamma_r*(r - proximity_offset))) # the intensity of object repulsion
                    self.gravity_graph[b : e] += Ir # update gravity graph vector to contain new data


        for i in range(self.n):
            self.gravity_graph[i] = min(max(self.gravity_graph[i], -1), 1)


    
    def update_goal(self):
        if self.has_sample:
            if self.goal == 's':
                self.goal = 'l'
                self.goal_tracker = None
                self.tracker_TO = 0

                time.sleep(1)
            else:
                self.goal = 'l' # always ensuring goal is updated to a valid value
        else:
            if self.goal == 'l':
                self.goal = 's'
                self.goal_tracker = None
                self.tracker_TO = 0

                time.sleep(1)
            else:
                self.goal = 's' # always update goal
            
            
        (self.status.update_status('s') if self.goal == 's' else self.status.update_status('r'))
                
                
                
    def dance(self):
        self.movement = [0, 0]
        self.set_velocities()
        self.status.strobe()
                
                

    def check_goal_updates(self):
        if self.goal_tracker != None: # if goal tracker is not None, activate goal-specific managment code
            r_threshold = 0.20 # min distance within which a goal is relevant
            
            r, _, identifier = self.goal_tracker # unpack goal_tracker data
            
            if r < r_threshold: # check goal_tracker is a relevant object
                if identifier == 'r' and not self.arms_prepped:
                    self.movement = [0, 0]
                    self.set_velocities()
                    self.collector.PrepArms()
                    self.arms_prepped = True
                    
                if self.goal == 's': # check if goal is sample searching
                    objects = self.objects['s'] + self.objects['r'] # compile goal relevant objects (samples and rocks)
                    
                    ranges = [] # init ranges list
                    
                    if objects != None and objects != []: # ensure objects list is iterable
                        ranges = [obj[0] for obj in objects] # compile ranges from objects
                        
                    activate = True # init activate variable. Must be initialised to true and proved false if r if simialar to the range of any visible goal object
                    
                    for rng in ranges: # loop over ranges
                        offset = 0.07 # range threshold
                        
                        if rng - offset < r and r < rng + offset: # if r is within this threshold for any goal object, goal_tracker is considered to be visible, activate = False
                            activate = False # set activate variable
                            break
                
                    if identifier == 's':
                        if activate or r < 0.1: # if r is dissimilar to all other goal ranges, it must be out of view, meaning it is likely within grabbing distance, collect in this case and in the case where r is very close (ie r < 0.1)
                            self.collect_sample(True) # activate sample collection but inch forward so to increase successful collection
                
                    elif identifier == 'r':
                        if r < 0.20:
                            self.movement = [0.09, 0]
                            self.set_velocities()
                            time.sleep(1.4)
                            
                            self.movement = [0, 0]
                            self.set_velocities()
                            
                            if self.arms_prepped:
                                self.flip_rock(True)
                            
                            self.movement = [0.1, 0]
                            self.set_velocities()
                            time.sleep(0.3)
                            self.movement = [0,0]
                            self.set_velocities()
                            self.collect_sample(False)
                            
            elif self.goal == 'l':
                vel, rot = self.movement
                self.movement = [0.08, rot]
                
                if r < 0.4:
                    '''rot_accel = 0
                    print('doing the thing')
                    
                    for i in range(10):
                        self.load_objects()
                        self.update_gravity_graph()
                        ro_accel = self.movement[1]*self.delta
                        self.movement = self.read_graph()
                        _, rot = self.movement
                        self.movement = [0, rot*rot_accel]
                        self.set_velocities()
                        time.sleep(0.2)'''
                        
                    self.movement = [0.45, 0]
                    self.set_velocities()
                    time.sleep(0.8)
                    
                    self.movement = [0, 0]
                    self.set_velocities()
                    time.sleep(2)
                    self.deposit_sample()
                    time.sleep(1)
                    self.drive_off_lander()
                    


    def main(self):
        self.flip_rock(False)
        self.drive_off_lander() # drive off lander !!!!!!!!!!!!! IT DRIVES BACKWARDS !!!!!!!!!!!!!!!
        
        
        #object avoidance test code
        #self.has_sample = True
        #self.collector.CloseCollector(False)
         
        # run until rover decides it has completed its tasks. ie terminate_main_loop = True
        while not self.terminate_main_loop:
            start_time = time.time() # start point of each loop


            self.load_objects() # load object data
            self.update_goal() # update goal
            self.update_gravity_graph() # take visible objects and update gravity graph
            self.movement = self.read_graph() # read the new gravity graph and set new movement data
            self.check_goal_updates() # if check_goal_updates returns True, it is chasing a sample that is off screen, skip graph reading
            
            self.set_velocities() # set movement data
            self.tracker_TO += 1
            
            if self.tracker_TO >= self.tracker_TO_limit:
                self.tracker_TO = 0
                self.goal_tracker = None


            # calculate variable delay and sleep for this time
            end_time = time.time()
            run_time = end_time - start_time
            #print('Movement: ' + str(self.movement))
            delay = max(0, self.delta - run_time)
            time.sleep(delay)
    

    
    def __init__(self):
        # define globals
        self.max_vel = 0.45 # maximum rover velocity
        self.max_rot = 0.785 # maximum rotational velocity
        self.movement = [0, 0] # movement variable
        self.objects = {'s': [], 'l': [], 'o': [], 'r': []} # visible objects container
        self.terminate_main_loop = False # global quit boolean, set to True when rover has completed tasks
        self.n = 50
        self.gravity_graph = numpy.zeros(self.n) # gravity graph used to house attraction values
        self.goal = None # 's' = search for sample or rock, 'l' = look for lander
        self.has_sample = False
        self.delta = 0.1 # time delay between main loop runs
        self.num_samples = 3
        self.sample_count = 0
        self.goal_tracker = None
        self.tracker_TO_limit = 50
        self.tracker_TO = 0
        self.arms_prepped = False

        self.status = Status.Status()
        self.vision = Vision_j_edit.Vision()
        self.mobility = Mobility.Mobility()
        self.collector = SampleCollector.SampleCollector()



import numpy, time, math
import Mobility
import SampleCollector
import Vision_j_edit
import Status

if __name__ == "__main__":
    main = MainBrain()
    main.main()



