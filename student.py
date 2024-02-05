from SMP.motion_planner.node import PriorityNode

from SMP.motion_planner.plot_config import DefaultPlotConfig
from SMP.motion_planner.search_algorithms.best_first_search import *
from commonroad_route_planner.route_planner import RoutePlanner

import numpy as np
from scipy.spatial import cKDTree
import math

class StudentMotionPlanner(AStarSearch): # Better to be GBFS
    def __init__(self, scenario, planningProblem, automata, plot_config=DefaultPlotConfig):
        super().__init__(scenario=scenario, planningProblem=planningProblem, automaton=automata,
                         plot_config=plot_config)
        
        self.optimal_route = optimalRoutePlannng(scenario, planningProblem, self.time_desired, self.distance_initial, 
                                                 self.state_initial.position, self.position_desired)
        
        
    def evaluation_function(self, node_current: PriorityNode) -> float:
        """
        Evaluation function of A* is f(n) = g(n) + h(n)
        """
        if self.reached_goal(node_current.list_paths[-1]):
            node_current.list_paths = self.remove_states_behind_goal(node_current.list_paths)
        
        node_current.priority += (len(node_current.list_paths[-1]) - 1) * self.scenario.dt

        return node_current.priority + self.heuristic_function(node_current=node_current)
    
    def heuristic_function(self, node_current: PriorityNode) -> float: 
        if self.reached_goal(node_current.list_paths[-1]):
            return 0.0
        
        #Survival 
        if self.position_desired is None:
            yaw_angle = node_current.list_paths[-1][-1].orientation 
            position = node_current.list_paths[-1][-1].position

            # closest point on optimal route
            point = self.optimal_route.closest_to_point(position[0], position[1])
            point_orientation = self.optimal_route.orientation_of_point()

            #scoring 
            distance_score = self.optimal_route.score_distance(position)
            orientation_score = self.optimal_route.score_orientation(yaw_angle)

            if node_current.list_paths[-1][0].time_step > 0:
                if self.is_collision_free(node_current.list_paths[-1]):
                    return ((self.time_desired.start - node_current.list_paths[-1][-1].time_step) + distance_score + orientation_score) * 0.5
                else:
                    return np.inf
            return ((self.time_desired.start - node_current.list_paths[-1][-1].time_step) + distance_score + orientation_score) * 0.2
        
        #Goal Based    
        else:
            if np.isclose(node_current.list_paths[-1][-1].velocity, 0):
                return np.inf
            
            time = node_current.list_paths[-1][-1].time_step
            
            distance_to_goal = self.calc_heuristic_distance(node_current.list_paths[-1][-1])
            travelled_distance = SearchBaseClass.calc_travelled_distance(node_current.list_paths[-1])
            yaw_angle = node_current.list_paths[-1][-1].orientation 
            position = node_current.list_paths[-1][-1].position
                        
            # closest point on optimal route
            point = self.optimal_route.closest_to_point(position[0], position[1])
            point_orientation = self.optimal_route.orientation_of_point()
            
            #scoring 
            distance_score = self.optimal_route.score_distance(position)
            orientation_score = self.optimal_route.score_orientation(yaw_angle)
            
            #tracking with optimal solution    
            time_difference, velocity_optimal = self.optimal_route.compare_progress_with_route(
                node_current.list_paths[-1][-1].position, 
                node_current.list_paths[-1][-1].time_step, 
                node_current.list_paths[-1][-1].velocity
            )

            #Collision so remove it from being a possible path 
            if node_current.list_paths[-1][0].time_step > 0 and not self.is_collision_free(node_current.list_paths[-1]):
                return np.inf
            
            if time_difference == -1 and velocity_optimal == -1:
                time_difference = abs(self.time_desired.start - node_current.list_paths[-1][-1].time_step)
                
                if self.velocity_desired.end != np.inf: 
                    velocity_optimal = (self.velocity_desired.start + self.velocity_desired.end)/2
                else: 
                    velocity_optimal = node_current.list_paths[-1][-1].velocity

            positional_distance = (self.euclidean_distance(
                np.array([node_current.list_paths[-1][0].position[0], node_current.list_paths[-1][0].position[1]]), 
                np.array([
                    (self.position_desired[0].start + self.position_desired[0].end) / 2, 
                    (self.position_desired[1].start + self.position_desired[1].end) / 2
                ])
            ))

            if self.velocity_desired.end is np.inf:
                velocity_distance = 0 
            else:
                velocity_distance = abs((self.velocity_desired.end + self.velocity_desired.start) / 2 - node_current.list_paths[-1][-1].velocity)
                            
            orientation_distance = abs((self.orientation_desired.end + self.orientation_desired.start) / 2 - node_current.list_paths[-1][-1].orientation)
                        
            curr_lanelet_ids = self.scenario.lanelet_network.find_lanelet_by_position([node_current.list_paths[-1][-1].position])[-1]

            weights_combined = [
                1.5 * distance_score,
                1.1 * orientation_score,
                #1 * time_difference,
                1 * abs(node_current.list_paths[-1][-1].velocity - velocity_optimal),
                1.5 * positional_distance,
                1 * velocity_distance,
                0.8 * orientation_distance,
                #1 * ((self.time_desired.start - self.time_desired.end / 2) - node_current.list_paths[-1][-1].time_step)
            ]
            
            if curr_lanelet_ids == []: #gone off a lanelet?
                return np.inf
            
            elif self.is_goal_in_lane(curr_lanelet_ids[0]):
                return (sum(weights_combined) / len(weights_combined)) * 0.8
            else:
                return (sum(weights_combined) / len(weights_combined)) * 1.3
        
class optimalRoutePlannng(AStarSearch):
    def __init__(self, scenario, planningProblem, steps_desired, distance_initial, position_initial, position_desired):
        """
        input:
            scenario: commonroad io object
            planningProblem: commonroad io object
            steps_desired: time_desired
            distance_initial: float
            position_initial.attributes (position_initial.start position_initial.end): int
            position_desired: None or [x,y]

        """
        self._distance_initial = distance_initial
        self._position_initial = position_initial
        self._position_desired = position_desired # consists of 2 pairs of points, one at start and one at end
        self._steps_desired_end = max((steps_desired.end + steps_desired.start)//2, steps_desired.end) 
        
        route_planner = RoutePlanner(scenario, planningProblem, backend=RoutePlanner.Backend.NETWORKX)
        candidate_holder = route_planner.plan_routes()
        route = candidate_holder.retrieve_best_route_by_orientation()
        try:
            self._route_path = route.reference_path
            self._relevant_route_path = None #this is redefined when we call track_path
            self._number_of_distance_steps_per_time_step = None #this is redefined when we call track_path
            self._solution = True
        
            # utilise a KD tree to measure if points in the route_path are close to an arbitrary current point from the heuristic 
            self._data_points = cKDTree(self._route_path)

            #now define the required steps to stay on track with the optimal path timewise
            self.track_optimal_path()
        except:
            self._solution = False
        

    def closest_to_point(self, point_x, point_y):
        if self._solution == True:
            # Query the k-d tree for points close to the arbitrary point
            distance_threshold = np.pi/5
            distance, closest_point_index = self._data_points.query((point_x, point_y))

            self._closest_point_index = closest_point_index
            self._point = self._data_points.data[closest_point_index]

            return self._data_points.data[closest_point_index]
        else:
            return 0
    
    def orientation_of_point(self):
        if self._solution == True:
            current_point = self._point
            previous_point = self._data_points.data[self._closest_point_index - 1]
            self._orientation = math.atan2(current_point[1] - previous_point[1], current_point[0] - previous_point[0])
            return math.atan2(current_point[1] - previous_point[1], current_point[0] - previous_point[0])
        else:
            return 0

    #Score distance doesn't include turning just straight-line distances
    def score_distance(self, point):
        if self._solution == True:
            distance = math.sqrt((point[0] - self._point[0])**2 + (point[1] - self._point[1])**2)
            return (distance)
        else:
            return 0 

    #Score orientation can be improved upon, did not do what I assumed it did. 
    def score_orientation(self, orientation):
        if self._solution == True:
            angle_difference = abs(orientation - self._orientation)
            score = math.exp(-0.5 * angle_difference)
            return score
        else:
            return 0
    
    def track_optimal_path(self):
         if self._solution == True:
            #find the initial point and distance from the optimal path
            distance, closest_point_index = self._data_points.query(self._position_initial)

            #deal with the different types of scenarios. E.g., survival vs goal-based
            if self._position_desired is not None:
                average_end_x = (self._position_desired[0].start - self._position_desired[0].end) / 2 + self._position_desired[0].start
                average_end_y = (self._position_desired[1].start - self._position_desired[1].end) / 2 + self._position_desired[1].start
                _, last_point_index = self._data_points.query((average_end_x, average_end_y))
            else:
                last_point_index = len(self._route_path)

            relevant_path_section = self._route_path[closest_point_index: last_point_index]
            self._relevant_route_length = sum(self.euclidean_distance(relevant_path_section[i], relevant_path_section[i + 1])
                                        for i in range(len(relevant_path_section) - 1))

            self._relevant_route_path = cKDTree(relevant_path_section)
            self._number_of_distance_steps_per_time_step = len(relevant_path_section) / self._steps_desired_end 
            if self._number_of_distance_steps_per_time_step == 0:
                self._solution = False

    def compare_progress_with_route(self, current_position, current_time_step, current_velocity):
        if self._solution == True:
            distance_to_closest_point, point_index = self._relevant_route_path.query(current_position)

             #a measure of how far along the relevant route section the object has traveled at the current time step
            progress_made = point_index / self._number_of_distance_steps_per_time_step         

            time_left = (self._steps_desired_end - progress_made)
            time_difference = abs(time_left - current_time_step)

            route_length_remaining = (1-(progress_made / self._steps_desired_end)) * self._relevant_route_length
            velocity_required =  route_length_remaining*(1/time_left*10)

            return time_difference, velocity_required 
        else:
            return -1, -1
           
