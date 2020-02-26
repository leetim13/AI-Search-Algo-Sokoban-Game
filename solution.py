#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems

def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    
    heur=0 #init
    for box in state.boxes:
      min_dist = float("inf") 
      for storage in state.storage:
          dist = abs(storage[0]-box[0])+abs(storage[1]-box[1])
          if dist <= min_dist:
              min_dist=dist
      heur += min_dist  
    return heur


#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  
  search_engine = SearchEngine('custom', 'full')
  wrapped_fval_function = (lambda sN: fval_function(sN, weight))
  search_engine.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)


  start_time = os.times()[0]
  goal = search_engine.search(timebound) 

  if goal:
      costbound = goal.gval 
      time_remaining = timebound - (os.times()[0] - start_time)
      best = goal
      while time_remaining > 0 and not search_engine.open.empty: 
          initial_time = os.times()[0]
          new_goal = search_engine.search(time_remaining, costbound = (float("inf"), float("inf"), costbound))
          time_remaining = time_remaining - (os.times()[0] - initial_time) 
          if new_goal:
              costbound = new_goal.gval
              best = new_goal
      return best 
  else:
      return False

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  search_engine = SearchEngine('best_first', 'full')
  search_engine.init_search(initial_state, goal_fn = sokoban_goal_state, heur_fn = heur_fn)
    
  start_time = os.times()[0]
    
  goal = search_engine.search(timebound) # Goal state.
    
  if goal:
     costbound = goal.gval # Costbound.
     time_remaining = timebound - (os.times()[0] - start_time)
     best = goal
     while time_remaining > 0 and not search_engine.open.empty: # While time remaining
         initial_time = os.times()[0]
         new_goal = search_engine.search(time_remaining, costbound = (costbound, float("inf"), float("inf")))
         time_remaining = time_remaining - (os.times()[0] - initial_time) # Update remaining time.
         if new_goal:
             costbound = new_goal.gval
             best = new_goal
     return best # Return the best state.
  else:
      return False



def heur_alternate(state):
#IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
    #heur_manhattan_distance has flaws.   
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    cost = 0
    for box in state.boxes:
        if (box not in state.storage):
            if (out_of_bounds(left(box), state) or (left(box) in state.obstacles)):
                if (out_of_bounds(down(box), state) or (down(box) in state.obstacles)):
                    return float("inf")
                if (out_of_bounds(up(box), state) or (up(box) in state.obstacles)):
                    return float("inf")
                if down(box) in state.boxes:
                    cost += 1
                if up(box) in state.boxes:
                    cost += 1
        #    
            if up(box) in state.boxes:
                if (left(box) in state.obstacles) or (left(box) in state.obstacles):
                    cost += 0.5
                if (right(box) in state.obstacles) or (right(box) in state.obstacles):
                    cost += 0.5
                
            if down(box) in state.boxes:
                if (left(box) in state.obstacles) or (left(box) in state.obstacles):
                    cost += 0.5
                if (right(box) in state.obstacles) or (right(box) in state.obstacles):
                    cost += 0.5
                    
            if left(box) in state.boxes:
                if (out_of_bounds(down(box), state) or (down(box) in state.obstacles)):
                    cost += 1
                if down(box) in state.boxes:
                    cost += 0.5
                if (up(box) in state.obstacles) or (up(box) in state.obstacles):
                    cost += 1
                if up(box) in state.boxes:
                    cost += 0.5
                    
            if (out_of_bounds(right(box), state) or (right(box) in state.obstacles)):
                if (out_of_bounds(down(box), state) or (down(box) in state.obstacles)):
                    return float("inf")
                if (out_of_bounds(up(box), state) or (up(box) in state.obstacles)):
                    return float("inf")
                if down(box) in state.boxes:
                    cost += 1
                if down(box) in state.boxes:
                    cost += 1
                    
            if right(box) in state.boxes:
                if (out_of_bounds(down(box), state) or (down(box) in state.obstacles)):
                    cost += 1
                if down(box) in state.boxes:
                    cost += 0.5
                if (up(box) in state.obstacles) or (up(box) in state.obstacles):
                    cost += 1
                if up(box) in state.boxes:
                    cost += 0.5  
            robo_dist = []
            for robot in state.robots:
                robo_dist.append(check_distance_robot(box, robot))
                if up(robot) in state.robots:
                    cost += 0.5
                if down(robot) in state.robots:
                    cost += 0.5
                if left(robot) in state.robots:
                    cost += 0.5
                if right(robot) in state.robots:
                    cost += 0.5                  
            cost += min(robo_dist)
            min_dist = []
            for goal in state.storage:
                if goal not in state.boxes:
                    min_dist.append(check_distance_robot(box, goal) * 2)
            cost += min(min_dist)
#    for robot in state.robots:
#        if robot in state.storage:
#            cost += 0.5
    #return corner_checking(state)
#    cost += robot_beside_nothing(state)
#    cost += distance(state)
#    cost += robo_dist(state)
#    cost += wall(state)
#    cost += next_dead(state)
#    cost += box_goal_dist(state) 
    return cost 

def out_of_bounds(box, state):
  if box[0] < 0: 
      return True
  if box[1] < 0: 
      return True
  if box[0] >= state.width:
      return True
  if box[1] >= state.height: 
      return True
  return False

def up(box):
  return (box[0], box[1]+1)
def down(box):
  return (box[0],box[1]-1)
def left(box):
  return (box[0]-1,box[1])
def right(box):
  return (box[0]+1,box[1])

def check_distance_robot(box, robot):
  return abs(box[0]-robot[0])+ abs(box[1]-robot[1])
