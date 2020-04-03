# AI-Search-Algo-Sokoban-Game
An AI search algorithm to solve the Sokoban Puzzle involving multiple agents (robots), 
where warehouse robot(s) must push boxes into storage spaces on path containing obstacles and/or other boxes.

Implemented with various search strategies:` ‘depth first’, ‘breadth first’, ‘best first’, (weighted) ‘a star’, or ‘custom’`, and multiple heuristics 
to optimize search time and space complexity (denoted `'time bound'` and `'cost bound'`).

Implemented cycle checking level: `‘none’`, `‘path’`, or `‘full’`.

### Description of Sokoban

• The puzzle is played on a rectangle board that is a grid board with N squares in the x-dimension and
M squares in the y-dimension.\
• Each state contains the x and y coordinates for each robot, the boxes, the storage spots, and the
obstacles.\
• From each state, each robot can move North, South, East, or West. No two robots can move simultaneously, however. If a robot moves to the location of a box, the box will move one square in the
same direction. Boxes and robots cannot pass through walls or obstacles, however. Robots cannot
push more than one box at a time; if two boxes are in succession the robot will not be able to move
them. Movements that cause a box to move more than one unit of the grid are also illegal. Whether
or not a robot is pushing an object does not change the cost.\
• Each movement is of equal cost. \
• The goal is achieved when each box is located in a storage area on the grid\



Sokoban can be played online at https://www.sokobanonline.com/play
