## A* algo
The starting point was set to `100 250` and goal to `200 350`
### $$\epsilon=1$$
![](./img/astar_e1.png)
expanded nodes: 135502
cost:  730.3229432149761
### $$\epsilon=10$$
![](./img/astar_e10.png)
expanded nodes: 26981
cost:  775.0580079512716
### $$\epsilon=20$$
![](./img/astar_e20.png)
expanded nodes: 26165
cost:  779.2001435750027

### Discussion
As can be seen from the results the number of expanded nodes decreases with a greater epsilon
but at a cost of a longer, less optimal path length


v2
e20
expanded nodes: 24142
cost:  476.3574311003817

e1
expanded nodes: 42013
cost:  476.3574311003817

v3
e1
expanded nodes: 135502
cost:  730.3229432149761

e20
expanded nodes: 26165
cost:  779.2001435750027

