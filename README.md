# DijkstraAlgorithm

In Esenboğa airport, there is a parking facility which is in the form of a graph having N
nodes and M edges. The graph does not have self-loops or multiple edges. Each node represents a
parking slot and has a capacity of vehicles it can hold. Each edge has a weight of w, which indicates
that it will cost w units to go from node u to node v. All parking slots have a parking fee F per
vehicle, which is the same for all slots.

There are K identical vehicles entering the parking facility, each vehicle enumerated with
a distinct number from 1 to K. The vehicles enter in their natural order, that is, vehicle number 1
enters, then vehicle number 2, then 3 and so on till vehicle number K. For each vehicle, you have
to print the minimum total cost that is incurred on the vehicle owner. Here, total cost includes cost
of the path taken to reach the parking slot and parking fee of the slot. It is guaranteed that you
can reach any slot from any other slot. All vehicles entering the parking facility enter from the
parking slot 1.

In the input, the first line contains 3 space separated integers, N, M and F. N denotes the
number of nodes, M denotes the number of edges and F denotes the parking fee. The second line
consists of N space separated integers denoting the seating capacity of each parking slot. Following
M lines contain three space separated integers each: u, v and w, denoting we can reach from node
u to node v incurring a cost of w units. The last line of input contains an integer K denoting the
number of vehicles enter the parking facility.

In the first line it is given that we have 5 parking slots, 4 connections between the parking
slots and 10 as parking fee which is the base fee for each car enters the parking facility. In the
second line, we have 5 parking slots with their capacity which are 1, 2, 1, 1, and 2. In the next 4
lines, the distance between these parking slots is given and the number of vehicles will be entering
the parking facility is 5. You need to give the total cost each car enters the facility.

### Sample Input 
```
 5 4 20
 1 2 1 1 2
 1 2 2 
 4 5 1
 3 4 2
 1 3 1
 5
```
### Sample Output 
```
20 21 22 22 23
```
In the output, print K space separated integers denoting answer for each vehicle. ith integer
in the space separated integers denotes answer for ith vehicle number. If it is not possible to enter
a parking slot print −1 for that vehicle.

The output for the about inputs as follows. Please check your program with this input as
well as the others that you will create. Please note that we may use other inputs when grading your
assignments.



