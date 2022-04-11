# EE538 Final Project - Spring 2022 - TrojanMap
Students: Jian Dong, Zi Yan
## 1.Overview
In this program, we have such data structures, the class Node and class Trojanmap. The attributes is shown as below diagram.
![node_attributes](https://user-images.githubusercontent.com/97215161/162642443-c03728f6-7356-4945-baae-0d44264eb178.jpg)

## 2.Function Descriptions

### 2.1. Step 1: Autocomplete the location name
#### 2.1.1.Detailed description
##### 1> Input and Return Value
Input: the partial name of the location (std::string name)   
Return value: a list of possible locations with partial name as prefix (std::vector< std::string >)

##### 2> Boundary Conditions Check
- If the input is empty, the function returns an empty.
- If the input is not found, the function returns an empty and outputs "No matched locations."
- Otherwise, the function returns a vector of names given a partial name.

##### 3> Implementation method
1) Remove spaces (if exist) at the end of the input string
2) Traverse all nodes on the map
3) If the size of input is greater than the size of node’s name, we skip this node
4) Covert two strings to the lower cases
5) If the input name is found in the node's name and the index is 0, push the node’s name into result vector
#### 2.1.2.Time Complexity Analysis
O(n*m), n is the length of input name, m is the size of data.  
We need to traverse all nodes on the map and compare two strings.
#### 2.1.3.Time Spent
Time taken by function: 2 ms


### 2.2. Step 2-1: Find the place's Coordinates in the Map
#### 2.2.1.Detailed description
##### 1> Input and Return Value
First, the input of this fucntions is location name(std::string type) and the return value is the pair of (latitude, longitude)(std::pair<double, double> type).
##### 2> Boundary Conditions Check
In this function, the input is location name, so we need to check if the location name exists. So we check the return value of GetID to check if the input value is correct. If it isn't correct, the function would return (-1, -1).
##### 3> Implementation method
For this function, I used data maps and a helper function. Because the input value is location name but the key of data map is id, so  
1) first, I call the GetID function to get the id through the location name. 
2) Then, I use id as the index to get the node. Through the attribute in the node we can get latitude and longitude of this location name. 
3) Then return the pair of (latitude, longitude) as the return value of this function.
#### 2.2.2.Time Complexity Analysis
For this function, the helper function GetID contribute the main run time comlplexity. The complexity for GetID is the complexity of searching the key in a unordered_map, it is O(1), and the other operation of GetPosition is constant time complexity, so the time complexity is O(1).
#### 2.2.3.Time Spent
Time taken by function: 0 ms
#### 2.2.4 helper functions
Under this step, we have created a lot of helper funtions to get the attributes of the node, Including GetLat, GetLon, GetName, GetNeighborIDs, GetID. Except GetID, the rest of them are all take ID of the node and return the node's attributes. I just use their id to index the node in data map, and return the attributes value of the node. For all of them we have checked if the id exists in the map. For these functions they only take O(1) to find the id in the map. While for GetID, it takes the location name and return ID, so we need to iterate the data map to find which node's name satisfy the requirement. So it would take O(N). So to avoid iteratint the map every time that we called the GetID. We create a unordered_map data structure called name_map, the key is name and the value is id. It will create when constructing the Trojan class objects. So our GetID function can index the name in O(1) time complexity.

### 2.3. Step 2-2: Check edit distance between two location names
#### 2.3.1.Detailed description
##### 1> Input and Return Value
- FindClosestName: Given a location name, return the name with smallest edit distance.  
Input: {std::string} name: location name      
Return value: {std::string} tmp: similar name  

- CalculateEditDistance: Calculate edit distance between two location names   
Input: {std::string} a, {std::string} b: two location names      
Return value: {int} distance: edit distance 

##### 2> Boundary Conditions Check
- If the input is empty, the function returns an empty.
- If the input is not found, the function returns an empty and outputs "No matched locations."
- Otherwise, the function returns a vector of names given a partial name.

##### 3> Implementation method
1) Remove spaces (if exist) from the end of the input string
2) Traverse all nodes on the map
3) If the size of input is greater than the size of node’s name, we skip this node
4) Covert two strings to the lower cases
5) If the input name is found in the node's name and the index is 0, push the node’s name into result vector
#### 2.3.2.Time Complexity Analysis
O(n*m), n is the length of input name, m is the size of data.  
We need to traverse all nodes on the map and compare two strings.
#### 2.3.3.Time Spent
Time taken by function: 2 ms

### 2.4. Step 3: CalculateShortestPath between two places
#### 2.4.1.Detailed description
##### 1> Input and Return Value
- Dijkstra: Given 2 locations, return the shortest path which is a list of id.
Input: {std::string} location1_name: start, {std::string} location2_name: goal
Return value: {std::vector<std::string>}: path

- Bellman_Ford: Given 2 locations, return the shortest path which is a list of id.
Input: {std::string} location1_name: start, {std::string} location2_name: goal
Return value: {std::vector<std::string>}: path

##### 2> Boundary Conditions Check
- Dijkstra:
If the input name is invalid, the function returns an empty vector.

- Bellman_Ford:
If the input name is invalid, the function returns an empty vector.

##### 3> Implementation method
- Dijkstra:
1) Though GetID, get the id of starting point and ending point.
2) initial the visited_map to keep every nodes' condition, the memo d to memrize the distance between each node and start node, the unvisited_map to keep all the unvisited node, a queue to help iterate through the neighbor, a stack to find shortest path.
3) push start point into the queue and start iterate all the points.
4) get the front point of the queue and then iterate all of unvisited neighbor of this node.
5) calculate the distance between this node and its parent node, then add the distance between its' parents and start point to get the new distance.
6) For memo d
    -- if this key isn't initialized, use this distance to initialiaze. 
    -- if this key is initialized
        --- check if the new distance is less than the record, update it use new distance.
        --- if not, keep the record.
7) find the node in unvisited map that has shortest distance between start point and itself.
8) push this node into queue and update visited map and unvisited map.
9) when finish iterating everynode, push the nodes in shortest path into stack through the record of their parent node id.
10) then pop all the node from stack and push them into vector and return it.

- Bellman_Ford:
1) Though GetID, get the id of starting point and ending point.
2) initial the visited_map to keep every nodes' condition, the memo d to memrize the distance between each node and start node, a queue to help iterate through the neighbor, a stack to find shortest path.
3) iterate n-1 times to update every edges(can early stop). If the memo d isn't updated through one iteration, then stop the iterating.(using flag to check if d updating.
4) push start point into the queue and start iterate all the points.
5) get the front point of the queue and then iterate all of unvisited neighbor of this node.
6) calculate the distance between this node and its parent node, then add the distance between its' parents and start point to get the new distance.
7) (update edges)For memo d
    -- if this key isn't initialized, use this distance to initialiaze. 
    -- if this key is initialized
        --- check if the new distance is less than the record, update it use new distance.
        --- if not, keep the record.
8) (early stop)check flag
    --if d is updated, then continue iterating
    --else d isn't updated, break.
9) when finish relaxing every edges, push the nodes in shortest path into stack through the record of their parent node id.
10) then pop all the node from stack and push them into vector and return it.

#### 2.4.2.Time Complexity Analysis
- Dijkstra:
    iterate ever node is O(n)
    in ever iteration, we need to iterate every neighbor and unvisited map. So, it would be O(n^2)
    push path into stack would be O(n) (the worse case)
    push path into vector would be O(n) (the worse case)
    So, Finally the Complexity for Dijkstra is O(n^2).

- Bellman_Ford:
    iterate every edges is O(m) (the worse case)
    iterate ever node is O(n)
    push path into stack would be O(n) (the worse case)
    push path into vector would be O(n) (the worse case)
    So, Finally the Complexity for Dijkstra is O(m*n).

#### 2.4.3.Time Spent
If I choose Ralphs as my starting point, Chick-fil-A as my ending point.
- Dijkstra:
Time taken by function: 1732 ms

- Bellman_Ford:
Time taken by function: 10931 ms
