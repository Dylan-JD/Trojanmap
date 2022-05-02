# EE538 Final Project - Spring 2022 - TrojanMap
Students: Jian Dong, Zi Yan
## 1.Overview
In this program, we have such data structures, the class Node and class Trojanmap. The attributes of node is shown as below diagram.
![node_attributes](https://user-images.githubusercontent.com/97215161/162642443-c03728f6-7356-4945-baae-0d44264eb178.jpg)   
The method of trojanmap is shown as below diagram.
![image](https://user-images.githubusercontent.com/97215161/166167619-e13af03f-b610-47ae-ae33-f0cbf654c520.png)   
We created a new unodered_map:namemap for trojanmap, which the index is name and the value is id. It is used by GetID to avoid iterate data map. In addition, we have create a cycledetection_helper function to help cycledetection to do dfs.Meanwhile, we also create a helper fuction for travelling trojan backtracking function to do recursive dfs.   
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
Input: {std::string} name: location name   
Return value: {std::pair<double,double>} (lat, lon): the pair of (latitude, longitude)   
##### 2> Boundary Conditions Check
- If the location name does not exist, the function returns  (-1, -1).   
##### 3> Implementation method
For this function, I used data maps and a helper function. Because the input value is location name but the key of data map is id, so  
1) first, I call the GetID function to get the id through the location name. 
2) Then, I use id as the index to get the node. Through the attribute in the node we can get latitude and longitude of this location name. 
3) Then return the pair of (latitude, longitude) as the return value of this function.
#### 2.2.2.Time Complexity Analysis
For this function, the helper function GetID contribute the main run time comlplexity.    
The complexity for GetID is the complexity of searching the key in a unordered_map, it is O(1) because we have create a unordered_map data structure called name_map, the key is name and the value is ID    
and the other operation of GetPosition is constant time complexity, so the time complexity is O(1).
#### 2.2.3.Time Spent
Time taken by function: 0 ms
#### 2.2.4 helper functions
Under this step, we have created a lot of helper funtions to get the attributes of the node, Including GetLat, GetLon, GetName, GetNeighborIDs, GetID.   
Except GetID, the rest of them are all take ID of the node and return the node's attributes. I just use their id to index the node in data map, and return the attributes value of the node.   
For all of them we have checked if the id exists in the map. For these functions they only take O(1) to find the id in the map. While for GetID, it takes the location name and return ID, so we need to iterate the data map to find which node's name satisfy the requirement. So it would take O(N).   
So to avoid iteratint the map every time that we called the GetID. We create a unordered_map data structure called name_map, the key is name and the value is id. It will create when constructing the Trojan class objects. So our GetID function can index the name in O(1) time complexity.
#### 2.2.5 result


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
- If the input is not found, the function returns a name with the smallest edit distance.

##### 3> Implementation method
FindClosestName():
1) Traverse all nodes on the map
2) If name is not empty, calculate its edit distance from the input name.
3) If the distance is less than the minimum edit distance, update the min_distance.
4) return min_distance.

CalculateEditDistance():
1) Implemented by dynamic programming
2) if the ith char of name a == the jth char of name b, dp[i][j] = 1 + std::min(dp[i-1][j-1]-1, std::min(dp[i-1][j], dp[i][j-1]));
3) else dp[i][j] = 1 + std::min(dp[i-1][j-1], std::min(dp[i-1][j], dp[i][j-1]));

#### 2.3.2.Time Complexity Analysis
O(n*a*b), n: the size of data, a: the length of input name a, b: the length of input name b.  
We need to traverse all nodes on the data and compute edit distance from the target name. Computing edit distance: traverse every element in both a and b.  

#### 2.3.3.Time Spent
Time taken by function: 0 ms

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
2) initial the visited_map to keep every nodes' condition, the memo d to memrize the distance between each node and start node, the unvisited_map to keep all the   unvisited node, a queue to help iterate through the neighbor, a stack to find shortest path.  
3) push start point into the queue and start iterate all the points.  
4) get the front point of the queue and then iterate all of unvisited neighbor of this node.  
5) calculate the distance between this node and its parent node, then add the distance between its' parents and start point to get the new distance.  
6) For memo d  
    -- if this key isn't initialized, use this distance to initialiaze.   
    -- if this key is initialized.  
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
    -- if this key is initialized.  
        --- check if the new distance is less than the record, update it use new distance.  
        --- if not, keep the record.  
8) (early stop)check flag  
    --if d is updated, then continue iterating  
    --else d isn't updated, break.  
9) when finish relaxing every edges, push the nodes in shortest path into stack through the record of their parent node id.  
10) then pop all the node from stack and push them into vector and return it.  

#### 2.4.2.Time Complexity Analysis
- Dijkstra:  
    iterate ever node is O(n).  
    in ever iteration, we need to iterate every neighbor and unvisited map. So, it would be O(n^2).  
    push path into stack would be O(n) (the worse case).  
    push path into vector would be O(n) (the worse case).  
    So, Finally the Complexity for Dijkstra is O(n^2).  

- Bellman_Ford:  
    iterate every edges is O(m) (the worse case).  
    iterate ever node is O(n).  
    push path into stack would be O(n) (the worse case).  
    push path into vector would be O(n) (the worse case).  
    So, Finally the Complexity for Dijkstra is O(m*n).  

#### 2.4.3.Time Spent
The First row is starting point - destination, the second row is the runtime of Dijkstra, the third row is the runtime of Bellman_Ford   
![image](https://user-images.githubusercontent.com/97215161/166129229-5018360c-c8ff-48f5-ace4-6874cb7b3ca7.png)


#### 2.4.4.result
From leavey library to Proto Homes LLC
![image](https://user-images.githubusercontent.com/97215161/166129298-d88d089f-1173-4761-a2b1-30d42e7e4668.png)


### 2.5. Step 5: Cycle Detection
#### 2.5.1.Detailed description
##### 1> Input and Return Value
- inSquare:  
Input: {std::string} id: location id,
       {std::vector<double>} square: four vertexes of the square area  
Return value: {bool}: in square or not  

- GetSubgraph:  
Input: {std::vector<double>} square: four vertexes of the square area  
Return value: {std::vector<std::string>} subgraph  : list of location ids in the square  
    
- CycleDetection:  
Input: {std::vector<std::string>} subgraph: list of location ids in the square,
       {std::vector<double>} square: four vertexes of the square area.  
Return value: {bool}: whether there is a cycle or not.  

- CycleDetection_Helper:  
Input: {std::string} node_id: the id of input node(in subgraph),
       {std::string} parent_id: the id of input node's parent,
       {std::vector<double>} square: four vertexes of the square area,
       {std::unordered_map<std::string, bool> } visited: the memo to store whether the node has been visited  
Return value: {bool}: whether there is a cycle or not.  
    
##### 2> Boundary Conditions Check  
- inSquare:  
    If the id in invaild, then return false.
    If the square input is invaild, then return false.  
    
- GetSubgraph:  
    If the square input is invaild, then return empty vector.  

- CycleDetection:  
    If the square input is invaild, then return false. 

- CycleDetection_Helper:  
    all the input is input by myself, so there wouldn't any invaild input.


##### 3> Implementation method  
- inSquare:  
    1) get the latitude and longtitude using its id.
    2) check if the position is in the input ranges.  

- GetSubgraph:  
    1) iterate all the node in the data map.
    2) check if it is in the square that input.  

- CycleDetection:  
    1) check if the input is valid
    2) call the CycleDetection_Helper function.
   
- CycleDetection_Helper:I used recursive dfs to find the cycle.  
    1) mark this node as visiting.
    2) find all the neighbors of input node that in the square.
    3) iterate all the neighbors.  
        -- if this neighbor is unvisited
            --- call the CycleDetection_Helper recursivly
        -- else if it is visiting
            ---it shows that this is a circle, then return true
    4) when finished this node's searching, mark this node as visited
    5) if there is a circle, return true
       if all the nodes in subgraph has been visited and there is no circle, return false.
 
#### 2.5.2.Time Complexity Analysis
- inSquare:   
    all the operation is O(1), so this function's time complexity is O(1).

- GetSubgraph:  
    iterate the map should take O(n), so this function's time complexity is O(n).

- CycleDetection:  
    iterate all the node use O(n)  
    iterate all the edge use O(m)  
    so this function's time complexity is O(m+n).
    
- CycleDetection_Helper:  
    iterate all the neighbor and edges use O(m+n)  
    so this function's time complexity is O(m+n).

#### 2.5.3.Time Spent
If I choose {-118.299, -118.264, 34.032, 34.011} as my square input.  
    - CycleDetection: 
    Time taken by function: 0 ms
    
    
### 2.6. Step 6: Topological Sort
#### 2.6.1.Detailed description
##### 1> Input and Return Value
- ReadLocationsFromCSVFile:  
    Input: {std::string} locations_filename: locations_filename  
    Return value: {std::vector<std::string>}: locations 
- ReadDependenciesFromCSVFile:  
    Input: {std::string} dependencies_filename: dependencies_filename  
    Return value: {std::vector<std::vector<std::string>>}: dependencies
- DeliveringTrojan:  
    Input: {std::vector<std::string>} locations: locations, 
    {std::vector<std::vector<std::string>>} dependencies: prerequisites  
    Return value: {std::vector<std::string>} results: results

##### 2> Boundary Conditions Check  
- ReadLocationsFromCSVFile:  
    If the file address is invaild, then return empty vector  
- ReadDependenciesFromCSVFile:  
    If the file address is invaild, then return empty vector  
- DeliveringTrojan:  
    If the location is empty, it will return empty vector.  
    If the dependence is empty, it will return a reversed vector of location vectors.  
    If the location sites contain some place that depencence don't have, the output would reverse location first, then put the place that dependency don't have in the first, and then follow the dependence.  
    If the dependency contains some place that location don't have, if will return empty vector.  
    If the dependency contains cycle, return empty vector.

##### 3> Implementation method  
- ReadLocationsFromCSVFile:
    Read CVS contents row by row and each row is pushed separately into the vector.
- ReadDependenciesFromCSVFile:  
    Read CVS contents row by row, each row is seperated by comma to construct a new vector and then push this new vector separately into the final vector.
- DeliveringTrojan:  
    1) construct DAG.  
    2) construct indegree form.  
    3) push all of the node which indegree = 0 into the bfs queue.  
    4) iterate all the neighbor, and for each neighbor, their indegree minus 1.  
    5) push all the new node that indegree = 0 into bfs queue after previews node iteration finish.
    6) if resulte size equal to locations size shows that the dependence can be followed, then renturn the result.  
       else return empty vector. 

#### 2.6.2.Time Complexity Analysis
- ReadLocationsFromCSVFile:
    iterate all the line using O(n), so the time comlpexity for this function is O(n).  
- ReadDependenciesFromCSVFile:
    iterate all the line using O(n), for each line is m, so the time comlpexity for this function is O(nm).  
- DeliveringTrojan:  
    Construct DAG using O(n)
    Construct indegree form using O(n^2)  
    bfs takes O(n^2)  
    so the time comlpexity for this function is O(n^2).  
    
#### 2.6.3.Time Spent
- DeliveringTrojan:  
    if I choose locations Ralphs, KFC, Chick-fil-A, Arco, Leavey Library, Subway 1, Adams Normandie Historic District, Honda, Main & Pico, Security Checkpoint  
    if I choose dependence {Ralphs,Chick-fil-A} {Ralphs,KFC} {Chick-fil-A,KFC} {KFC,Arco} {Arco,Leavey Library} {Leavey Library,Subway 1} {Subway 1,Adams Normandie Historic District} {Adams Normandie Historic District,Honda} {Honda,Main & Pico} {Main & Pico,Security Checkpoint}
    Time taken by function: 0 ms

#### 2.6.4. Result
![image](https://user-images.githubusercontent.com/97215161/166130238-1f1b45b7-bff4-4f8b-a803-9cc5cd136261.png)

### 2.7. Step 4: The Travelling Trojan Problem (AKA Travelling Salesman!)
#### 2.7.1.Detailed description
##### 1> Input and Return Value
- TravellingTrojan_Brute_force(), TravellingTrojan_Backtracking(), and TravellingTrojan_2opt():   
Input: {std::vector<std::string>} input : a list of locations needs to visit   
Return value: {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path  

##### 2> Boundary Conditions Check
- If the input is empty, the function returns an empty.
- If the input size is 1, the function returns records which the first is 0, the second is the input id.
- else returns a pair of minimum distance and the all the progress to get final path  

##### 3> Implementation method
TravellingTrojan_Brute_force():
1) Implemented by back tracking algorithm.
2) In each recursion, we traverse all locations.
3) If current locaiton is not visited, add it to curr_path and update curr_cost.
4) After calling the backTracking function, we remove the location from the curr_path.
5) If the curr_path size equals the input size and curr_cost is less than min_cost, we update min_cost and add the curr_path to the result. 

TravellingTrojan_Backtracking():
1) Same as Brute Force, also implemented by dynamic programming.
2) The difference is that when the cur_cost >= min_cose we early stop the this recursiong and return.

TravellingTrojan_2opt():
1) We use two loops to obtain a sub part in location ids vector and reverse this sub part.
2) If the updated cur_cost is smaller, we go back to start again.
3) Repeat until no improvement.

#### 2.7.2.Time Complexity Analysis
TravellingTrojan_Brute_force(): O(n!)   
TravellingTrojan_Backtracking(): O(n!)   
TravellingTrojan_2opt(): O(n^2)  

#### 2.7.3.Time Spent
![image](https://user-images.githubusercontent.com/85814736/166167615-30336cdd-9bae-4888-be75-534ff5566791.png)
    
### 2.8. Step 7: Find Nearby
#### 2.8.1.Detailed description
##### 1> Input and Return Value
Input:   
        {std::string} attributesName: the attribute name   
        {std::string} name: the name of the location  
        {int} r: search radius  
        {int} k: search numbers  
Return value: {std::vector<std::string>}: location name that meets the requirements  

##### 2> Boundary Conditions Check
- If attributesName.empty() || name.empty() || r <= 0 || k <= 0, the function returns an empty.
- Else returns a vector of string ids

##### 3> Implementation method
1) Traverse all nodes on the map  
2) If its attribute == attributesName and distance <= r, we put this node into a min_heap  
3) returns at most k elements in the min_heap
    
#### 2.8.2.Time Complexity Analysis
O(n * logn), n is the size of data.  
We need to traverse all nodes on the map (time: n) and push into the heap (time: logn). The worst case is all nodes are qualified.  

#### 2.8.3.Time Spent
![image](https://user-images.githubusercontent.com/85814736/166169349-ef235195-00a4-4a38-8e06-deed4d8cc51d.png)
![image](https://user-images.githubusercontent.com/85814736/166169432-3b76c811-383b-484d-9d4a-b0a81a39c3a5.png)

### 3. Discussion and Conclusion
- For shortest path algorithm     
According to the table 1 in 2.4.3  
the runtime of Dijkstra is extremely shorter than the bellman-ford. But Dijkstra can’t handle negative edges and cycles Because the map doesn’t have this situation, we can use Dijkstra to improve search speed.   
- For Travelling Trojan Problem  
![image](https://user-images.githubusercontent.com/85814736/166169989-110aed0c-e319-41d9-9317-c3650b7f558e.png)
2 opt is the most efficient algorithm. Early backTracking is the second. When the number of points is greater than 12, Brute Force is not practical, but 2 opt only needs several milliseconds.

                                                      
### 4. Lesson Learned 
1)	We learned the importance of time complexity and how to analyze the time complexity for different cases.
2)	We also learned shortest path algorithms and the difference between them.
3)	Travelling salesman problem is really interesting. Early backtracking can obviously improve the algorithm’s performance. And the 2 opt algorithm is amazingly fast to find the optimum path.

