# EE538 Final Project - Spring 2022 - TrojanMap
Students: Jian Dong, Zi Yan
## 1.Overview
In this program, we have such data structures, the class Node and class Trojanmap.
![node_attributes](https://user-images.githubusercontent.com/97215161/162642443-c03728f6-7356-4945-baae-0d44264eb178.jpg)

## 2.Function Descriptions

### 2.1.Auto complete
#### 2.1.1.Detailed description
##### 1> Input and Return Value
Input: the partial name of the location (std::string name)  
Return value: a list of possible locations with partial name as prefix (std::vector<std::string>)

##### 2> Boundary Conditions Check

##### 3> Implementation method

#### 2.1.2.Time Complexity Analysis

#### 2.1.3.Time Spent
Time taken by function: 0 ms
#### 2.1.4 helper functions


### 2.2.GetPosition
#### 2.2.1.Detailed description
##### 1> Input and Return Value
First, the input of this fucntions is location name(std::string type) and the return value is the pair of (latitude, longitude)(std::pair<double, double> type).
##### 2> Boundary Conditions Check
In this function, the input is location name, so we need to check if the location name exists. So we check the return value of GetID to check if the input value is correct. If it isn't correct, the function would return (-1, -1).
##### 3> Implementation method
For this function, I used data maps and a helper function. Because the input value is location name but the key of data map is id, so first I call the GetID function to get the id through the location name. Then, I use id as the index to get the node. Through the attribute in the node we can get latitude and longitude of this location name. Then return the pair of (latitude, longitude) as the return value of this function.
#### 2.2.2.Time Complexity Analysis
For this function, the helper function GetID contribute the main run time comlplexity. The complexity for GetID is the complexity of searching the key in a unordered_map, it is O(1), and the other operation of GetPosition is constant time complexity, so the time complexity is O(1).
#### 2.2.3.Time Spent
Time taken by function: 0 ms
#### 2.2.4 helper functions
Under this step, we have created a lot of helper funtions to get the attributes of the node, Including GetLat, GetLon, GetName, GetNeighborIDs, GetID. Except GetID, the rest of them are all take ID of the node and return the node's attributes. I just use their id to index the node in data map, and return the attributes value of the node. For all of them we have checked if the id exists in the map. For these functions they only take O(1) to find the id in the map. While for GetID, it takes the location name and return ID, so we need to iterate the data map to find which node's name satisfy the requirement. So it would take O(N). So to avoid iteratint the map every time that we called the GetID. We create a unordered_map data structure called name_map, the key is name and the value is id. It will create when constructing the Trojan class objects. So our GetID function can index the name in O(1) time complexity.

### 2.3.EditDistance
