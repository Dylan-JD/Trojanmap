# EE538 Final Project - Spring 2022 - TrojanMap
## 1.Overview

## 2.Function Descriptions

### 1.Auto complete

### 2.GetPosition
#### 2.1.Detailed description
##### 1> Input and Return Value
First, the input of this fucntions is location name(std::string type) and the return value is the pair of (latitude, longitude)(std::pair<double, double> type).
##### 2> Boundary Conditions Check
In this function, the input is location name, so we need to check if the location name exists. So we check the return value of GetID to check if the input value is correct. If it isn't correct, the function would return (-1, -1).
##### 3> Implementation method
For this function, I used data maps and a helper function. Because the input value is location name but the key of data map is id, so first I call the GetID function to get the id through the location name. Then, I use id as the index to get the node. Through the attribute in the node we can get latitude and longitude of this location name. Then return the pair of (latitude, longitude) as the return value of this function.
#### 2.2.Time Complexity Analysis
For this function, the helper function GetID contribute the main run time comlplexity. The complexity for GetID is the complexity of searching the key in a unordered_map, it is O(1), and the other operation of GetPosition is constant time complexity, so the time complexity is O(1).
#### 2.3.Time Spent
Time taken by function: 0 ms

### 3.EditDistance
