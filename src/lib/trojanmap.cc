#include "trojanmap.h"

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string& id) {
  if(data.count(id) == 0){
    return -1;
  }
  Node node = data[id];

  return node.lat;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string& id) { 
  if(data.count(id) == 0){
    return -1;
  }
  Node node = data[id];

  return node.lon;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return "NULL".
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string& id) { 
  if(data.count(id) == 0){
    return "NULL";
  }
  Node node = data[id];

  return node.name;
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return an empty vector.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string& id) {
  if(data.count(id) == 0){
    return {};
  }
  Node node = data[id];
  return node.neighbors;
}

/**
 * GetID: Given a location name, return the id. 
 * If the node does not exist, return an empty string. 
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */
std::string TrojanMap::GetID(const std::string& name) {
  if(name_map.count(name) == 0)
    return "";
  else
    return name_map[name];
}

/**
 * GetPosition: Given a location name, return the position. If id does not exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::string node_id = GetID(name);
  std::pair<double, double> results;
  if(node_id == ""){
    results.first = -1;
    results.second = -1;
  }
  else{
    results.first = data[node_id].lat;
    results.second = data[node_id].lon;
  }
  return results;
}


/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * 
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b){
    int n = a.length();
    int m = b.length();
    if(n == 0) return m;
    if(m == 0) return n;

    std::vector<std::vector<int>> dp (n+1, std::vector<int>(m+1, INT_MAX));

    for(int i = 0; i <= n; ++i){
        for(int j = 0; j<= m; ++j){
            if( i == 0){
                dp[i][j] = j;
            }else if(j == 0){
                dp[i][j] = i;
            }else{
                if(a[i-1] == b[j-1]){
                    dp[i][j] = 1 + std::min(dp[i-1][j-1]-1, std::min(dp[i-1][j], dp[i][j-1]));
                }else{
                    dp[i][j] = 1 + std::min(dp[i-1][j-1], std::min(dp[i-1][j], dp[i][j-1]));
                }
            }
        }
    }
    return dp[n][m];
}

/**
 * FindClosestName: Given a location name, return the name with smallest edit distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::string tmp = "";
  int edit_distance = INT_MAX;
  for (auto iter = data.begin(); iter != data.end(); ++iter) {
    std::string name_node = iter->second.name;
    if(name_node.size() != 0){
      int distance = CalculateEditDistance(name, name_node);
      if( distance < edit_distance){
        edit_distance = distance;
        tmp = name_node;
      }
    }
  }

  return tmp;
}


/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results;
  if(name.empty()) return results;
  while(name[name.size()-1] == ' '){
    name.erase(name.size()-1);
  }

  std::string name_tolower = name;
  for (std::string::iterator it = name_tolower.begin(); it != name_tolower.end(); ++it)
  {
    *it = tolower(*it);
  }

  for (auto iter = data.begin(); iter != data.end(); ++iter) {
    std::string name_node = iter->second.name;
    if(name_node.size() >= name_tolower.size()){
      for(long unsigned int i = 0; i < name_tolower.size(); ++i){
        name_node[i] = tolower(name_node[i]);
      }
      if(name_node.find(name_tolower,0) == 0){
        results.push_back(iter->second.name);
      }
    }
  }
  return results;
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id, const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0;i < int(path.size())-1; i++) {
    sum += CalculateDistance(path[i], path[i+1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;
  std::unordered_map<std::string, bool> visited;
  std::unordered_map<std::string, std::pair<double, std::string>> d;
  std::unordered_map<std::string, std::pair<double, std::string>> unvisited_node;
  std::queue<std::string> node_queue;
  std::stack<std::string> node_stack;
  std::string start_id = GetID(location1_name);
  std::string end_id = GetID(location2_name);
  if(start_id == "" || end_id == ""){
    return {};
  }
  node_queue.push(start_id);
  d[start_id] = std::make_pair(0, "start");
  visited[start_id] = true;
  while(!node_queue.empty()){
    std::string node_id = node_queue.front();
    double min_dis = INT_MAX;
    std::string min_node_id = "NULL";
    for(auto n : GetNeighborIDs(node_id)){
      if(visited.count(n) == 0){
        double new_dis = d[node_id].first + CalculateDistance(node_id, n);
        if(d.count(n) == 0){
          d[n] = std::make_pair(new_dis, node_id);
        }
        else if(d[n].first > new_dis){
          d[n] = std::make_pair(new_dis, node_id);
        }
        unvisited_node[n] = d[n];
      }
    }
    std::unordered_map<std::string, std::pair<double, std::string>>::iterator iter;
    iter = unvisited_node.begin();
    while(iter != unvisited_node.end()){
      if(visited.count(iter->first) == 0){
        if(iter->second.first < min_dis){
          min_node_id = iter->first;
          min_dis = iter->second.first;
        }
      }
      iter++;
    }

    if(min_node_id != "NULL" && visited.count(min_node_id) == 0){
      std::cout<<min_node_id<<std::endl;
      visited[min_node_id] = true;
      unvisited_node.erase(min_node_id);
      node_queue.push(min_node_id);
    }
    node_queue.pop();
  }
  std::string node_id;
  node_stack.push(end_id);
  while(node_id != start_id){
    node_id = node_stack.top();
    if(node_id == start_id) break;
    node_stack.push(d[node_id].second);
  }

  while(!node_stack.empty()){
    path.push_back(node_stack.top());
    node_stack.pop();
  }
  
  return path;
}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Do the early termination when there is no change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name){
  std::vector<std::string> path;
  std::unordered_map<std::string, std::pair<double, std::string>> d;
  std::queue<std::string> node_queue;
  std::unordered_map<std::string, bool> visited;
  std::stack<std::string> node_stack;

  std::string start_id = GetID(location1_name);
  std::string end_id = GetID(location2_name);
  if(start_id == "" || end_id == ""){
    return {};
  }
  d[start_id] = std::make_pair(0, "strat");
  for(long unsigned int i = 0; i < data.size(); i++){
    visited.clear();
    visited[start_id] = true;
    int count = 0;
    node_queue.push(start_id);
    bool flag = false;
    while(!node_queue.empty()){
      count++;
      std::string node_id = node_queue.front();
      for(auto n : GetNeighborIDs(node_id)){
        double new_dis = d[node_id].first + CalculateDistance(node_id, n);
        if(d.count(n) == 0){
          flag = true;
          d[n] = std::make_pair(new_dis, node_id);
        }
        else if(d[n].first > new_dis){
          flag = true;
          d[n] = std::make_pair(new_dis, node_id);
        }

        if(visited.count(n) == 0){
          node_queue.push(n);
          visited[n] = true;
        }
      }
      node_queue.pop();

    }
    if(flag == false){
      break;
    }
  }

  std::string node_id;
  node_stack.push(end_id);
  while(node_id != start_id){
    node_id = node_stack.top();
    if(node_id == start_id) break;
    node_stack.push(d[node_id].second);
  }

  while(!node_stack.empty()){
    path.push_back(node_stack.top());
    node_stack.pop();
  }

  return path;
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  sort(location_ids.begin()+1, location_ids.end());

  auto temp = location_ids;            // temp is added the the first id, to form a cycle
  temp.push_back(location_ids[0]);
  double min_distance = CalculatePathLength(temp);
  std::vector<std::string> min_ids = temp; 

  while(std::next_permutation(location_ids.begin()+1, location_ids.end())){
    temp = location_ids;         // temp is added the the first id, to form a cycle
    temp.push_back(location_ids[0]);
    double distance = CalculatePathLength(temp);

    if(min_distance > distance ){
      records.first = min_distance;
      records.second.push_back(min_ids);
      min_distance = distance;
      min_ids = temp;
    }else{
      records.second.push_back(temp);
    }
    
  }
  records.first = min_distance;
  records.second.push_back(min_ids);
  return records;
}

void TrojanMap::backTracking_helper(std::vector<std::string>& location_ids, 
                                    std::pair<double, std::vector<std::vector<std::string>>> & records,
                                    int level, double& min_distance, std::vector<std::string>& min_ids){
  if(level == location_ids.size()){
    auto temp = location_ids;         // temp is added the the first id, to form a cycle
    temp.push_back(location_ids[0]);
    double distance = CalculatePathLength(temp);
    
    if(min_distance > distance ){
      if(!min_ids.empty())
        records.second.push_back(min_ids);
      min_distance = distance;
      min_ids = temp;
    }else{
      if(!min_ids.empty())
        records.second.push_back(temp);
    }
    return ;
  }
  
  for(int i = level; i < location_ids.size(); ++i){
    swap(location_ids[i],location_ids[level]);
    backTracking_helper(location_ids,records,level+1, min_distance, min_ids);
    swap(location_ids[i],location_ids[level]);
  }
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  records.first = DBL_MAX;
  double min_distance = DBL_MAX;
  std::vector<std::string> min_ids;
  backTracking_helper(location_ids,records,0, min_distance, min_ids);
  records.first = min_distance;
  records.second.push_back(min_ids);
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  std::vector<std::string> location_names_from_csv;
  std::fstream fin;
  fin.open(locations_filename, std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    location_names_from_csv.push_back(line);
  }
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  std::vector<std::vector<std::string>> dependencies_from_csv;
  std::fstream fin;
  fin.open(dependencies_filename, std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);
    std::vector<std::string> single_line;
    while(getline(s, word, ',')){
      single_line.push_back(word);
    }
    dependencies_from_csv.push_back(single_line);
  }
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies. If there is no way to do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::unordered_map<std::string, std::vector<std::string>> adj;
  for(auto n : locations){
    std::vector<std::string> temp;
    adj[n] = temp;
  }

  for(auto n:dependencies){
    adj[n[0]].push_back(n[1]);
  }
  std::unordered_map<std::string, std::vector<std::string>>::iterator iter = adj.begin();
  std::unordered_map<std::string, bool> visited;
  std::queue<std::string> bfs_queue;
  std::vector<std::string> result;
  std::unordered_map<std::string, int> indegree;
  std::unordered_map<std::string, int>::iterator degree_iter;
  
  for(auto n : locations){
    indegree[n] = 0;
  }
  
  while(iter != adj.end()){
    for(auto n: iter->second){
        indegree[n]++;
    }
    iter++;
  }

  degree_iter = indegree.begin();
  while(degree_iter != indegree.end()){
    if(degree_iter->second == 0){
      bfs_queue.push(degree_iter->first);
    }
    degree_iter++;
  }

  while(!bfs_queue.empty()){
    std::string name = bfs_queue.front();
    bfs_queue.pop();
    result.push_back(name);
    for(auto n : adj[name]){
      indegree[n]--;
      if(indegree[n] == 0){
        bfs_queue.push(n);
      }
    }
  }

  if(result.size() == locations.size()){
    return result;
  }
  else{
    return {};
  }                                          
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  double lat = GetLat(id);
  double lon = GetLon(id);
  if(lat == -1 && lon == -1){
    return false;
  }
  if(lon < square[0]){
    return false;
  }
  else if(lon > square[1]){
    return false;
  }
  else if(lat > square[2]){
    return false;
  }
  else if(lat < square[3]){
    return false;
  }
  return true;
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;
  std::unordered_map<std::string, Node>::iterator iter;
  iter = data.begin();
  while(iter != data.end()){
    if(inSquare(iter->second.id, square) == true){
      subgraph.push_back(iter->second.id);
    }
    iter++;
  }
  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
  /*visited map: didn't exsited key means unvisited, true means visiting, false means visited*/
  std::unordered_map<std::string, bool> visited;
  for(auto i : subgraph){
    if(visited.count(i) == 0){
      if (CycleDetection_Helper(i, "-1", square, visited) == true){
        return true;
      }
    }
    std::cout<<visited[i]<<std::endl;
  }
  return false;
}

bool TrojanMap::CycleDetection_Helper(std::string node_id, std::string parent_id, 
        std::vector<double> &square, std::unordered_map<std::string, bool> &visited) {
    std::vector<std::string> neighbor = GetNeighborIDs(node_id);
    visited[node_id] = true; 
    for(auto n : neighbor){
      if(n == parent_id){
        continue;
      }
      if(inSquare(n, square) == true){
        if(visited.count(n) == 0){
          if(CycleDetection_Helper(n, node_id, square, visited) == true){
              std::cout<<"if 1"<<std::endl;
              return true;
          }
        }
        else if(visited[n] == true){
          std::cout<<"if 2"<<std::endl;
          std::cout<<node_id<<std::endl;
          std::cout<<n<<std::endl;
          return true;
        }
      }
      visited[node_id] = false;
    }
  return false;
}

/**
 * FindNearby: Given a class name C, a location name L and a number r, 
 * find all locations in class C on the map near L with the range of r and return a vector of string ids
 * 
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {int} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;
  return res;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0]))
          n.attributes.insert(word);
        if (isdigit(word[0]))
          n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}


//Initialize name_map;
//Get id according to the name: O(1)
void TrojanMap::InitNameMap(){
  std::unordered_map<std::string, Node>::iterator iter;
  iter = data.begin();
  while(iter != data.end()){
    if(!iter->second.name.empty()){
      name_map[iter->second.name] = iter->second.id;
    }
    iter++;
  }
}