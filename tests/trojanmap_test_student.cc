#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"
// #include "src/lib/mapui.h"


TEST(TrojanMapStudentTest, GetLat) {
  TrojanMap map;
  EXPECT_EQ(34.0064023, map.GetLat("358791610"));
  EXPECT_EQ(34.0125138, map.GetLat("358791878"));
  EXPECT_EQ(-1, map.GetLat("666"));
}

TEST(TrojanMapStudentTest, GetLon) {
  TrojanMap map;
  EXPECT_EQ(-118.2995193, map.GetLon("358791610"));
  EXPECT_EQ(-118.2578516, map.GetLon("358791878"));
  EXPECT_EQ(-1, map.GetLon("666"));
}

TEST(TrojanMapStudentTest, GetName) {
  TrojanMap map;
  EXPECT_EQ("Starbucks 2", map.GetName("4399693642"));
  EXPECT_EQ("Expo Park/USC 1", map.GetName("4399693643"));
  EXPECT_EQ("NULL", map.GetName("666"));
}

TEST(TrojanMapStudentTest, GetNeighborIDs) {
  TrojanMap map;
  std::vector<std::string> ans1 = {"6808200836"};
  std::vector<std::string> ans2 = {"4380040158", "4380040153", "8530535241", "4380040156"};
  std::vector<std::string> ans3;
  EXPECT_EQ(ans1, map.GetNeighborIDs("4399693642"));
  EXPECT_EQ(ans2, map.GetNeighborIDs("4380040157"));
  EXPECT_EQ(ans3, map.GetNeighborIDs("666"));
}

TEST(TrojanMapStudentTest, GetID) {
  TrojanMap map;
  EXPECT_EQ("358789632", map.GetID("National Schools"));
  EXPECT_EQ("358791610", map.GetID("Saint Cecilia School"));
  EXPECT_EQ("", map.GetID("la"));
}

TEST(TrojanMapStudentTest, GetPosition) {
  TrojanMap map;
  std::pair<double, double> ans1 = {34.0105691, -118.2820189};
  std::pair<double, double> ans2 = {34.0064023, -118.2995193};
  std::pair<double, double> ans3 = {-1, -1};
  EXPECT_EQ(ans1, map.GetPosition("National Schools"));
  EXPECT_EQ(ans2, map.GetPosition("Saint Cecilia School"));
  EXPECT_EQ(ans3, map.GetPosition("la"));
}

TEST(TrojanMapStudentTest, inSquare) {
  TrojanMap map;
  std::vector<double> square = {-118.299, -118.264, 34.032, 34.011};
  EXPECT_EQ(false, map.inSquare("358786032", square));
  EXPECT_EQ(true, map.inSquare("354063330",square));
  EXPECT_EQ(false, map.inSquare("348129362", square));
  EXPECT_EQ(false, map.inSquare("666",square));
}

TEST(TrojanMapStudentTest, GetSubgraph) {
  TrojanMap map;
  std::vector<double> square = {-118.2717843, -118.2717841, 34.0381882, 34.0381880};
  std::vector<double> square2 = {-118.2717840, -118.2717841, 34.0381882, 34.0381880};
  std::vector<std::string> ans1 = {"123067117"};
  std::vector<std::string> ans2;
  EXPECT_EQ(ans1, map.GetSubgraph(square));
  EXPECT_EQ(ans2, map.GetSubgraph(square2));
}

TEST(TrojanMapStudentTest, ReadLocationsFromCSVFile) {
  TrojanMap map;
  std::vector<std::string> ans = {"Ralphs", "KFC", "Chick-fil-A"};
  std::string locations_filename = "/home/ee538/Desktop/EE538_HW/final-project-Dylan-JD/input/topologicalsort_locations.csv";
  EXPECT_EQ(ans, map.ReadLocationsFromCSVFile(locations_filename));
}

TEST(TrojanMapStudentTest, ReadLocationsFromCSVFile_error) {
  TrojanMap map;
  std::vector<std::string> ans = {};
  std::string locations_filename = "/home/ee538/Desktop/EE538_HW/final-project-Dylan-JD/input/error.csv";
  EXPECT_EQ(ans, map.ReadLocationsFromCSVFile(locations_filename));
}

TEST(TrojanMapStudentTest, ReadDependenciesFromCSVFile) {
  TrojanMap map;
  std::vector<std::string> str1 = {"Ralphs","Chick-fil-A"};
  std::vector<std::string> str2 = {"Ralphs","KFC"};
  std::vector<std::string> str3 = {"Chick-fil-A","KFC"};
  std::vector<std::vector<std::string>>  ans = {str1, str2, str3};
  std::string dependencies_filename = "/home/ee538/Desktop/EE538_HW/final-project-Dylan-JD/input/topologicalsort_dependencies.csv";
  EXPECT_EQ(ans, map.ReadDependenciesFromCSVFile(dependencies_filename));
}

TEST(TrojanMapStudentTest, ReadDependenciesFromCSVFile_error) {
  TrojanMap map;
  std::vector<std::vector<std::string>>  ans = {};
  std::string dependencies_filename = "/home/ee538/Desktop/EE538_HW/final-project-Dylan-JD/input/error.csv";
  EXPECT_EQ(ans, map.ReadDependenciesFromCSVFile(dependencies_filename));
}

TEST(TrojanMapStudentTest, TopologicalSort) {
  TrojanMap m;
  
  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","KFC"}, {"Ralphs","Chick-fil-A"}, {"Chick-fil-A", "Ralphs"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt;
  EXPECT_EQ(result, gt);
  std::vector<std::string> location_names2 = {"Ralphs", "Chick-fil-A", "KFC", "Arco", "Trojan Grounds (Starbucks)"};
  std::vector<std::vector<std::string>> dependencies2 = {{"Ralphs","KFC"}, {"Ralphs","Chick-fil-A"}, {"Chick-fil-A", "KFC"}};
  auto result2 = m.DeliveringTrojan(location_names2, dependencies2);
  std::vector<std::string> gt2 = {"Trojan Grounds (Starbucks)", "Arco", "Ralphs", "Chick-fil-A", "KFC" };
  EXPECT_EQ(result2, gt2);
  std::vector<std::vector<std::string>> dependencies3 = {{"Ralphs","KFC"}, {"Ralphs","Chick-fil-A"}, 
                                                        {"Chick-fil-A", "KFC"}, {"KFC", "Arco"}, {"Arco", "Trojan Grounds (Starbucks)"}};
  auto result3 = m.DeliveringTrojan(location_names2, dependencies3);
  std::vector<std::string> gt3 = {"Ralphs", "Chick-fil-A", "KFC", "Arco", "Trojan Grounds (Starbucks)"};
  EXPECT_EQ(result3, gt3);
  std::vector<std::string> empty;
  std::vector<std::vector<std::string>> empty_dependencies;
  auto result4 = m.DeliveringTrojan(empty, dependencies3);
  std::vector<std::string> gt4;
  EXPECT_EQ(result4, gt4);
  auto result5 = m.DeliveringTrojan(location_names2, empty_dependencies);
  std::vector<std::string> gt5 = {"Trojan Grounds (Starbucks)", "Arco", "KFC", "Chick-fil-A", "Ralphs"};
  EXPECT_EQ(result5, gt5);
  auto result6 = m.DeliveringTrojan(empty, empty_dependencies);
  EXPECT_EQ(result6, gt4);
  auto result7 = m.DeliveringTrojan(location_names, dependencies3);
  EXPECT_EQ(result7, gt4);
}

TEST(TrojanMapStudentTest, CalculateShortestPath_Dijkstra) {
  TrojanMap m;
  std::vector<std::string> gt;
  auto result1 = m.CalculateShortestPath_Dijkstra("error", "error");
  EXPECT_EQ(result1, gt);
  auto result2 = m.CalculateShortestPath_Dijkstra("leavey library", "error");
  EXPECT_EQ(result2, gt);
  auto result3 = m.CalculateShortestPath_Dijkstra("Leavey Library", "dulce");
  EXPECT_EQ(result3, gt);
}

TEST(TrojanMapStudentTest, CalculateShortestPath_Bellman_Ford) {
  TrojanMap m;
  std::vector<std::string> gt;
  auto result1 = m.CalculateShortestPath_Bellman_Ford("error", "error");
  EXPECT_EQ(result1, gt);
  auto result2 = m.CalculateShortestPath_Bellman_Ford("leavey library", "error");
  EXPECT_EQ(result2, gt);
  auto result3 = m.CalculateShortestPath_Bellman_Ford("Leavey Library", "error");
  EXPECT_EQ(result3, gt);
}

TEST(TrojanMapStudentTest, CycleDetection) {
  TrojanMap m;
  std::vector<std::string> gt;
  std::vector<double> square1 = {-118.296, -118.263, 34.034, 34.009};
  std::vector<double> square2 = {-118.290, -118.289, 34.028, 34.022};
  std::vector<double> square3;
  std::vector<std::string> sub3;
  auto sub1 = m.GetSubgraph(square1);
  auto sub2 = m.GetSubgraph(square2);
  auto result1 = m.CycleDetection(sub1, square1);
  auto result2 = m.CycleDetection(sub2, square2);
  EXPECT_EQ(result1, true);
  EXPECT_EQ(result2, false);
  auto result3 = m.CycleDetection(sub2, square3);
  EXPECT_EQ(result3, false);
  auto result4 = m.CycleDetection(sub3, square1);
  EXPECT_EQ(result4, false);
}




