#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"


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











