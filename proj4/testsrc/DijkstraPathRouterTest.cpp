#include "DijkstraPathRouter.h"
#include <gtest/gtest.h>
#include <any>
#include <string>

// Test fixture for CDijkstraPathRouter
class DijkstraPathRouterTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize the router for each test
        router = std::make_unique<CDijkstraPathRouter>();
    }

    void TearDown() override {
        // Clean up after each test
        router.reset();
    }

    std::unique_ptr<CDijkstraPathRouter> router;
};

// Test adding vertices
TEST_F(DijkstraPathRouterTest, AddVertex) {
    auto v1 = router->AddVertex(std::string("A"));
    auto v2 = router->AddVertex(std::string("B"));

    EXPECT_EQ(router->VertexCount(), 2);

    // Extract the tag from std::any and compare
    EXPECT_EQ(std::any_cast<std::string>(router->GetVertexTag(v1)), "A");
    EXPECT_EQ(std::any_cast<std::string>(router->GetVertexTag(v2)), "B");
}

// Test adding edges
TEST_F(DijkstraPathRouterTest, AddEdge) {
    auto v1 = router->AddVertex(std::string("A"));
    auto v2 = router->AddVertex(std::string("B"));

    EXPECT_TRUE(router->AddEdge(v1, v2, 1.0));
    EXPECT_FALSE(router->AddEdge(v1, v2, -1.0)); // Negative weight should fail
}

// Test finding the shortest path
TEST_F(DijkstraPathRouterTest, FindShortestPath) {
    auto v1 = router->AddVertex(std::string("A"));
    auto v2 = router->AddVertex(std::string("B"));
    auto v3 = router->AddVertex(std::string("C"));

    router->AddEdge(v1, v2, 1.0);
    router->AddEdge(v2, v3, 2.0);
    router->AddEdge(v1, v3, 4.0);

    std::vector<CPathRouter::TVertexID> path;
    double distance = router->FindShortestPath(v1, v3, path);

    EXPECT_EQ(distance, 3.0); // Shortest path distance should be 3.0
    EXPECT_EQ(path.size(), 3); // Path should contain 3 vertices
    EXPECT_EQ(path[0], v1);    // Path should start at v1
    EXPECT_EQ(path[1], v2);    // Path should go through v2
    EXPECT_EQ(path[2], v3);    // Path should end at v3
}

// Test no path exists
TEST_F(DijkstraPathRouterTest, NoPathExists) {
    auto v1 = router->AddVertex(std::string("A"));
    auto v2 = router->AddVertex(std::string("B"));
    auto v3 = router->AddVertex(std::string("C"));

    router->AddEdge(v1, v2, 1.0); // No edge between v2 and v3

    std::vector<CPathRouter::TVertexID> path;
    double distance = router->FindShortestPath(v1, v3, path);

    EXPECT_EQ(distance, CDijkstraPathRouter::NoPathExists); // No path should exist
    EXPECT_TRUE(path.empty()); // Path should be empty
}