#include <assert.h>
#include <stdio.h>
#include <queue>
#include <random>
#include "maze.h"
using namespace navigation;

int main() {
    Maze maze;
    maze.reset();
    for (int i = 0; i < size; ++i) {
        assert(!maze.canTravel(i, 0, 2, false));
        assert(!maze.canTravel(i, size - 1, 0, false));
        assert(!maze.canTravel(0, i, 3, false));
        assert(!maze.canTravel(size - 1, i, 1, false));
    }
    maze.observe(0, 0, 2, false);
    assert(!maze.canTravel(0, 0, 2, false));
    assert(!maze.canTravel(-1, 0, 0, false));
    assert(!maze.canTravel(0, 0, 4, false));
    Path path;
    assert(!maze.buildPath(Pose{}, false, path));
    maze.flood(false, false);
    assert(maze.nextDirection(Pose{}, false) == 0);
    maze.observe(0, 0, 0, true);
    assert(!maze.canTravel(0, 1, 2, false));
    maze.flood(false, false);
    assert(maze.nextDirection(Pose{}, false) == 1);
    maze.observe(0, 0, 1, true);
    maze.flood(false, false);
    assert(maze.nextDirection(Pose{}, false) == -1);
    maze.reset();
    Pose p;
    for (int i = 0; i < 7; ++i) maze.observe(0, i, 0, false);
    for (int i = 0; i < 7; ++i) maze.observe(i, 7, 1, false);
    assert(maze.buildPath(p, false, path) && path.length == 14);
    p.x = p.y = 7;
    assert(maze.buildPath(p, true, path) && path.length == 14);
    for (int x = 7; x <= 8; ++x) for (int y = 7; y <= 8; ++y) {
        p.x = x; p.y = y;
        assert(maze.buildPath(p, false, path) && path.length == 0);
    }
    // A single corridor can visit every cell before returning to its origin.
    maze.reset();
    for (int x = 0; x < size; ++x) for (int y = 0; y < size; ++y) {
        if (x < size - 1) maze.observe(x, y, 1, false);
        if (y < size - 1 && x == ((y % 2 == 0) ? size - 1 : 0)) maze.observe(x, y, 0, false);
    }
    p.x = 0; p.y = 15;
    assert(maze.buildPath(p, true, path) && path.length == 255);

    std::mt19937 random(42);
    for (int trial = 0; trial < 200; ++trial) {
        maze.reset();
        bool open[size][size][4] = {};
        for (int x = 0; x < size; ++x) for (int y = 0; y < size; ++y) {
            for (int d = 0; d <= 1; ++d) {
                const int nx = x + dx[d], ny = y + dy[d];
                if (!Maze::contains(nx, ny)) continue;
                const int observation = random() % 3;
                if (observation == 0) continue;
                maze.observe(x, y, d, observation == 1);
                if (observation == 2) open[x][y][d] = open[nx][ny][(d + 2) & 3] = true;
            }
        }
        int expected[size][size];
        std::queue<int> queue;
        for (int x = 0; x < size; ++x) for (int y = 0; y < size; ++y) {
            expected[x][y] = -1;
            if (Maze::isGoal(x, y)) { expected[x][y] = 0; queue.push(x * size + y); }
        }
        while (!queue.empty()) {
            int cell = queue.front(); queue.pop();
            int x = cell / size, y = cell % size;
            for (int d = 0; d < 4; ++d) if (open[x][y][d]) {
                int nx = x + dx[d], ny = y + dy[d];
                if (expected[nx][ny] < 0) {
                    expected[nx][ny] = expected[x][y] + 1;
                    queue.push(nx * size + ny);
                }
            }
        }
        for (int x = 0; x < size; ++x) for (int y = 0; y < size; ++y) {
            p.x = x; p.y = y;
            bool found = maze.buildPath(p, false, path);
            assert(found == (expected[x][y] >= 0));
            if (!found) { assert(path.length == 0); continue; }
            assert(path.length == expected[x][y]);
            for (int i = 0; i < path.length; ++i) {
                int d = path.directions[i];
                assert(open[p.x][p.y][d]);
                p.x += dx[d]; p.y += dy[d];
            }
            assert(Maze::isGoal(p.x, p.y));
        }
    }
    puts("Navigation tests passed: borders, wall updates, four goals, known routes, 255-step path, 51,200 randomized routes.");
}
