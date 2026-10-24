#pragma once
#include <stdint.h>

namespace navigation {
constexpr int size = 16, cells = size * size;
constexpr uint16_t unreachable = 0xffff;
constexpr int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
struct Pose { int x = 0, y = 0, heading = 0; };
struct Path {
    uint8_t directions[cells - 1] = {};
    uint16_t length = 0;
};

class Maze {
public:
    static bool contains(int x, int y) {
        return x >= 0 && x < size && y >= 0 && y < size;
    }
    static bool isGoal(int x, int y) {
        return (x == size / 2 - 1 || x == size / 2) &&
               (y == size / 2 - 1 || y == size / 2);
    }
    void reset() {
        for (int x = 0; x < size; ++x)
            for (int y = 0; y < size; ++y) walls_[x][y] = known_[x][y] = 0;
        for (int i = 0; i < size; ++i) {
            observe(i, 0, 2, true); observe(i, size - 1, 0, true);
            observe(0, i, 3, true); observe(size - 1, i, 1, true);
        }
    }
    void observe(int x, int y, int d, bool wall) {
        if (!contains(x, y) || d < 0 || d > 3) return;
        const int nx = x + dx[d], ny = y + dy[d];
        if (!contains(nx, ny)) wall = true;
        mark(x, y, d, wall);
        if (contains(nx, ny)) mark(nx, ny, (d + 2) & 3, wall);
    }
    bool canTravel(int x, int y, int d, bool knownOnly) const {
        if (!contains(x, y) || d < 0 || d > 3) return false;
        const uint8_t bit = 1u << d;
        return contains(x + dx[d], y + dy[d]) && !(walls_[x][y] & bit) &&
               (!knownOnly || (known_[x][y] & bit));
    }
    void flood(bool toStart, bool knownOnly) {
        uint16_t queue[cells];
        int head = 0, tail = 0;
        for (int x = 0; x < size; ++x) {
            for (int y = 0; y < size; ++y) {
                distance_[x][y] = unreachable;
                if (toStart ? (x == 0 && y == 0) : isGoal(x, y)) {
                    distance_[x][y] = 0;
                    queue[tail++] = x * size + y;
                }
            }
        }
        while (head < tail) {
            const int cell = queue[head++], x = cell / size, y = cell % size;
            for (int d = 0; d < 4; ++d) {
                if (!canTravel(x, y, d, knownOnly)) continue;
                const int nx = x + dx[d], ny = y + dy[d];
                if (distance_[nx][ny] != unreachable) continue;
                distance_[nx][ny] = distance_[x][y] + 1;
                queue[tail++] = nx * size + ny;
            }
        }
    }
    int nextDirection(const Pose &pose, bool knownOnly) const {
        if (!contains(pose.x, pose.y) || pose.heading < 0 || pose.heading > 3 ||
            distance_[pose.x][pose.y] == unreachable || distance_[pose.x][pose.y] == 0) return -1;
        const int preference[] = {pose.heading, (pose.heading + 3) & 3,
                                  (pose.heading + 1) & 3, (pose.heading + 2) & 3};
        for (int d : preference) {
            if (canTravel(pose.x, pose.y, d, knownOnly) &&
                distance_[pose.x + dx[d]][pose.y + dy[d]] + 1 == distance_[pose.x][pose.y]) return d;
        }
        return -1;
    }
    bool buildPath(Pose pose, bool toStart, Path &path) {
        path.length = 0;
        if (!contains(pose.x, pose.y) || pose.heading < 0 || pose.heading > 3) return false;
        flood(toStart, true);
        if (distance_[pose.x][pose.y] == unreachable) return false;
        while (distance_[pose.x][pose.y] != 0) {
            const int d = nextDirection(pose, true);
            if (d < 0 || path.length >= cells - 1) { path.length = 0; return false; }
            path.directions[path.length++] = static_cast<uint8_t>(d);
            pose.x += dx[d]; pose.y += dy[d]; pose.heading = d;
        }
        return true;
    }
private:
    uint8_t walls_[size][size] = {}, known_[size][size] = {};
    uint16_t distance_[size][size] = {};
    void mark(int x, int y, int d, bool wall) {
        const uint8_t bit = 1u << d;
        known_[x][y] |= bit;
        // A detected wall stays closed until the map is reset.
        if (wall) walls_[x][y] |= bit;
    }
};
}
