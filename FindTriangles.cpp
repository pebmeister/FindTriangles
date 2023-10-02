// ReSharper disable CppInconsistentNaming
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>
#include <chrono>

using namespace std;

// Margin of error for comparing floats
static constexpr double compare_tolerance = .00001;

// Define a point structure
// with floats for x and y
// override the == operator to compare to another point
typedef struct point
{
    float x;
    float y;

    point(const float x, const float y)
        : x(x),
        y(y)
    {}

    bool operator==(const point& other) const
    {
        return abs(x - other.x) < compare_tolerance && abs(y - other.y) < compare_tolerance;
    }
} point;


// Define a line segment structure as 2 points
typedef struct line_segment
{
    point p1;
    point p2;

    line_segment(const float a, const float b, const float c, const float d)
        : p1(a, b),
        p2(c, d)
    {}

    line_segment(const point& p1, const point& p2)
        : p1(p1),
        p2(p2)
    {}
} line_segment;

// define a triangle structure as 3 points
typedef struct triangle
{
    point p1;
    point p2;
    point p3;

    triangle(const point& p1, const point& p2, const point& p3)
        : p1(p1),
        p2(p2),
        p3(p3)
    {}
} triangle;

// determine if a given point is contained in a vector of points
bool find_point(vector<point>& points, const point& pt)
{
    return find(points.begin(), points.end(), pt) != points.end();
}

// calculate the intersection of 2 line segments
// segment 1 = points A and B
// segment 2 = points C and D
// if there is an intersection return the point in pt
// from https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
//
//      /      \     /         \          /      \     /         \
//      |  x1  |     | x2 - x1 |          |  x3  |     | x4 - x3 |
// L1 = |  --  | + t | ------- |,    L2 = |  --  | + u | ------- |
//      |  y1  |     | y2 - y1 |          |  y3  |     | y4 - y3 |
//      \      /     \         /          \      /     \         /
//
//
//      |  x1 - x3   x3 - x4  |
//      |  y1 - y3   y3 - y4  |     (x1 - x3)(y3 - y4) - (y1 - y3)(x3 - x4)
// t  = -----------------------  =  ---------------------------------------
//      |  x1 - x2   x3 - x4  |     (x1 - x2)(y3 - y4) - (y1 - y2)(x3 - x4)
//      |  y1 - y2   y3 - y4  |
//
//
//      |  x1 - x3   x1 - x2  |
//      |  y1 - y3   y1 - y2  |     (x1 - x3)(y1 - y2) - (y1 - y3)(x1 - x2)
// u  = -----------------------  =  ---------------------------------------
//      |  x1 - x2   x3 - x4  |     (x1 - x2)(y3 - y4) - (y1 - y2)(x3 - x4)
//      |  y1 - y2   y3 - y4  |
//
// (Px,Py) = (x1 + t(x2 - x1), y1 + t(y2 - y1))
//
// NOTE:
//    denominators are the same for t and u and must not be 0
//    t and u must be in the range 0 to 1
//
bool calc_intersection(const point& A, const point& B, const point& C, const point& D, point& pt)
{
    pt = { 0,0 };

    const auto x1 = A.x;
    const auto y1 = A.y;
    const auto x2 = B.x;
    const auto y2 = B.y;
    const auto x3 = C.x;
    const auto y3 = C.y;
    const auto x4 = D.x;
    const auto y4 = D.y;

    // simplify terms
    const auto x1_x2 = x1 - x2;
    const auto x1_x3 = x1 - x3;
    const auto x2_x1 = x2 - x1;
    const auto x3_x4 = x3 - x4;
    const auto y1_y2 = y1 - y2;
    const auto y1_y3 = y1 - y3;
    const auto y2_y1 = y2 - y1;
    const auto y3_y4 = y3 - y4;

    const auto denominator = x1_x2 * y3_y4 - y1_y2 * x3_x4;
    if (abs(denominator) < compare_tolerance)
        return false;

    const auto t = (x1_x3 * y3_y4 - y1_y3 * x3_x4) / denominator;
    if (t < 0 || t > 1)
        return false;

    const auto u = (x1_x3 * y1_y2 - y1_y3 * x1_x2) / denominator;
    if (u < 0 || u > 1)
        return false;

    pt = point(x1 + t * x2_x1, y1 + t * y2_y1);
    return true;
}

// calculate the intersection of 2 line segments
// given 2 line segments
// if there is an intersection return the point in pt
bool calc_intersection(const line_segment& ls1, const line_segment& ls2, point& pt)
{
    return calc_intersection(ls1.p1, ls1.p2, ls2.p1, ls2.p2, pt);
}

// calculate the intersections of line segments
// given a vector of line segments
// output the intersections in a vector of point vectors
// vector[0] will output a vector of all the intersections in line segment 0
// vector[1] will output a vector of all the intersections in line segment 1
// vector[N] will output a vector of all the intersections in line segment N
void calc_intersections(const vector<line_segment>& segments, vector<vector<point>>& intersects)
{
    for (auto i = 0; i < static_cast<int>(segments.size()) - 1; ++i)
    {
        for (auto j = i + 1; j < static_cast<int>(segments.size()); ++j)
        {
            point intersect_pt(0, 0);
            if (calc_intersection(segments[i], segments[j], intersect_pt))
            {
                if (!find_point(intersects[i], intersect_pt))
                    intersects[i].push_back(intersect_pt);

                if (!find_point(intersects[j], intersect_pt))
                    intersects[j].push_back(intersect_pt);
            }
        }
    }
}

// calculate the triangles with the intersections of line segments
// intersects[0] contains the intersection points for line segment 0
// intersects[1] contains the intersection points for line segment 1
// intersects[N] contains the intersection points for line segment N
void calc_triangles(vector<vector<point>>& intersects, vector<triangle>& triangles)
{
    const int num_line_segments = static_cast<int>(intersects.size());
    for (auto segment_one_index = 0; segment_one_index < num_line_segments - 2; ++segment_one_index)
    {
        for (point& start_point : intersects[segment_one_index])
        {
            for (auto segment_two_index = segment_one_index + 1; segment_two_index < num_line_segments - 1; ++segment_two_index)
            {
                if (!find_point(intersects[segment_two_index], start_point))
                    continue;

                for (point& middle_point : intersects[segment_two_index])
                {
                    if (middle_point == start_point)
                        continue;

                    for (auto segment_three_index = segment_two_index + 1; segment_three_index < num_line_segments; ++segment_three_index)
                    {
                        if (!find_point(intersects[segment_three_index], middle_point))
                            continue;

                        for (point& last_point : intersects[segment_three_index])
                        {
                            if (last_point == middle_point || !find_point(intersects[segment_one_index], last_point))
                                continue;

                            triangles.emplace_back(start_point, middle_point, last_point);
                        }
                    }
                }
            }
        }
    }
}

// calculate the triangles with the intersections of line segments
// calculate the intersection point for the segments
// calculate the triangles given the intersection points
int calc_triangles(const vector<line_segment>& segments, vector<triangle>& triangles)
{
    vector<vector<point>> intersects;
    intersects.resize(segments.size());

    calc_intersections(segments, intersects);
    calc_triangles(intersects, triangles);
    return static_cast<int>(triangles.size());
}

// main entry point
// create line segments
// calculate the triangles
// output results
int main()
{
    vector<triangle> triangles;
    const vector<line_segment> line_segments =
    {
        line_segment(5, 1, 9, 9),
        line_segment(4, 3, 7, 9),
        line_segment(3, 5, 5, 9),
        line_segment(2, 7, 3, 9),

        line_segment(5, 1, 1, 9),
        line_segment(6, 3, 3, 9),
        line_segment(7, 5, 5, 9),
        line_segment(8, 7, 7, 9),

        line_segment(4, 3, 6, 3),
        line_segment(3, 5, 7, 5),
        line_segment(2, 7, 8, 7),
        line_segment(1, 9, 9, 9),
    };

    calc_triangles(line_segments, triangles);

    cout << "Line segments" << endl;
    for (const auto& line_segment : line_segments)
    {
        cout <<
            "(" << setw(3) << line_segment.p1.x << ", " << setw(3) << line_segment.p1.y << "), " <<
            "(" << setw(3) << line_segment.p2.x << ", " << setw(3) << line_segment.p2.y << ")" <<
            endl;
    }
    cout << endl << "Triangles" << endl;
    for (const auto& triangle : triangles)
    {
        cout <<
            "(" << setw(3) << triangle.p1.x << ", " << setw(3) << triangle.p1.y << "), " <<
            "(" << setw(3) << triangle.p2.x << ", " << setw(3) << triangle.p2.y << "), " <<
            "(" << setw(3) << triangle.p3.x << ", " << setw(3) << triangle.p3.y << ")" <<
            endl;
    }
    cout << endl << "There are " << triangles.size() << " triangle(s) found." << endl;
}
