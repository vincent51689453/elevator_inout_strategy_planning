// C++ STL
#include <iostream>
#include <string>
#include <vector>

// A class to store 'point' in PCL
class obstacle_point{
    private:
        double x;
        double y;
        double z;
    
    public:
        double read_x() {return x;}
        double read_y() {return y;}
        double read_z() {return z;}
        void set_x(int i) {x = i;}
        void set_y(int j) {y = j;}
        void set_z(int k) {z = k;}
};

// A vector contains multiple obstacle_points
std::vector<int> obstacle_cloud;