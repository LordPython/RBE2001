#pragma once

// A simple list
template<typename T, size_t S>
class List {
public:
    List() : _size(0) {}

    inline size_t size() {
        return _size;
    }

    void insert(T t) {
        if(_size < S) {
            array[_size++] = t;
        } else {
#ifdef DEBUG
            Serial.println("Array overflow");
            Serial.flush();
#endif
        }
    }

    bool remove(T t) {
        for(int i = 0; i < _size; ++i) {
            if(array[i] == t) {
                --_size;
                for (int j = i; j < _size; ++j) {
                    array[j] = array[j+1];
                }
                return true;
            }
        }
        return false;
    }

    bool removeAt(int i) {
        if (i >= _size) return false;
        --_size;
        for (int j = i; j < _size; ++j) {
            array[j] = array[j+1];
        }
        return true;
    }

    T operator[](int i) {
#ifdef DEBUG
        if (i >= _size) {
            Serial.println("Out of bounds array access");
            Serial.flush();
        }
#endif
        return array[i];
    }
private:
    size_t _size;
    T array[S];
};

template<typename T, size_t S>
class CircularQueue {
public:
    CircularQueue() : first(0), last(0) {}

    bool empty() {
        return last == first;
    }

    bool insert(T t) {
        size_t new_last = (last+1)%S;
        if (new_last != first) {
            array[last] = t;
            last = new_last;
            return true;
        } else {
            return false;
        }
    }

    bool pop(T& t) {
        if (!empty()) {
            t = array[first];
            first = (first+1)%S;
            return true;
        } else return false;
    }

    bool peek(T& t) {
        t = array[first];
        return !empty();
    }
private:
    size_t first;
    size_t last;
    T array[S];
};

struct Vector {
    int x, y;

    int cross(Vector other) {
        return x*other.y - y*other.x;
    }

    int dot(Vector other) {
        return x*other.x + y*other.y;
    }

    Vector operator-() {
        return Vector { -x, -y };
    }

    void rotate(int r) {
        if (r != 0) {
            int t = x;
            x = -r*y;
            y =  r*t;
        }
    }
};

inline Vector operator+=(Vector& a, Vector b) {
    a.x = a.x + b.x;
    a.y = a.y + b.y;
    return a;
}

inline bool operator==(Vector a, Vector b) {
    return a.x == b.x && a.y == b.y;
}

inline bool operator!=(Vector a, Vector b) {
    return a.x != b.x || a.y != b.y;
}

const Vector REACTOR_A = Vector { 0, 0 };
const Vector REACTOR_B = Vector { 5, 0 };
const Vector SUPPLY_1 = Vector { 1, -1 };
const Vector SUPPLY_2 = Vector { 2, -1 };
const Vector SUPPLY_3 = Vector { 3, -1 };
const Vector SUPPLY_4 = Vector { 4, -1 };
const Vector STORAGE_1 = Vector { 1, 1 };
const Vector STORAGE_2 = Vector { 2, 1 };
const Vector STORAGE_3 = Vector { 3, 1 };
const Vector STORAGE_4 = Vector { 4, 1 };
const Vector START = Vector { 1, 0 };

const Vector NORTH = Vector { 0, 1 };
const Vector SOUTH = Vector { 0, -1 };
const Vector EAST = Vector { 1, 0 };
const Vector WEST = Vector { -1, 0 };

const int LEFT = 1;
const int RIGHT = -1;
