#pragma once

/**
 * A simple template for a statically allocated list
 * of a given size
 **/
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

/**
 * Template for a statically allocated circular queue of a given size
 **/
template<typename T, size_t S>
class CircularQueue {
public:
    CircularQueue() : first(0), last(0) {}

    /**
     * @return true if the queue is empty, false otherwise
     **/
    bool empty() {
        return last == first;
    }

    /**
     * Insert an element to the end of the queue
     * @param t element to insert
     * @return true if the element was successfully inserted, false otherwise
     **/
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

    /**
     * Pops an element from the beginning of the queue
     * @param t element popped
     * @return true if an element was actually popped (i.e. queue had elements) false otherwise
     **/
    bool pop(T& t) {
        if (!empty()) {
            t = array[first];
            first = (first+1)%S;
            return true;
        } else return false;
    }

    /**
     * Looks at the first element of the queue
     * @param t element peeked
     * @return true if the queue had elements, false otherwise.
     **/
    bool peek(T& t) {
        t = array[first];
        return !empty();
    }
private:
    size_t first;
    size_t last;
    T array[S];
};

/**
 * A 2d vector
 **/
struct Vector {
    int x, y;

    /**
     * 2d vector cross product
     * @param other vector to compute cross product with
     * @return the cross product
     **/
    int cross(Vector other) {
        return x*other.y - y*other.x;
    }

    /**
     * 2d vector cross product
     * @param other vector to compute dot product with
     * @return the dot product
     **/
    int dot(Vector other) {
        return x*other.x + y*other.y;
    }

    /**
     * Vector negation
     * @return the vector of the same magnitude in the opposite direction
     **/
    Vector operator-() {
        return Vector { -x, -y };
    }

    /**
     * Rotate this vector by 90 degrees.
     * @param r Direction to rotate 1 is counterclockwise, -1 is clockwise. 0 performs no rotation
     **/
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

// Constants for field locations
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
const Vector START_POS = Vector { 1, 0 };
// Vector direction constants
const Vector NORTH = Vector { 0, 1 };
const Vector SOUTH = Vector { 0, -1 };
const Vector EAST = Vector { 1, 0 };
const Vector WEST = Vector { -1, 0 };
// Rotation constnts
const int LEFT = 1;
const int RIGHT = -1;
