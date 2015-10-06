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
private:
    size_t first;
    size_t last;
    T array[S];
};

