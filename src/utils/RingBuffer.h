#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <Arduino.h>


template<typename T, size_t SIZE> //oop generic T depolanacak veri tipi SIZE tampon boyutu (2'nin kuvveti olmalı)

//2 nin kuvveti olma sebebi algoritma mantığından gelmektedir normalde (index % SIZE) yapılması gerekir.
//Fakat % işlemi zahmetlidir ve interrupt içinde kullanmak risklidir.
//Bunun yerine (index & (SIZE - 1))(SIZE 2'nin kuvveti) yapılırsa aynı sonuç alınır.

class RingBuffer {//ring buffer tek üretici ve tek tüketici tarafından kullanılan halka şeklinde bir veri tamponudur ve FIFO prensiplidir.
public:

    RingBuffer() : _head(0), _tail(0) {
        static_assert(SIZE > 0, "Tampon boyutu > 0 olmali");

        static_assert((SIZE & (SIZE - 1)) == 0, "Tampon boyutu 2'nin kuvveti olmali");
    }

    bool push(const T &item) {
        size_t nextHead = (_head + 1) & (SIZE - 1);

        if (nextHead == _tail) {
            return false;
        }
        _buffer[_head] = item;
        _head = nextHead;

        return true;
    }

    bool pop(T &item) {
        if (_tail == _head) {
            return false;
        }

        item = _buffer[_tail];
        _tail = (_tail + 1) & (SIZE - 1);

        return true;
    }

    size_t available() const {
        return (_head - _tail) & (SIZE - 1);
    }

    bool isFull() const {
        return ((_head + 1) & (SIZE - 1)) == _tail;
    }

    bool isEmpty() const {
        return _head == _tail;
    }

    void clear() {
        _tail = _head;
    }

private:
    T _buffer[SIZE];
    volatile size_t _head;
    volatile size_t _tail;
};

#endif
