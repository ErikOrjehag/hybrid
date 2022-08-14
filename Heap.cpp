#include "include/Heap.hpp"
#include <iostream>
#include <string>

template<Derived<HeapItem> T, size_t max_heap_size>
Heap<T, max_heap_size>::Heap()
 : count(0)
{

}

template<Derived<HeapItem> T, size_t max_heap_size>
void Heap<T, max_heap_size>::push(T item)
{
    if (count >= max_heap_size) {
        throw std::string("Can not push into full heap");
    }

    item.heap_index = count;
    items[count] = item;
    sort_up(item);
    count += 1;
}

template<Derived<HeapItem> T, size_t max_heap_size>
T Heap<T, max_heap_size>::pop()
{
    if (count == 0) {
        throw std::string("Can not pop from empty heap");
    }

    T first = items[0];
    count -= 1;
    items[0] = items[count];
    items[0].heap_index = 0;
    sort_down(items[0]);
    return first;
}

template<Derived<HeapItem> T, size_t max_heap_size>
void Heap<T, max_heap_size>::sort_up(T& item)
{
    while (item.heap_index > 0)
    {
        size_t parent_index = (item.heap_index - 1) / 2;

        T& parent = items[parent_index];

        if (item < parent) {
            swap(item, parent);
        } else {
            break;
        }
    }
}

template<Derived<HeapItem> T, size_t max_heap_size>
void Heap<T, max_heap_size>::sort_down(T& item)
{
    while (true)
    {
        size_t index_left_child = item.heap_index * 2 + 1;
        size_t index_right_child = item.heap_index * 2 + 2;
        size_t index_smallest_child = 0;

        if (index_left_child < count)
        {
            if (index_right_child < count && items[index_right_child] < items[index_left_child])
            {
                index_smallest_child = index_right_child;
            }
            else
            {
                index_smallest_child = index_left_child;
            }
            if (item > items[index_smallest_child])
            {
                swap(item, items[index_smallest_child]);
            }
            else
            {
                return;
            }
        }
        else
        {
            return;
        }
    }
}

template<Derived<HeapItem> T, size_t max_heap_size>
void Heap<T, max_heap_size>::swap(T& item_a, T& item_b)
{
    items[item_a.heap_index] = item_b;
    items[item_b.heap_index] = item_a;
    size_t index_a = item_a.heap_index;
    item_a.heap_index = item_b.heap_index;
    item_b.heap_index = item_a.heap_index;
}

template<Derived<HeapItem> T, size_t max_heap_size>
void Heap<T, max_heap_size>::clear()
{
    count = 0;
}