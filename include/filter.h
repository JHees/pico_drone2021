#pragma once
#include "pico/stdlib.h"
#include <algorithm>
#include <stdio.h>
#include <string.h>
#include <utility>

template <class queueType, std::size_t length> class queue
{
  protected:
    queueType data[length];
    size_t first_index, end_index;

  public:
    queue() : first_index(0), end_index(0), data({queueType()}){};

    void operator<<(queueType new_data) // TODO move copy
    {
        push_back(new_data);
    };
    void push_back(queueType new_data)
    {
        data[end_index] = new_data;
        end_index++;
        end_index %= length;
        if (end_index == first_index)
        {
            first_index++;
            first_index %= length;
        }
    }
    void pop()
    {
        assert(get_size() != 0);
        if (end_index)
            end_index--;
        else
            end_index = length - 1;
    }
    void clean()
    {
        first_index = 0;
        end_index = 0;
    }
    queueType *const get_ptr()
    {
        return data;
    }
    queueType &get_data(size_t idx)
    {
        return data[(first_index + idx) % length];
    }
    queueType &operator[](size_t idx)
    {
        return get_data(idx);
    }
    queueType &last()
    {
        return end_index ? data[end_index - 1] : data[length - 1];
    }
    size_t get_size()
    {
        return (length + end_index - first_index) % length;
    }
};

template <class FilterType, size_t length> class MiddleFilter : public queue<FilterType, length>
{
  private:
    // queue<FilterType, length> data;
    using queueClass = queue<FilterType, length>;
    FilterType buffer[length];
    bool flag; // true if it had been read

  public:
    MiddleFilter() = default;
    bool get_flag() const
    {
        return flag;
    }
    void set_flag()
    {
        flag = false;
    }
    FilterType result()
    {
        if (queueClass::get_size() == length)
            memcpy(buffer, queueClass::data, sizeof(buffer));
        else if (queueClass::first_index <= queueClass::end_index)
        {
            memcpy(buffer, queueClass::data + queueClass::first_index,
                   (queueClass::get_size() + 1) * sizeof(FilterType));
        }
        else
        {
            memcpy(buffer, queueClass::data, (queueClass::end_index + 2) * sizeof(FilterType));
            memcpy(buffer + queueClass::end_index, queueClass::data + queueClass::first_index,
                   (length - queueClass::first_index) * sizeof(FilterType));
        }
        for (size_t i = queueClass::get_size(); i < length; ++i)
        {
            buffer[i] = 0;
        }
        std::sort(buffer, buffer + length);
        flag = true;
        return buffer[int(length - queueClass::get_size() / 2)];
    }
};