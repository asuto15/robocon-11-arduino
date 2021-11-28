#ifndef _WAKWAK_KOBA_MOVING_AVERAGE_
#define _WAKWAK_KOBA_MOVING_AVERAGE_

#include <stdint.h>

template <typename T>
class MovingAverage
{
public:
    MovingAverage(int buffer_size = 100)
    {
        this->buffer_size = buffer_size;
        buffer = new T[buffer_size];
    }
    ~MovingAverage()
    {
        delete[] buffer;
    }

    int setValue(T value)
    {
        buffer[index++] = value;
        if (index >= buffer_size)
            index = 0;
        return ++count;
    }

    T getAverage(int max_count = 0)
    {
        if (!max_count)
            max_count = min((unsigned long)count, (unsigned long)buffer_size);
        else if (count < max_count)
            max_count = count;
        else if (max_count > buffer_size)
            max_count = buffer_size;

        int idx = index - 1;
        int sum_count = 0;
        float sum_value = 0;

        for (; sum_count < max_count; sum_count++)
        {
            if (idx < 0)
                idx += buffer_size;
            sum_value += (float)buffer[idx--];
        }

        return (T)(sum_value / sum_count);
    }

    T getMin(int max_count = 0)
    {
        if (!max_count)
            max_count = min((unsigned long)count, (unsigned long)buffer_size);
        else if (count < max_count)
            max_count = count;
        else if (max_count > buffer_size)
            max_count = buffer_size;

        int idx = index - 1;
        int sum_count = 0;
        T result = 0;

        for (; sum_count < max_count; sum_count++)
        {
            if (idx < 0)
                idx += buffer_size;
            T value = buffer[idx--];
            result = !sum_count ? value : min(result, value);
        }

        return result;
    }

    T getMax(int max_count = 0)
    {
        if (!max_count)
            max_count = min((unsigned long)count, (unsigned long)buffer_size);
        else if (count < max_count)
            max_count = count;
        else if (max_count > buffer_size)
            max_count = buffer_size;

        int idx = index - 1;
        int sum_count = 0;
        T result = 0;

        for (; sum_count < max_count; sum_count++)
        {
            if (idx < 0)
                idx += buffer_size;
            T value = buffer[idx--];
            result = !sum_count ? value : max(result, value);
        }

        return result;
    }

protected:
private:
    int buffer_size;
    volatile T *buffer;
    volatile unsigned long count = 0;
    volatile unsigned int index = 0;
};

#endif
