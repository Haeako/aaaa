#ifndef _data_struct_h_
#define _data_struct_h_
#include <stdint.h>
#include<string.h>
#define DIRECTION 3
typedef struct 
{
    double vector[DIRECTION];
} PIDvector;

class RingBufferAccessor
{
private:
    PIDvector **m_audio_buffers;
    int m_number_PIDvector_buffers;
    PIDvector *m_current_buffer;
    int m_buffer_pos;
    int m_buffer_idx;
    int m_total_size;

public:
    RingBufferAccessor(PIDvector **PIDvectors, int number_PIDvector_buffers)
    {
        m_buffer_pos = 0;
        m_total_size = number_PIDvector_buffers * DIRECTION;
        m_number_PIDvector_buffers = ;
         m_current_buffer = PIDvectors[0];
    }
    int getIndex()
    {
        return m_buffer_idx * DIRECTION + m_buffer_pos;
    }
    void setIndex(int index)
    {
        // handle negative indexes
        index = (index + m_total_size) % m_total_size;
        // work out which buffer
        m_buffer_idx = (index / DIRECTION) % m_number_PIDvector_buffers;
        // and where we are in the buffer
        m_buffer_pos = index % DIRECTION;
        m_current_buffer = m_audio_buffers[m_buffer_idx];
    }

    inline void rewind(int samples) {
        setIndex(getIndex() - samples);
    }
    inline bool moveToNextSample()
    {
        m_buffer_pos++;
        if (m_buffer_pos == DIRECTION)
        {
            m_buffer_pos = 0;
            m_buffer_idx++;
            if (m_buffer_idx == m_number_PIDvector_buffers)
            {
                m_buffer_idx = 0;
            }
            m_current_buffer = m_audio_buffers[m_buffer_idx];
            return true;
        }
        return false;
    }
};

#endif