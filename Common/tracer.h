//
// Created by WuKun on 8/24/18.
// Contact me:wk707060335@gmail.com
//

#ifndef POINTCLOUDAPPLICATION_TRACER_H
#define POINTCLOUDAPPLICATION_TRACER_H

#include <iostream>

#define TRACER_DEBUG
#ifdef  TRACER_DEBUG
#define TRACER \
   Tracer tracer(__func__)
#else
#define TRACER
#endif

class Tracer
{
public:
    explicit Tracer(const char msg[]):
            m_msg(msg)
    {
        std::cout << ">>>Enter: " << m_msg << std::endl;
    }

    ~Tracer()
    {
        std::cout << "<<<Leave: " << m_msg << std::endl;
    }

private:
    const char* m_msg;
};

#endif //POINTCLOUDAPPLICATION_TRACER_H
