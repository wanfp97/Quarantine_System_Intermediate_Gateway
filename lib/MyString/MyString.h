#ifndef MyString_h
#define MyString_h

#include <WString.h>

class MyString : public String {
    public:
        using String::String;
        bool modify_string(unsigned int index, char value);
};

#endif
