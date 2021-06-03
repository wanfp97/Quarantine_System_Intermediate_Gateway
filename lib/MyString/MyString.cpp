#include "MyString.h"

bool MyString :: modify_string(unsigned int index, char value) 
{
    if(buffer[index]) {
        buffer[index] = value;
        return true;
    }
    return false;
}
