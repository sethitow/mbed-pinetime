
#ifndef MBED_SEGGER_RTT_H
#define MBED_SEGGER_RTT_H

#include "platform/platform.h"

#include "platform/FileHandle.h"

class SeggerRTT : public mbed::FileHandle {

public:

    SeggerRTT(void);

    virtual ssize_t write(const void *buffer, size_t size);

    virtual ssize_t read(void *buffer, size_t size)
    {
        /* Reading is not supported by this file handle */
        return -EBADF;
    }

    virtual off_t seek(off_t offset, int whence = SEEK_SET)
    {
        /* Seeking is not support by this file handler */
        return -ESPIPE;
    }

    virtual off_t size()
    {
        /* Size is not defined for this file handle */
        return -EINVAL;
    }

    virtual int isatty()
    {
        /* File handle is used for terminal output */
        return true;
    }

    virtual int close()
    {
        return 0;
    }
};


#endif // MBED_SEGGER_RTT_H
