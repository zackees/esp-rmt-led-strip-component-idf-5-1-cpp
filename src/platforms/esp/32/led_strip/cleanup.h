#pragma once

struct Cleanup {
    typedef void (*void_fn_t)(void*);
    template<typename CleanupFunction>
    Cleanup(CleanupFunction fn, void *arg) : mFn((void_fn_t)fn), mArg(arg) {}
    
    ~Cleanup() {
        if (mFn) {
            mFn(mArg);
        }
    }
    void release() {
        mFn = nullptr;
    }
    void_fn_t mFn = nullptr;
    void *mArg = nullptr;
};