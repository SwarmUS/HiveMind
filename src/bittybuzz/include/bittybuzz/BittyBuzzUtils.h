#ifndef BITTYBUZZUTILS_H_
#define BITTYBUZZUTILS_H_

#include "BittyBuzzSystem.h"
#include <bbzvm.h>

/**
 *@brief Namespace with various utilities for bittybuzz */
namespace BittyBuzzUtils {

    /**
     *@brief Logs an bittybuzz object to a human readable format
     *
     *@param [in] obj the object to log
     *@param [in] fun the functor to call, takes a const char* as first argument, a float/int a
     *second and the passed args from the template
     *@param [in] args the arguments to pass for context
     */
    template <typename F, typename... Args>
    void logObj(bbzobj_t* obj, F fun, Args... args) {
        switch (bbztype(*obj)) {
        case BBZTYPE_NIL: {
            fun("[nil]", &args...);
            break;
        }
        case BBZTYPE_INT: {
            fun("%d", obj->i.value, &args...);
            break;
        }
        case BBZTYPE_FLOAT: {
            fun("%f", bbzfloat_tofloat(obj->f.value), &args...);
            break;
        }
        case BBZTYPE_TABLE: {
            fun("[t:%d]", obj->t.value, &args...);
            break;
        }
        case BBZTYPE_STRING: {
            std::optional<const char*> optionString =
                BittyBuzzSystem::g_stringResolver->getString(obj->s.value);
            if (optionString) {
                fun("%s", optionString.value(), &args...);
            } else {
                fun("{UNKNOWN STRID: %d}", obj->s.value, &args...);
            }
            break;
        }
        case BBZTYPE_CLOSURE: {
            if (bbztype_isclosurelambda(*obj)) {
                fun("[cl: %d}", obj->l.value.ref, &args...);
            } else {
                fun("[c: %d]", (int)(intptr_t)obj->c.value, &args...);
            }
            break;
        }
        case BBZTYPE_USERDATA: {
            fun("[u: %d]", (int)obj->u.value, &args...);
            break;
        }
        default: {
            fun("{UNKNOWN TYPE}", &args...);
            break;
        }
        }
    }

} // namespace BittyBuzzUtils

#endif // BITTYBUZZUTILS_H_
