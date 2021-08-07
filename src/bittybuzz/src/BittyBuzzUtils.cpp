#include "BittyBuzzUtils.h"

int BittyBuzzUtils::logObj(bbzobj_t* obj, char* str, uint16_t maxSize) {
    {
        switch (bbztype(*obj)) {
        case BBZTYPE_NIL: {
            return snprintf(str, maxSize, "[nil]");
            break;
        }
        case BBZTYPE_INT: {
            return snprintf(str, maxSize, "%d", obj->i.value);
        }
        case BBZTYPE_FLOAT: {
            return snprintf(str, maxSize, "%f", bbzfloat_tofloat(obj->f.value));
        }
        case BBZTYPE_TABLE: {
            return snprintf(str, maxSize, "[t:%d]", obj->t.value);
        }
        case BBZTYPE_STRING: {
            std::optional<const char*> optionString =
                BittyBuzzSystem::g_stringResolver->getString(obj->s.value);
            if (optionString) {
                return snprintf(str, maxSize, "%s", optionString.value());
            }
            return snprintf(str, maxSize, "{UNKNOWN STRID: %d}", obj->s.value);
        }
        case BBZTYPE_CLOSURE: {
            if (bbztype_isclosurelambda(*obj)) {
                return snprintf(str, maxSize, "[cl: %d}", obj->l.value.ref);
            }
            return snprintf(str, maxSize, "[c: %d]", (int)(intptr_t)obj->c.value);
        }
        case BBZTYPE_USERDATA: {
            return snprintf(str, maxSize, "[u: %d]", (int)obj->u.value);
        }
        default: {
            return snprintf(str, maxSize, "{UNKNOWN TYPE}");
        }
        }
    }
}
