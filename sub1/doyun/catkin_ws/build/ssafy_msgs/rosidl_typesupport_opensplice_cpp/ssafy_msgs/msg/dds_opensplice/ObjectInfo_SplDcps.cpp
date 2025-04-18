#include "ObjectInfo_SplDcps.h"
#include "ccpp_ObjectInfo_.h"

#include <v_copyIn.h>
#include <v_topic.h>
#include <os_stdlib.h>
#include <string.h>
#include <os_report.h>

v_copyin_result
__ssafy_msgs_msg_dds__ObjectInfo___copyIn(
    c_base base,
    const struct ::ssafy_msgs::msg::dds_::ObjectInfo_ *from,
    struct _ssafy_msgs_msg_dds__ObjectInfo_ *to)
{
    v_copyin_result result = V_COPYIN_RESULT_OK;
    (void) base;

    to->num_obj_ = (c_short)from->num_obj_;
    {
/* Code generated by C:\cygwin\home\dds\OvernightTests\tmp_yZe4TVbJGx\build\src\tools\idlpp\code\idl_genCorbaCxxCopyin.c at line 838 */

        c_type type0 = NULL;
        c_type subtype0;
        c_ulong length0;
        c_short *dest0;
        const ::ssafy_msgs::msg::dds_::ObjectInfo_::_idx_obj__seq *src = &from->idx_obj_;

        if (type0 == NULL) {
            subtype0 = c_type(c_metaResolve (c_metaObject(base), "c_short"));
            type0 = c_metaSequenceTypeNew(c_metaObject(base),"C_SEQUENCE<c_short>",subtype0,0);
            c_free(subtype0);
        }
        length0 = (c_ulong)(*src).length();
#ifdef OSPL_BOUNDS_CHECK
         dest0 = (c_short *)c_newSequence_s(c_collectionType(type0),length0);
         if(dest0 != NULL) {
            /* Code generated by C:\cygwin\home\dds\OvernightTests\tmp_yZe4TVbJGx\build\src\tools\idlpp\code\idl_genCorbaCxxCopyin.c at line 2041 */
            const c_short *buf0;
            buf0 = (const c_short *)(*src).get_buffer();
            memcpy (dest0,buf0,length0* sizeof(*dest0));
            to->idx_obj_ = (c_sequence)dest0;
         } else {
             result = V_COPYIN_RESULT_OUT_OF_MEMORY;
          }
#else
        dest0 = (c_short *)c_newSequence_s(c_collectionType(type0),length0);
        if(dest0 != NULL) {
            /* Code generated by C:\cygwin\home\dds\OvernightTests\tmp_yZe4TVbJGx\build\src\tools\idlpp\code\idl_genCorbaCxxCopyin.c at line 2041 */
            const c_short *buf0;
            buf0 = (const c_short *)(*src).get_buffer();
            memcpy (dest0,buf0,length0* sizeof(*dest0));
            to->idx_obj_ = (c_sequence)dest0;
        } else {
            result = V_COPYIN_RESULT_OUT_OF_MEMORY;
        }
#endif
        c_free(type0);
    }
    {
/* Code generated by C:\cygwin\home\dds\OvernightTests\tmp_yZe4TVbJGx\build\src\tools\idlpp\code\idl_genCorbaCxxCopyin.c at line 838 */

        c_type type0 = NULL;
        c_type subtype0;
        c_ulong length0;
        c_float *dest0;
        const ::ssafy_msgs::msg::dds_::ObjectInfo_::_x__seq *src = &from->x_;

        if (type0 == NULL) {
            subtype0 = c_type(c_metaResolve (c_metaObject(base), "c_float"));
            type0 = c_metaSequenceTypeNew(c_metaObject(base),"C_SEQUENCE<c_float>",subtype0,0);
            c_free(subtype0);
        }
        length0 = (c_ulong)(*src).length();
#ifdef OSPL_BOUNDS_CHECK
         dest0 = (c_float *)c_newSequence_s(c_collectionType(type0),length0);
         if(dest0 != NULL) {
            /* Code generated by C:\cygwin\home\dds\OvernightTests\tmp_yZe4TVbJGx\build\src\tools\idlpp\code\idl_genCorbaCxxCopyin.c at line 2041 */
            const c_float *buf0;
            buf0 = (const c_float *)(*src).get_buffer();
            memcpy (dest0,buf0,length0* sizeof(*dest0));
            to->x_ = (c_sequence)dest0;
         } else {
             result = V_COPYIN_RESULT_OUT_OF_MEMORY;
          }
#else
        dest0 = (c_float *)c_newSequence_s(c_collectionType(type0),length0);
        if(dest0 != NULL) {
            /* Code generated by C:\cygwin\home\dds\OvernightTests\tmp_yZe4TVbJGx\build\src\tools\idlpp\code\idl_genCorbaCxxCopyin.c at line 2041 */
            const c_float *buf0;
            buf0 = (const c_float *)(*src).get_buffer();
            memcpy (dest0,buf0,length0* sizeof(*dest0));
            to->x_ = (c_sequence)dest0;
        } else {
            result = V_COPYIN_RESULT_OUT_OF_MEMORY;
        }
#endif
        c_free(type0);
    }
    {
/* Code generated by C:\cygwin\home\dds\OvernightTests\tmp_yZe4TVbJGx\build\src\tools\idlpp\code\idl_genCorbaCxxCopyin.c at line 838 */

        c_type type0 = NULL;
        c_type subtype0;
        c_ulong length0;
        c_float *dest0;
        const ::ssafy_msgs::msg::dds_::ObjectInfo_::_y__seq *src = &from->y_;

        if (type0 == NULL) {
            subtype0 = c_type(c_metaResolve (c_metaObject(base), "c_float"));
            type0 = c_metaSequenceTypeNew(c_metaObject(base),"C_SEQUENCE<c_float>",subtype0,0);
            c_free(subtype0);
        }
        length0 = (c_ulong)(*src).length();
#ifdef OSPL_BOUNDS_CHECK
         dest0 = (c_float *)c_newSequence_s(c_collectionType(type0),length0);
         if(dest0 != NULL) {
            /* Code generated by C:\cygwin\home\dds\OvernightTests\tmp_yZe4TVbJGx\build\src\tools\idlpp\code\idl_genCorbaCxxCopyin.c at line 2041 */
            const c_float *buf0;
            buf0 = (const c_float *)(*src).get_buffer();
            memcpy (dest0,buf0,length0* sizeof(*dest0));
            to->y_ = (c_sequence)dest0;
         } else {
             result = V_COPYIN_RESULT_OUT_OF_MEMORY;
          }
#else
        dest0 = (c_float *)c_newSequence_s(c_collectionType(type0),length0);
        if(dest0 != NULL) {
            /* Code generated by C:\cygwin\home\dds\OvernightTests\tmp_yZe4TVbJGx\build\src\tools\idlpp\code\idl_genCorbaCxxCopyin.c at line 2041 */
            const c_float *buf0;
            buf0 = (const c_float *)(*src).get_buffer();
            memcpy (dest0,buf0,length0* sizeof(*dest0));
            to->y_ = (c_sequence)dest0;
        } else {
            result = V_COPYIN_RESULT_OUT_OF_MEMORY;
        }
#endif
        c_free(type0);
    }
    return result;
}

void
__ssafy_msgs_msg_dds__ObjectInfo___copyOut(
    const void *_from,
    void *_to)
{
    const struct _ssafy_msgs_msg_dds__ObjectInfo_ *from = (const struct _ssafy_msgs_msg_dds__ObjectInfo_ *)_from;
    struct ::ssafy_msgs::msg::dds_::ObjectInfo_ *to = (struct ::ssafy_msgs::msg::dds_::ObjectInfo_ *)_to;
    to->num_obj_ = (::DDS::Short)from->num_obj_;
    {
        long size0;
        const c_short *src0 = (const c_short *)from->idx_obj_;
        ::ssafy_msgs::msg::dds_::ObjectInfo_::_idx_obj__seq *dst = &to->idx_obj_;

        size0 = c_arraySize(c_sequence(from->idx_obj_));
        to->idx_obj_.length(size0);
        {
            c_short *buf0;
            buf0 = (c_short *)(*dst).get_buffer();
            memcpy ((void *)buf0,src0,size0* sizeof(*buf0));
        }
    }
    {
        long size0;
        const c_float *src0 = (const c_float *)from->x_;
        ::ssafy_msgs::msg::dds_::ObjectInfo_::_x__seq *dst = &to->x_;

        size0 = c_arraySize(c_sequence(from->x_));
        to->x_.length(size0);
        {
            c_float *buf0;
            buf0 = (c_float *)(*dst).get_buffer();
            memcpy ((void *)buf0,src0,size0* sizeof(*buf0));
        }
    }
    {
        long size0;
        const c_float *src0 = (const c_float *)from->y_;
        ::ssafy_msgs::msg::dds_::ObjectInfo_::_y__seq *dst = &to->y_;

        size0 = c_arraySize(c_sequence(from->y_));
        to->y_.length(size0);
        {
            c_float *buf0;
            buf0 = (c_float *)(*dst).get_buffer();
            memcpy ((void *)buf0,src0,size0* sizeof(*buf0));
        }
    }
}

