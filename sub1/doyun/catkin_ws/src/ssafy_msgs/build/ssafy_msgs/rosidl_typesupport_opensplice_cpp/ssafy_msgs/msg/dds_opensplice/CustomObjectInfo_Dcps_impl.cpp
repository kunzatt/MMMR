#include "CustomObjectInfo_Dcps_impl.h"
#include "CustomObjectInfo_SplDcps.h"
//#include "ssafy_msgs/msg/rosidl_typesupport_opensplice_cpp__visibility_control.h"

extern v_copyin_result
__ssafy_msgs_msg_dds__CustomObjectInfo___copyIn (
    c_base base,
    const struct ssafy_msgs::msg::dds_::CustomObjectInfo_ *from,
    struct _ssafy_msgs_msg_dds__CustomObjectInfo_ *to);

extern void
__ssafy_msgs_msg_dds__CustomObjectInfo___copyOut (
    const void *_from,
    void *_to);

// DDS ssafy_msgs::msg::dds_::CustomObjectInfo_ TypeSupportMetaHolder Object Body
ssafy_msgs::msg::dds_::CustomObjectInfo_TypeSupportMetaHolder::CustomObjectInfo_TypeSupportMetaHolder () :
        DDS::OpenSplice::TypeSupportMetaHolder ("ssafy_msgs::msg::dds_::CustomObjectInfo_", "", "")
{
    copyIn = (DDS::OpenSplice::cxxCopyIn) __ssafy_msgs_msg_dds__CustomObjectInfo___copyIn;
    copyOut = (DDS::OpenSplice::cxxCopyOut) __ssafy_msgs_msg_dds__CustomObjectInfo___copyOut;
    metaDescriptorArrLength = 6;
    metaDescriptorLength = 541;
    const char *tmp[] = {"<MetaData version=\"1.0.0\"><Module name=\"geometry_msgs\"><Module name=\"msg\"><Module name=\"dds_\">",
"<Struct name=\"Vector3_\"><Member name=\"x_\"><Double/></Member><Member name=\"y_\"><Double/></Member>",
"<Member name=\"z_\"><Double/></Member></Struct></Module></Module></Module><Module name=\"ssafy_msgs\">",
"<Module name=\"msg\"><Module name=\"dds_\"><Struct name=\"CustomObjectInfo_\"><Member name=\"position_\">",
"<Sequence><Type name=\"::geometry_msgs::msg::dds_::Vector3_\"/></Sequence></Member></Struct></Module>",
"</Module></Module></MetaData>"};
    metaDescriptor = new const char*[metaDescriptorArrLength];
    memcpy(metaDescriptor, tmp, sizeof(tmp));
}

ssafy_msgs::msg::dds_::CustomObjectInfo_TypeSupportMetaHolder::~CustomObjectInfo_TypeSupportMetaHolder ()
{
    // Rely on parent destructor.
}

::DDS::OpenSplice::TypeSupportMetaHolder *
ssafy_msgs::msg::dds_::CustomObjectInfo_TypeSupportMetaHolder::clone()
{
    return new ssafy_msgs::msg::dds_::CustomObjectInfo_TypeSupportMetaHolder();
}

::DDS::OpenSplice::DataWriter *
ssafy_msgs::msg::dds_::CustomObjectInfo_TypeSupportMetaHolder::create_datawriter ()
{
    return new ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl();
}

::DDS::OpenSplice::DataReader *
ssafy_msgs::msg::dds_::CustomObjectInfo_TypeSupportMetaHolder::create_datareader ()
{
    return new ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl();
}

::DDS::OpenSplice::DataReaderView *
ssafy_msgs::msg::dds_::CustomObjectInfo_TypeSupportMetaHolder::create_view ()
{
    return new ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl();
}

// DDS ssafy_msgs::msg::dds_::CustomObjectInfo_ TypeSupport Object Body
ssafy_msgs::msg::dds_::CustomObjectInfo_TypeSupport::CustomObjectInfo_TypeSupport () :
        DDS::OpenSplice::TypeSupport()
{
    tsMetaHolder = new ssafy_msgs::msg::dds_::CustomObjectInfo_TypeSupportMetaHolder();
}

ssafy_msgs::msg::dds_::CustomObjectInfo_TypeSupport::~CustomObjectInfo_TypeSupport ()
{
    DDS::release(tsMetaHolder);
}

// DDS ssafy_msgs::msg::dds_::CustomObjectInfo_ DataWriter_impl Object Body
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::CustomObjectInfo_DataWriter_impl ()
{
    // Parent constructor takes care of everything.
}

ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::~CustomObjectInfo_DataWriter_impl ()
{
    // Parent destructor takes care of everything.
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::init (
        DDS::OpenSplice::Publisher *publisher,
        DDS::OpenSplice::DomainParticipant *participant,
        const DDS::DataWriterQos &qos,
        DDS::OpenSplice::Topic *a_topic,
        const char *name,
        DDS::OpenSplice::cxxCopyIn copyIn,
        DDS::OpenSplice::cxxCopyOut copyOut,
        u_writerCopy writerCopy,
        void *cdrMarshaler)
{
    return DDS::OpenSplice::FooDataWriter_impl::nlReq_init(
            publisher, participant, qos, a_topic, name, copyIn, copyOut, writerCopy, cdrMarshaler);
}

::DDS::InstanceHandle_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::register_instance (
    const ssafy_msgs::msg::dds_::CustomObjectInfo_ & instance_data) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataWriter_impl::register_instance(&instance_data);
}

::DDS::InstanceHandle_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::register_instance_w_timestamp (
    const CustomObjectInfo_ & instance_data,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataWriter_impl::register_instance_w_timestamp(&instance_data, source_timestamp);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::unregister_instance (
    const ssafy_msgs::msg::dds_::CustomObjectInfo_ & instance_data,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataWriter_impl::unregister_instance(&instance_data, handle);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::unregister_instance_w_timestamp (
    const CustomObjectInfo_ & instance_data,
    ::DDS::InstanceHandle_t handle,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataWriter_impl::unregister_instance_w_timestamp(&instance_data, handle, source_timestamp);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::write (
    const ssafy_msgs::msg::dds_::CustomObjectInfo_ & instance_data,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataWriter_impl::write(&instance_data, handle);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::write_w_timestamp (
    const CustomObjectInfo_ & instance_data,
    ::DDS::InstanceHandle_t handle,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataWriter_impl::write_w_timestamp(&instance_data, handle, source_timestamp);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::dispose (
    const ssafy_msgs::msg::dds_::CustomObjectInfo_ & instance_data,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataWriter_impl::dispose(&instance_data, handle);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::dispose_w_timestamp (
    const CustomObjectInfo_ & instance_data,
    ::DDS::InstanceHandle_t handle,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataWriter_impl::dispose_w_timestamp(&instance_data, handle, source_timestamp);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::writedispose (
    const ssafy_msgs::msg::dds_::CustomObjectInfo_ & instance_data,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataWriter_impl::writedispose(&instance_data, handle);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::writedispose_w_timestamp (
    const CustomObjectInfo_ & instance_data,
    ::DDS::InstanceHandle_t handle,
    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataWriter_impl::writedispose_w_timestamp(&instance_data, handle, source_timestamp);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::get_key_value (
    CustomObjectInfo_ & key_holder,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataWriter_impl::get_key_value(&key_holder, handle);
}

::DDS::InstanceHandle_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataWriter_impl::lookup_instance (
    const ssafy_msgs::msg::dds_::CustomObjectInfo_ & instance_data) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataWriter_impl::lookup_instance(&instance_data);
}

// DDS ssafy_msgs::msg::dds_::CustomObjectInfo_ DataReader_impl Object Body
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::CustomObjectInfo_DataReader_impl ()
{
    // Parent constructor takes care of everything.
}

ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::~CustomObjectInfo_DataReader_impl ()
{
    // Parent destructor takes care of everything.
}

DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::init (
    DDS::OpenSplice::Subscriber *subscriber,
    const DDS::DataReaderQos &qos,
    DDS::OpenSplice::TopicDescription *a_topic,
    const char *name,
    DDS::OpenSplice::cxxCopyIn copyIn,
    DDS::OpenSplice::cxxCopyOut copyOut,
    DDS::OpenSplice::cxxReaderCopy readerCopy,
    void *cdrMarshaler)
{
    return DDS::OpenSplice::FooDataReader_impl::nlReq_init(
            subscriber, qos, a_topic, name, copyIn, copyOut, readerCopy, cdrMarshaler,
            this->dataSeqAlloc, this->dataSeqLength, this->dataSeqGetBuffer, this->dataSeqCopyOut);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::read (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status = DDS::RETCODE_OK;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReader_impl::read(&received_data, info_seq, max_samples, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::take (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status = DDS::RETCODE_OK;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReader_impl::take(&received_data, info_seq, max_samples, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::read_w_condition (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status = DDS::RETCODE_OK;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReader_impl::read_w_condition(&received_data, info_seq, max_samples, a_condition);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::take_w_condition (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status = DDS::RETCODE_OK;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReader_impl::take_w_condition(&received_data, info_seq, max_samples, a_condition);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::read_next_sample (
    ssafy_msgs::msg::dds_::CustomObjectInfo_ & received_data,
    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataReader_impl::read_next_sample(&received_data, sample_info);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::take_next_sample (
    ssafy_msgs::msg::dds_::CustomObjectInfo_ & received_data,
    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataReader_impl::take_next_sample(&received_data, sample_info);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::read_instance (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReader_impl::read_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::take_instance (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReader_impl::take_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::read_next_instance (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReader_impl::read_next_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::take_next_instance (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReader_impl::take_next_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::read_next_instance_w_condition (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReader_impl::read_next_instance_w_condition(&received_data, info_seq, max_samples, a_handle, a_condition);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::take_next_instance_w_condition (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReader_impl::take_next_instance_w_condition(&received_data, info_seq, max_samples, a_handle, a_condition);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::return_loan (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t result = ::DDS::RETCODE_OK;

    result = this->write_lock ();
    if (result == DDS::RETCODE_OK) {
        if ( received_data.length() > 0 ) {
            if (received_data.length() == info_seq.length() &&
                received_data.release() == info_seq.release() ) {
                if (!received_data.release()) {
                    result = DDS::OpenSplice::FooDataReader_impl::wlReq_return_loan( received_data.get_buffer(),
                                                                                     info_seq.get_buffer() );
                    if ( result == ::DDS::RETCODE_OK ) {
                        if ( !received_data.release() ) {
                            ssafy_msgs::msg::dds_::CustomObjectInfo_Seq::freebuf( received_data.get_buffer(false) );
                            received_data.replace(0, 0, NULL, false);
                            ::DDS::SampleInfoSeq::freebuf( info_seq.get_buffer(false) );
                            info_seq.replace(0, 0, NULL, false);
                        }
                    }
                }
            } else {
                result = ::DDS::RETCODE_PRECONDITION_NOT_MET;
            }
        }
        this->unlock();
    }
    return result;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::get_key_value (
    ssafy_msgs::msg::dds_::CustomObjectInfo_ & key_holder,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataReader_impl::get_key_value(&key_holder, handle);
}

::DDS::InstanceHandle_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::lookup_instance (
    const ssafy_msgs::msg::dds_::CustomObjectInfo_ & instance) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataReader_impl::lookup_instance(&instance);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::check_preconditions (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples)
{
    if ((max_samples < 0) && (max_samples != DDS::LENGTH_UNLIMITED)) {
        return DDS::RETCODE_BAD_PARAMETER;
    }

    /* Rule 1 : The values of len, max_len, and owns
     * for the two collections must be identical.
     */
    if ((received_data.length()  != info_seq.length())  ||
        (received_data.maximum() != info_seq.maximum()) ||
        (received_data.release() != info_seq.release()))
    {
        return DDS::RETCODE_PRECONDITION_NOT_MET;
    }

    /* Rule 4: If the input max_len>0 and the input owns==FALSE,
     * then the read operation will fail.
     */
    if ((info_seq.maximum() > 0) &&
        (info_seq.release() == false))
    {
        return DDS::RETCODE_PRECONDITION_NOT_MET;
    }

    /* Rule 5: If input max_len>0 and the input owns==TRUE,
     * then the read operation will...
     */
    if (info_seq.maximum() > 0) { /* owns is true, because of rule 4. */
        /* Rule 5a: If max_samples = LENGTH_UNLIMITED,
         * then at most max_len values will be copied.
         * Rule 5b: If max_samples <= max_len,
         * then at most max_samples values will be copied.
         */
        /* Rule 5c: If max_samples > max_len,
         * then the read operation will fail.
         */
        if ((max_samples != DDS::LENGTH_UNLIMITED) &&
            ((DDS::ULong)max_samples > info_seq.maximum())) {
            return DDS::RETCODE_PRECONDITION_NOT_MET;
        }
    }

    if ((max_samples == 0) ||
        ((info_seq.maximum() == 0) && (info_seq.release()))) {
        return DDS::RETCODE_NO_DATA;
    }

    return DDS::RETCODE_OK;
}

void *
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::dataSeqAlloc (
    void * received_data,
    DDS::ULong len)
{
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq *data_seq = reinterpret_cast<ssafy_msgs::msg::dds_::CustomObjectInfo_Seq *>(received_data);
    data_seq->replace(len, len, data_seq->allocbuf(len), false);
    return data_seq->get_buffer();
}

void *
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::dataSeqGetBuffer (
    void * received_data,
    DDS::ULong index)
{
	ssafy_msgs::msg::dds_::CustomObjectInfo_Seq *data_seq = reinterpret_cast<ssafy_msgs::msg::dds_::CustomObjectInfo_Seq *>(received_data);
	return &((*data_seq)[index]);
}

void
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::dataSeqLength (
    void * received_data,
    DDS::ULong len)
{
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq *data_seq = reinterpret_cast<ssafy_msgs::msg::dds_::CustomObjectInfo_Seq *>(received_data);
    data_seq->length(len);
}

void
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::dataSeqCopyOut (
    const void * from,
    void * to)
{
    ssafy_msgs::msg::dds_::CustomObjectInfo_ *data = reinterpret_cast<ssafy_msgs::msg::dds_::CustomObjectInfo_ *>(to);
    __ssafy_msgs_msg_dds__CustomObjectInfo___copyOut(from, data);
}

void
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::copyDataOut (
    const void * from,
    void * to)
{
    ssafy_msgs::msg::dds_::CustomObjectInfo_ *data = reinterpret_cast<ssafy_msgs::msg::dds_::CustomObjectInfo_ *>(to);
    __ssafy_msgs_msg_dds__CustomObjectInfo___copyOut(from, data);
}


// DDS ssafy_msgs::msg::dds_::CustomObjectInfo_ DataReaderView_impl Object Body
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::CustomObjectInfo_DataReaderView_impl ()
{
    // Parent constructor takes care of everything.
}

ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::~CustomObjectInfo_DataReaderView_impl ()
{
    // Parent destructor takes care of everything.
}

DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::init (
    DDS::OpenSplice::DataReader *reader,
    const char *name,
    const DDS::DataReaderViewQos &qos,
    DDS::OpenSplice::cxxCopyIn copyIn,
    DDS::OpenSplice::cxxCopyOut copyOut)
{
    return DDS::OpenSplice::FooDataReaderView_impl::nlReq_init(
            reader, name, qos, copyIn, copyOut, ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::dataSeqAlloc,
            ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::dataSeqLength);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::read (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReaderView_impl::read(&received_data, info_seq, max_samples, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::take (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReaderView_impl::take(&received_data, info_seq, max_samples, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::read_w_condition (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReaderView_impl::read_w_condition(&received_data, info_seq, max_samples, a_condition);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::take_w_condition (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReaderView_impl::take_w_condition(&received_data, info_seq, max_samples, a_condition);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::read_next_sample (
    ssafy_msgs::msg::dds_::CustomObjectInfo_ & received_data,
    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataReaderView_impl::read_next_sample(&received_data, sample_info);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::take_next_sample (
    ssafy_msgs::msg::dds_::CustomObjectInfo_ & received_data,
    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataReaderView_impl::take_next_sample(&received_data, sample_info);
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::read_instance (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReaderView_impl::read_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::take_instance (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReaderView_impl::take_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::read_next_instance (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReaderView_impl::read_next_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::take_next_instance (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::SampleStateMask sample_states,
    ::DDS::ViewStateMask view_states,
    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReaderView_impl::take_next_instance(&received_data, info_seq, max_samples, a_handle, sample_states, view_states, instance_states);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::read_next_instance_w_condition (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReaderView_impl::read_next_instance_w_condition(&received_data, info_seq, max_samples, a_handle, a_condition);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::take_next_instance_w_condition (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq,
    ::DDS::Long max_samples,
    ::DDS::InstanceHandle_t a_handle,
    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t status;

    status = ssafy_msgs::msg::dds_::CustomObjectInfo_DataReader_impl::check_preconditions(received_data, info_seq, max_samples);
    if ( status == ::DDS::RETCODE_OK ) {
        status = DDS::OpenSplice::FooDataReaderView_impl::take_next_instance_w_condition(&received_data, info_seq, max_samples, a_handle, a_condition);
    }
    return status;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::return_loan (
    ssafy_msgs::msg::dds_::CustomObjectInfo_Seq & received_data,
    ::DDS::SampleInfoSeq & info_seq) THROW_ORB_EXCEPTIONS
{
    ::DDS::ReturnCode_t result = ::DDS::RETCODE_OK;

    result = this->write_lock ();
    if (result == DDS::RETCODE_OK) {
        if ( received_data.length() > 0 ) {
            if (received_data.length() == info_seq.length() &&
                received_data.release() == info_seq.release() ) {
                if (!received_data.release()) {
                    result = DDS::OpenSplice::FooDataReaderView_impl::wlReq_return_loan( received_data.get_buffer(),
                                                           info_seq.get_buffer() );

                    if ( result == ::DDS::RETCODE_OK ) {
                        if ( !received_data.release() ) {
                            ssafy_msgs::msg::dds_::CustomObjectInfo_Seq::freebuf( received_data.get_buffer(false) );
                            received_data.replace(0, 0, NULL, false);
                            ::DDS::SampleInfoSeq::freebuf( info_seq.get_buffer(false) );
                            info_seq.replace(0, 0, NULL, false);
                        }
                    } else if ( result == ::DDS::RETCODE_NO_DATA ) {
                        if ( received_data.release() ) {
                            result = ::DDS::RETCODE_OK;
                        } else {
                            result = ::DDS::RETCODE_PRECONDITION_NOT_MET;
                        }
                    }
                }
            } else {
                result = ::DDS::RETCODE_PRECONDITION_NOT_MET;
            }
        }
        this->unlock();
    }
    return result;
}

::DDS::ReturnCode_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::get_key_value (
    ssafy_msgs::msg::dds_::CustomObjectInfo_ & key_holder,
    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataReaderView_impl::get_key_value(&key_holder, handle);
}

::DDS::InstanceHandle_t
ssafy_msgs::msg::dds_::CustomObjectInfo_DataReaderView_impl::lookup_instance (
    const ssafy_msgs::msg::dds_::CustomObjectInfo_ & instance) THROW_ORB_EXCEPTIONS
{
    return DDS::OpenSplice::FooDataReaderView_impl::lookup_instance(&instance);
}
