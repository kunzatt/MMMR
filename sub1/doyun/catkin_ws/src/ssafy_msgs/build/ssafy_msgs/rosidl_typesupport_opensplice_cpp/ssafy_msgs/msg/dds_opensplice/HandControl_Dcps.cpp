//******************************************************************
// 
//  Generated by IDL to C++ Translator
//  
//  File name: HandControl_Dcps.cpp
//  Source: ssafy_msgs\msg\HandControl_.idl
//  Generated: timestamp removed to make the build reproducible
//  OpenSplice 6.9.190403OSS
//  
//******************************************************************

#include "HandControl_Dcps.h"

#if DDS_USE_EXPLICIT_TEMPLATES
template class DDS_DCPSUFLSeq < ssafy_msgs::msg::dds_::HandControl_, struct HandControl_Seq_uniq_>;
#endif

const char * ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface::_local_id = "IDL:ssafy_msgs/msg/dds_/HandControl_TypeSupportInterface:1.0";

ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface_ptr ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface::_duplicate (ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface_ptr p)
{
   if (p) p->m_count++;
   return p;
}

DDS::Boolean ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface::_local_is_a (const char * _id)
{
   if (strcmp (_id, ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface::_local_id) == 0)
   {
      return true;
   }

   typedef DDS::TypeSupport NestedBase_1;

   if (NestedBase_1::_local_is_a (_id))
   {
      return true;
   }

   return false;
}

ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface_ptr ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface::_narrow (DDS::Object_ptr p)
{
   ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface_ptr result = NULL;
   if (p && p->_is_a (ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface::_local_id))
   {
      result = dynamic_cast < ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface_ptr> (p);
      if (result) result->m_count++;
   }
   return result;
}

ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface_ptr ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface::_unchecked_narrow (DDS::Object_ptr p)
{
   ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface_ptr result;
   result = dynamic_cast < ssafy_msgs::msg::dds_::HandControl_TypeSupportInterface_ptr> (p);
   if (result) result->m_count++;
   return result;
}

const char * ssafy_msgs::msg::dds_::HandControl_DataWriter::_local_id = "IDL:ssafy_msgs/msg/dds_/HandControl_DataWriter:1.0";

ssafy_msgs::msg::dds_::HandControl_DataWriter_ptr ssafy_msgs::msg::dds_::HandControl_DataWriter::_duplicate (ssafy_msgs::msg::dds_::HandControl_DataWriter_ptr p)
{
   if (p) p->m_count++;
   return p;
}

DDS::Boolean ssafy_msgs::msg::dds_::HandControl_DataWriter::_local_is_a (const char * _id)
{
   if (strcmp (_id, ssafy_msgs::msg::dds_::HandControl_DataWriter::_local_id) == 0)
   {
      return true;
   }

   typedef DDS::DataWriter NestedBase_1;

   if (NestedBase_1::_local_is_a (_id))
   {
      return true;
   }

   return false;
}

ssafy_msgs::msg::dds_::HandControl_DataWriter_ptr ssafy_msgs::msg::dds_::HandControl_DataWriter::_narrow (DDS::Object_ptr p)
{
   ssafy_msgs::msg::dds_::HandControl_DataWriter_ptr result = NULL;
   if (p && p->_is_a (ssafy_msgs::msg::dds_::HandControl_DataWriter::_local_id))
   {
      result = dynamic_cast < ssafy_msgs::msg::dds_::HandControl_DataWriter_ptr> (p);
      if (result) result->m_count++;
   }
   return result;
}

ssafy_msgs::msg::dds_::HandControl_DataWriter_ptr ssafy_msgs::msg::dds_::HandControl_DataWriter::_unchecked_narrow (DDS::Object_ptr p)
{
   ssafy_msgs::msg::dds_::HandControl_DataWriter_ptr result;
   result = dynamic_cast < ssafy_msgs::msg::dds_::HandControl_DataWriter_ptr> (p);
   if (result) result->m_count++;
   return result;
}

const char * ssafy_msgs::msg::dds_::HandControl_DataReader::_local_id = "IDL:ssafy_msgs/msg/dds_/HandControl_DataReader:1.0";

ssafy_msgs::msg::dds_::HandControl_DataReader_ptr ssafy_msgs::msg::dds_::HandControl_DataReader::_duplicate (ssafy_msgs::msg::dds_::HandControl_DataReader_ptr p)
{
   if (p) p->m_count++;
   return p;
}

DDS::Boolean ssafy_msgs::msg::dds_::HandControl_DataReader::_local_is_a (const char * _id)
{
   if (strcmp (_id, ssafy_msgs::msg::dds_::HandControl_DataReader::_local_id) == 0)
   {
      return true;
   }

   typedef DDS::DataReader NestedBase_1;

   if (NestedBase_1::_local_is_a (_id))
   {
      return true;
   }

   return false;
}

ssafy_msgs::msg::dds_::HandControl_DataReader_ptr ssafy_msgs::msg::dds_::HandControl_DataReader::_narrow (DDS::Object_ptr p)
{
   ssafy_msgs::msg::dds_::HandControl_DataReader_ptr result = NULL;
   if (p && p->_is_a (ssafy_msgs::msg::dds_::HandControl_DataReader::_local_id))
   {
      result = dynamic_cast < ssafy_msgs::msg::dds_::HandControl_DataReader_ptr> (p);
      if (result) result->m_count++;
   }
   return result;
}

ssafy_msgs::msg::dds_::HandControl_DataReader_ptr ssafy_msgs::msg::dds_::HandControl_DataReader::_unchecked_narrow (DDS::Object_ptr p)
{
   ssafy_msgs::msg::dds_::HandControl_DataReader_ptr result;
   result = dynamic_cast < ssafy_msgs::msg::dds_::HandControl_DataReader_ptr> (p);
   if (result) result->m_count++;
   return result;
}

const char * ssafy_msgs::msg::dds_::HandControl_DataReaderView::_local_id = "IDL:ssafy_msgs/msg/dds_/HandControl_DataReaderView:1.0";

ssafy_msgs::msg::dds_::HandControl_DataReaderView_ptr ssafy_msgs::msg::dds_::HandControl_DataReaderView::_duplicate (ssafy_msgs::msg::dds_::HandControl_DataReaderView_ptr p)
{
   if (p) p->m_count++;
   return p;
}

DDS::Boolean ssafy_msgs::msg::dds_::HandControl_DataReaderView::_local_is_a (const char * _id)
{
   if (strcmp (_id, ssafy_msgs::msg::dds_::HandControl_DataReaderView::_local_id) == 0)
   {
      return true;
   }

   typedef DDS::DataReaderView NestedBase_1;

   if (NestedBase_1::_local_is_a (_id))
   {
      return true;
   }

   return false;
}

ssafy_msgs::msg::dds_::HandControl_DataReaderView_ptr ssafy_msgs::msg::dds_::HandControl_DataReaderView::_narrow (DDS::Object_ptr p)
{
   ssafy_msgs::msg::dds_::HandControl_DataReaderView_ptr result = NULL;
   if (p && p->_is_a (ssafy_msgs::msg::dds_::HandControl_DataReaderView::_local_id))
   {
      result = dynamic_cast < ssafy_msgs::msg::dds_::HandControl_DataReaderView_ptr> (p);
      if (result) result->m_count++;
   }
   return result;
}

ssafy_msgs::msg::dds_::HandControl_DataReaderView_ptr ssafy_msgs::msg::dds_::HandControl_DataReaderView::_unchecked_narrow (DDS::Object_ptr p)
{
   ssafy_msgs::msg::dds_::HandControl_DataReaderView_ptr result;
   result = dynamic_cast < ssafy_msgs::msg::dds_::HandControl_DataReaderView_ptr> (p);
   if (result) result->m_count++;
   return result;
}



