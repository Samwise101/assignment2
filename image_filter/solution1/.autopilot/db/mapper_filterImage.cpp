#include <systemc>
#include <vector>
#include <iostream>
#include "hls_stream.h"
#include "ap_int.h"
#include "ap_fixed.h"
using namespace std;
using namespace sc_dt;
class AESL_RUNTIME_BC {
  public:
    AESL_RUNTIME_BC(const char* name) {
      file_token.open( name);
      if (!file_token.good()) {
        cout << "Failed to open tv file " << name << endl;
        exit (1);
      }
      file_token >> mName;//[[[runtime]]]
    }
    ~AESL_RUNTIME_BC() {
      file_token.close();
    }
    int read_size () {
      int size = 0;
      file_token >> mName;//[[transaction]]
      file_token >> mName;//transaction number
      file_token >> mName;//pop_size
      size = atoi(mName.c_str());
      file_token >> mName;//[[/transaction]]
      return size;
    }
  public:
    fstream file_token;
    string mName;
};
extern "C" void filterImage(char*, char*, char*, char*);
extern "C" void apatb_filterImage_hw(volatile void * __xlx_apatb_param_rowBelow, volatile void * __xlx_apatb_param_rowCenter, volatile void * __xlx_apatb_param_rowAbove, volatile void * __xlx_apatb_param_outputRow) {
  // Collect __xlx_rowBelow__tmp_vec
  vector<sc_bv<8> >__xlx_rowBelow__tmp_vec;
  for (int j = 0, e = 120; j != e; ++j) {
    __xlx_rowBelow__tmp_vec.push_back(((char*)__xlx_apatb_param_rowBelow)[j]);
  }
  int __xlx_size_param_rowBelow = 120;
  int __xlx_offset_param_rowBelow = 0;
  int __xlx_offset_byte_param_rowBelow = 0*1;
  char* __xlx_rowBelow__input_buffer= new char[__xlx_rowBelow__tmp_vec.size()];
  for (int i = 0; i < __xlx_rowBelow__tmp_vec.size(); ++i) {
    __xlx_rowBelow__input_buffer[i] = __xlx_rowBelow__tmp_vec[i].range(7, 0).to_uint64();
  }
  // Collect __xlx_rowCenter__tmp_vec
  vector<sc_bv<8> >__xlx_rowCenter__tmp_vec;
  for (int j = 0, e = 120; j != e; ++j) {
    __xlx_rowCenter__tmp_vec.push_back(((char*)__xlx_apatb_param_rowCenter)[j]);
  }
  int __xlx_size_param_rowCenter = 120;
  int __xlx_offset_param_rowCenter = 0;
  int __xlx_offset_byte_param_rowCenter = 0*1;
  char* __xlx_rowCenter__input_buffer= new char[__xlx_rowCenter__tmp_vec.size()];
  for (int i = 0; i < __xlx_rowCenter__tmp_vec.size(); ++i) {
    __xlx_rowCenter__input_buffer[i] = __xlx_rowCenter__tmp_vec[i].range(7, 0).to_uint64();
  }
  // Collect __xlx_rowAbove__tmp_vec
  vector<sc_bv<8> >__xlx_rowAbove__tmp_vec;
  for (int j = 0, e = 120; j != e; ++j) {
    __xlx_rowAbove__tmp_vec.push_back(((char*)__xlx_apatb_param_rowAbove)[j]);
  }
  int __xlx_size_param_rowAbove = 120;
  int __xlx_offset_param_rowAbove = 0;
  int __xlx_offset_byte_param_rowAbove = 0*1;
  char* __xlx_rowAbove__input_buffer= new char[__xlx_rowAbove__tmp_vec.size()];
  for (int i = 0; i < __xlx_rowAbove__tmp_vec.size(); ++i) {
    __xlx_rowAbove__input_buffer[i] = __xlx_rowAbove__tmp_vec[i].range(7, 0).to_uint64();
  }
  // Collect __xlx_outputRow__tmp_vec
  vector<sc_bv<8> >__xlx_outputRow__tmp_vec;
  for (int j = 0, e = 120; j != e; ++j) {
    __xlx_outputRow__tmp_vec.push_back(((char*)__xlx_apatb_param_outputRow)[j]);
  }
  int __xlx_size_param_outputRow = 120;
  int __xlx_offset_param_outputRow = 0;
  int __xlx_offset_byte_param_outputRow = 0*1;
  char* __xlx_outputRow__input_buffer= new char[__xlx_outputRow__tmp_vec.size()];
  for (int i = 0; i < __xlx_outputRow__tmp_vec.size(); ++i) {
    __xlx_outputRow__input_buffer[i] = __xlx_outputRow__tmp_vec[i].range(7, 0).to_uint64();
  }
  // DUT call
  filterImage(__xlx_rowBelow__input_buffer, __xlx_rowCenter__input_buffer, __xlx_rowAbove__input_buffer, __xlx_outputRow__input_buffer);
// print __xlx_apatb_param_rowBelow
  sc_bv<8>*__xlx_rowBelow_output_buffer = new sc_bv<8>[__xlx_size_param_rowBelow];
  for (int i = 0; i < __xlx_size_param_rowBelow; ++i) {
    __xlx_rowBelow_output_buffer[i] = __xlx_rowBelow__input_buffer[i+__xlx_offset_param_rowBelow];
  }
  for (int i = 0; i < __xlx_size_param_rowBelow; ++i) {
    ((char*)__xlx_apatb_param_rowBelow)[i] = __xlx_rowBelow_output_buffer[i].to_uint64();
  }
// print __xlx_apatb_param_rowCenter
  sc_bv<8>*__xlx_rowCenter_output_buffer = new sc_bv<8>[__xlx_size_param_rowCenter];
  for (int i = 0; i < __xlx_size_param_rowCenter; ++i) {
    __xlx_rowCenter_output_buffer[i] = __xlx_rowCenter__input_buffer[i+__xlx_offset_param_rowCenter];
  }
  for (int i = 0; i < __xlx_size_param_rowCenter; ++i) {
    ((char*)__xlx_apatb_param_rowCenter)[i] = __xlx_rowCenter_output_buffer[i].to_uint64();
  }
// print __xlx_apatb_param_rowAbove
  sc_bv<8>*__xlx_rowAbove_output_buffer = new sc_bv<8>[__xlx_size_param_rowAbove];
  for (int i = 0; i < __xlx_size_param_rowAbove; ++i) {
    __xlx_rowAbove_output_buffer[i] = __xlx_rowAbove__input_buffer[i+__xlx_offset_param_rowAbove];
  }
  for (int i = 0; i < __xlx_size_param_rowAbove; ++i) {
    ((char*)__xlx_apatb_param_rowAbove)[i] = __xlx_rowAbove_output_buffer[i].to_uint64();
  }
// print __xlx_apatb_param_outputRow
  sc_bv<8>*__xlx_outputRow_output_buffer = new sc_bv<8>[__xlx_size_param_outputRow];
  for (int i = 0; i < __xlx_size_param_outputRow; ++i) {
    __xlx_outputRow_output_buffer[i] = __xlx_outputRow__input_buffer[i+__xlx_offset_param_outputRow];
  }
  for (int i = 0; i < __xlx_size_param_outputRow; ++i) {
    ((char*)__xlx_apatb_param_outputRow)[i] = __xlx_outputRow_output_buffer[i].to_uint64();
  }
}
