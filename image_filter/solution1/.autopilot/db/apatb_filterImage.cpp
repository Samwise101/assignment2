#include <systemc>
#include <iostream>
#include <cstdlib>
#include <cstddef>
#include <stdint.h>
#include "SysCFileHandler.h"
#include "ap_int.h"
#include "ap_fixed.h"
#include <complex>
#include <stdbool.h>
#include "autopilot_cbe.h"
#include "hls_stream.h"
#include "hls_half.h"
#include "hls_signal_handler.h"

using namespace std;
using namespace sc_core;
using namespace sc_dt;

// wrapc file define:
#define AUTOTB_TVIN_rowBelow "../tv/cdatafile/c.filterImage.autotvin_rowBelow.dat"
#define AUTOTB_TVOUT_rowBelow "../tv/cdatafile/c.filterImage.autotvout_rowBelow.dat"
// wrapc file define:
#define AUTOTB_TVIN_rowCenter "../tv/cdatafile/c.filterImage.autotvin_rowCenter.dat"
#define AUTOTB_TVOUT_rowCenter "../tv/cdatafile/c.filterImage.autotvout_rowCenter.dat"
// wrapc file define:
#define AUTOTB_TVIN_rowAbove "../tv/cdatafile/c.filterImage.autotvin_rowAbove.dat"
#define AUTOTB_TVOUT_rowAbove "../tv/cdatafile/c.filterImage.autotvout_rowAbove.dat"
// wrapc file define:
#define AUTOTB_TVIN_outputRow "../tv/cdatafile/c.filterImage.autotvin_outputRow.dat"
#define AUTOTB_TVOUT_outputRow "../tv/cdatafile/c.filterImage.autotvout_outputRow.dat"

#define INTER_TCL "../tv/cdatafile/ref.tcl"

// tvout file define:
#define AUTOTB_TVOUT_PC_rowBelow "../tv/rtldatafile/rtl.filterImage.autotvout_rowBelow.dat"
// tvout file define:
#define AUTOTB_TVOUT_PC_rowCenter "../tv/rtldatafile/rtl.filterImage.autotvout_rowCenter.dat"
// tvout file define:
#define AUTOTB_TVOUT_PC_rowAbove "../tv/rtldatafile/rtl.filterImage.autotvout_rowAbove.dat"
// tvout file define:
#define AUTOTB_TVOUT_PC_outputRow "../tv/rtldatafile/rtl.filterImage.autotvout_outputRow.dat"

class INTER_TCL_FILE {
  public:
INTER_TCL_FILE(const char* name) {
  mName = name; 
  rowBelow_depth = 0;
  rowCenter_depth = 0;
  rowAbove_depth = 0;
  outputRow_depth = 0;
  trans_num =0;
}
~INTER_TCL_FILE() {
  mFile.open(mName);
  if (!mFile.good()) {
    cout << "Failed to open file ref.tcl" << endl;
    exit (1); 
  }
  string total_list = get_depth_list();
  mFile << "set depth_list {\n";
  mFile << total_list;
  mFile << "}\n";
  mFile << "set trans_num "<<trans_num<<endl;
  mFile.close();
}
string get_depth_list () {
  stringstream total_list;
  total_list << "{rowBelow " << rowBelow_depth << "}\n";
  total_list << "{rowCenter " << rowCenter_depth << "}\n";
  total_list << "{rowAbove " << rowAbove_depth << "}\n";
  total_list << "{outputRow " << outputRow_depth << "}\n";
  return total_list.str();
}
void set_num (int num , int* class_num) {
  (*class_num) = (*class_num) > num ? (*class_num) : num;
}
void set_string(std::string list, std::string* class_list) {
  (*class_list) = list;
}
  public:
    int rowBelow_depth;
    int rowCenter_depth;
    int rowAbove_depth;
    int outputRow_depth;
    int trans_num;
  private:
    ofstream mFile;
    const char* mName;
};

static void RTLOutputCheckAndReplacement(std::string &AESL_token, std::string PortName) {
  bool no_x = false;
  bool err = false;

  no_x = false;
  // search and replace 'X' with '0' from the 3rd char of token
  while (!no_x) {
    size_t x_found = AESL_token.find('X', 0);
    if (x_found != string::npos) {
      if (!err) { 
        cerr << "WARNING: [SIM 212-201] RTL produces unknown value 'X' on port" 
             << PortName << ", possible cause: There are uninitialized variables in the C design."
             << endl; 
        err = true;
      }
      AESL_token.replace(x_found, 1, "0");
    } else
      no_x = true;
  }
  no_x = false;
  // search and replace 'x' with '0' from the 3rd char of token
  while (!no_x) {
    size_t x_found = AESL_token.find('x', 2);
    if (x_found != string::npos) {
      if (!err) { 
        cerr << "WARNING: [SIM 212-201] RTL produces unknown value 'x' on port" 
             << PortName << ", possible cause: There are uninitialized variables in the C design."
             << endl; 
        err = true;
      }
      AESL_token.replace(x_found, 1, "0");
    } else
      no_x = true;
  }
}
extern "C" void filterImage_hw_stub_wrapper(volatile void *, volatile void *, volatile void *, volatile void *);

extern "C" void apatb_filterImage_hw(volatile void * __xlx_apatb_param_rowBelow, volatile void * __xlx_apatb_param_rowCenter, volatile void * __xlx_apatb_param_rowAbove, volatile void * __xlx_apatb_param_outputRow) {
  refine_signal_handler();
  fstream wrapc_switch_file_token;
  wrapc_switch_file_token.open(".hls_cosim_wrapc_switch.log");
  int AESL_i;
  if (wrapc_switch_file_token.good())
  {

    CodeState = ENTER_WRAPC_PC;
    static unsigned AESL_transaction_pc = 0;
    string AESL_token;
    string AESL_num;{
      static ifstream rtl_tv_out_file;
      if (!rtl_tv_out_file.is_open()) {
        rtl_tv_out_file.open(AUTOTB_TVOUT_PC_outputRow);
        if (rtl_tv_out_file.good()) {
          rtl_tv_out_file >> AESL_token;
          if (AESL_token != "[[[runtime]]]")
            exit(1);
        }
      }
  
      if (rtl_tv_out_file.good()) {
        rtl_tv_out_file >> AESL_token; 
        rtl_tv_out_file >> AESL_num;  // transaction number
        if (AESL_token != "[[transaction]]") {
          cerr << "Unexpected token: " << AESL_token << endl;
          exit(1);
        }
        if (atoi(AESL_num.c_str()) == AESL_transaction_pc) {
          std::vector<sc_bv<8> > outputRow_pc_buffer(120);
          int i = 0;

          rtl_tv_out_file >> AESL_token; //data
          while (AESL_token != "[[/transaction]]"){

            RTLOutputCheckAndReplacement(AESL_token, "outputRow");
  
            // push token into output port buffer
            if (AESL_token != "") {
              outputRow_pc_buffer[i] = AESL_token.c_str();;
              i++;
            }
  
            rtl_tv_out_file >> AESL_token; //data or [[/transaction]]
            if (AESL_token == "[[[/runtime]]]" || rtl_tv_out_file.eof())
              exit(1);
          }
          if (i > 0) {{
            int i = 0;
            for (int j = 0, e = 120; j < e; j += 1, ++i) {
            ((char*)__xlx_apatb_param_outputRow)[j] = outputRow_pc_buffer[i].to_int64();
          }}}
        } // end transaction
      } // end file is good
    } // end post check logic bolck
  
    AESL_transaction_pc++;
    return ;
  }
static unsigned AESL_transaction;
static AESL_FILE_HANDLER aesl_fh;
static INTER_TCL_FILE tcl_file(INTER_TCL);
std::vector<char> __xlx_sprintf_buffer(1024);
CodeState = ENTER_WRAPC;
//rowBelow
aesl_fh.touch(AUTOTB_TVIN_rowBelow);
aesl_fh.touch(AUTOTB_TVOUT_rowBelow);
//rowCenter
aesl_fh.touch(AUTOTB_TVIN_rowCenter);
aesl_fh.touch(AUTOTB_TVOUT_rowCenter);
//rowAbove
aesl_fh.touch(AUTOTB_TVIN_rowAbove);
aesl_fh.touch(AUTOTB_TVOUT_rowAbove);
//outputRow
aesl_fh.touch(AUTOTB_TVIN_outputRow);
aesl_fh.touch(AUTOTB_TVOUT_outputRow);
CodeState = DUMP_INPUTS;
unsigned __xlx_offset_byte_param_rowBelow = 0;
// print rowBelow Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_rowBelow, __xlx_sprintf_buffer.data());
  {  __xlx_offset_byte_param_rowBelow = 0*1;
  if (__xlx_apatb_param_rowBelow) {
    for (int j = 0  - 0, e = 120 - 0; j != e; ++j) {
sc_bv<8> __xlx_tmp_lv = ((char*)__xlx_apatb_param_rowBelow)[j];

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_rowBelow, __xlx_sprintf_buffer.data()); 
      }
  }
}
  tcl_file.set_num(120, &tcl_file.rowBelow_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_rowBelow, __xlx_sprintf_buffer.data());
}
unsigned __xlx_offset_byte_param_rowCenter = 0;
// print rowCenter Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_rowCenter, __xlx_sprintf_buffer.data());
  {  __xlx_offset_byte_param_rowCenter = 0*1;
  if (__xlx_apatb_param_rowCenter) {
    for (int j = 0  - 0, e = 120 - 0; j != e; ++j) {
sc_bv<8> __xlx_tmp_lv = ((char*)__xlx_apatb_param_rowCenter)[j];

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_rowCenter, __xlx_sprintf_buffer.data()); 
      }
  }
}
  tcl_file.set_num(120, &tcl_file.rowCenter_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_rowCenter, __xlx_sprintf_buffer.data());
}
unsigned __xlx_offset_byte_param_rowAbove = 0;
// print rowAbove Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_rowAbove, __xlx_sprintf_buffer.data());
  {  __xlx_offset_byte_param_rowAbove = 0*1;
  if (__xlx_apatb_param_rowAbove) {
    for (int j = 0  - 0, e = 120 - 0; j != e; ++j) {
sc_bv<8> __xlx_tmp_lv = ((char*)__xlx_apatb_param_rowAbove)[j];

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_rowAbove, __xlx_sprintf_buffer.data()); 
      }
  }
}
  tcl_file.set_num(120, &tcl_file.rowAbove_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_rowAbove, __xlx_sprintf_buffer.data());
}
unsigned __xlx_offset_byte_param_outputRow = 0;
// print outputRow Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_outputRow, __xlx_sprintf_buffer.data());
  {  __xlx_offset_byte_param_outputRow = 0*1;
  if (__xlx_apatb_param_outputRow) {
    for (int j = 0  - 0, e = 120 - 0; j != e; ++j) {
sc_bv<8> __xlx_tmp_lv = ((char*)__xlx_apatb_param_outputRow)[j];

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_outputRow, __xlx_sprintf_buffer.data()); 
      }
  }
}
  tcl_file.set_num(120, &tcl_file.outputRow_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_outputRow, __xlx_sprintf_buffer.data());
}
CodeState = CALL_C_DUT;
filterImage_hw_stub_wrapper(__xlx_apatb_param_rowBelow, __xlx_apatb_param_rowCenter, __xlx_apatb_param_rowAbove, __xlx_apatb_param_outputRow);
CodeState = DUMP_OUTPUTS;
// print outputRow Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVOUT_outputRow, __xlx_sprintf_buffer.data());
  {  __xlx_offset_byte_param_outputRow = 0*1;
  if (__xlx_apatb_param_outputRow) {
    for (int j = 0  - 0, e = 120 - 0; j != e; ++j) {
sc_bv<8> __xlx_tmp_lv = ((char*)__xlx_apatb_param_outputRow)[j];

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVOUT_outputRow, __xlx_sprintf_buffer.data()); 
      }
  }
}
  tcl_file.set_num(120, &tcl_file.outputRow_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVOUT_outputRow, __xlx_sprintf_buffer.data());
}
CodeState = DELETE_CHAR_BUFFERS;
AESL_transaction++;
tcl_file.set_num(AESL_transaction , &tcl_file.trans_num);
}
