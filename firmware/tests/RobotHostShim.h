#pragma once
// Host-only device transport doubles. robot_management_test.cpp executes the
// actual sketch dispatch/loop, not a second implementation of its policy.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
using std::isfinite;
inline uint32_t hostNow=0;
inline uint32_t millis(){return hostNow;}
inline void delay(unsigned ms){hostNow+=ms;}
inline uint32_t esp_random(){static uint32_t n=1;return ++n;}
constexpr int UART_CLK_SRC_APB=1;
struct HostSerial {
  std::string output,input;size_t cursor=0;
  int txCapacity=8192;
  int clockSource=-1;bool begun=false;
  bool setClockSource(int source){if(begun)return false;clockSource=source;return true;}
  void setTxBufferSize(unsigned){} void begin(unsigned){begun=true;}
  int availableForWrite(){return txCapacity;}int available(){return int(input.size()-cursor);}char read(){return input[cursor++];}
  size_t write(const uint8_t *p,size_t n){output.append(reinterpret_cast<const char*>(p),n);return n;}
  size_t write(char c){output+=c;return 1;}
} inline Serial;
struct HostWire {
  uint8_t registers[256]{},pointer=0;std::vector<uint8_t> data;
  bool ok=true;unsigned effectiveWrites=0,fullOffWrites=0;
  void begin(int,int,int){}void setTimeOut(int){}
  void beginTransmission(uint8_t){data.clear();}
  size_t write(uint8_t n){data.push_back(n);return 1;}
  int endTransmission(bool=true){
    if(!ok)return 4;
    if(data.empty())return 0;pointer=data[0];
    for(size_t i=1;i<data.size();++i)registers[uint8_t(pointer+i-1)]=data[i];
    if(pointer==54 && data.size()==17){
      if(data[4]&0x10)++fullOffWrites;else ++effectiveWrites;
    }
    return 0;
  }
  int requestFrom(uint8_t,int n){return ok?n:0;}
  int read(){return registers[pointer++];}
} inline Wire;
constexpr int WIFI_STA=1,WIFI_IF_STA=0,WIFI_SECOND_CHAN_NONE=0,ESP_OK=0,pdTRUE=1;
struct HostWiFi{void mode(int){}}inline WiFi;
inline int esp_wifi_get_mac(int,uint8_t *p){const uint8_t m[]={0xC0,0xCD,0xD6,0xD0,0x9B,0x54};std::memcpy(p,m,6);return 0;}
inline int esp_wifi_set_channel(unsigned,int){return 0;}
struct wifi_pkt_rx_ctrl_t{int rssi;};
struct esp_now_recv_info_t{const uint8_t *src_addr;wifi_pkt_rx_ctrl_t *rx_ctrl;};
inline int esp_now_init(){return 0;}
inline int esp_now_register_recv_cb(void(*)(const esp_now_recv_info_t*,const uint8_t*,int)){return 0;}
using QueueHandle_t=void*;
inline void *xQueueCreate(unsigned,unsigned){return reinterpret_cast<void*>(1);}
inline void xQueueOverwrite(void*,const void*){}
inline int xQueueReceive(void*,void*,unsigned){return 0;}
struct cJSON {
  enum Kind {Object,Number,String,True,False,Null,Array}type=Object;
  cJSON *child=nullptr,*next=nullptr;
  char *string=nullptr,*valuestring=nullptr;double valuedouble=0;
};
inline bool cJSON_IsString(const cJSON *p){return p&&p->type==cJSON::String;}
inline bool cJSON_IsNumber(const cJSON *p){return p&&p->type==cJSON::Number;}
inline bool cJSON_IsTrue(const cJSON *p){return p&&p->type==cJSON::True;}
inline bool cJSON_IsObject(const cJSON *p){return p&&p->type==cJSON::Object;}
inline bool cJSON_IsArray(const cJSON *p){return p&&p->type==cJSON::Array;}
inline const cJSON *cJSON_GetObjectItemCaseSensitive(const cJSON *p,const char *key){
  if(p)for(auto *n=p->child;n;n=n->next)if(n->string&&!std::strcmp(n->string,key))return n;return nullptr;
}
inline char *hostCopy(const char *p){const size_t n=std::strlen(p)+1;auto *r=static_cast<char*>(std::malloc(n));std::memcpy(r,p,n);return r;}
inline char *cJSON_PrintUnformatted(const cJSON *p){
  if(cJSON_IsString(p))return hostCopy((std::string("\"")+p->valuestring+"\"").c_str());
  if(cJSON_IsNumber(p)){char b[64];std::snprintf(b,sizeof(b),"%.17g",p->valuedouble);return hostCopy(b);}
  return hostCopy(cJSON_IsTrue(p)?"true":p&&p->type==cJSON::False?"false":"null");
}
inline void cJSON_free(void *p){std::free(p);}
inline void cJSON_Delete(cJSON *p){if(!p)return;for(auto *n=p->child;n;){auto *next=n->next;cJSON_Delete(n);n=next;}std::free(p->string);std::free(p->valuestring);delete p;}
inline unsigned hostParseCalls=0;
inline cJSON *cJSON_ParseWithOpts(const char*,const char**,bool){++hostParseCalls;return nullptr;}
