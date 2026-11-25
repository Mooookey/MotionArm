#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace robot {
// Response-only queue: an action is never retried when UART is backpressured.
// Main loop owns this queue. No callback, allocation, blocking or partial line.
class ManagementOutbox {
 public:
  static constexpr unsigned capacity=4, wireLimit=512;
  bool empty() const { return count_==0; }
  bool full() const { return count_==capacity; }
  unsigned count() const { return count_; }
  void clear() { head_=count_=0; }
  bool enqueue(const char *json,size_t length) {
    if(!json || !length || length>=wireLimit || full())return false;
    auto &line=lines_[(head_+count_)%capacity];
    std::memcpy(line.data,json,length);line.data[length]='\n';line.length=uint16_t(length+1);++count_;return true;
  }
  const char *data() const { return empty()?nullptr:lines_[head_].data; }
  size_t length() const { return empty()?0:lines_[head_].length; }
  void pop() { if(count_){head_=(head_+1)%capacity;--count_;} }
 private:
  struct Line { char data[wireLimit]{};uint16_t length=0; } lines_[capacity];
  unsigned head_=0,count_=0;
};
}
